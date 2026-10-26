#!/usr/bin/env python3
"""Browser 3D viewer for PL-EVIO poses (stella_vslam socket_viewer style).

Read-only ROS2 node: subscribes to the estimator/pose_graph topics and serves a three.js
page over plain HTTP (no extra Python deps; the page polls /state). Run on the Jetson:

    python3 evio_pose_viewer.py            # then browse to http://<jetson>:3002/

Shows: live camera frustum, VIO trajectory, loop-closed pose-graph path, map points.
The viewing browser needs internet access for the three.js CDN scripts.
"""
import json
import os
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from collections import deque
import time
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Bool, Float64MultiArray

PORT = 3002
MAX_PATH_PTS = 3000
MAX_CLOUD_PTS = 4000

state = {"odom": None, "path": [], "pg_path": [], "cloud": [], "n_odom": 0, "diag": None, "cam": None}
lock = threading.Lock()
node_ref = [None]
logs = deque(maxlen=300)
log_seq = [0]
last_diag = [0.0]


def path_pts(msg):
    pts = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
    step = max(1, len(pts) // MAX_PATH_PTS)
    return pts[::step]


def path_pts_from(msg, skip):
    pts = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses[skip:]]
    step = max(1, len(pts) // MAX_PATH_PTS)
    return pts[::step]


class Viewer(Node):
    def __init__(self):
        super().__init__("evio_pose_viewer")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, "/evio_estimator/odometry", self.on_odom, qos)
        self.create_subscription(Odometry, "/evio_estimator/camera_pose", self.on_cam, qos)
        self.create_subscription(Path, "/evio_estimator/path", self.on_path, qos)
        self.create_subscription(PointCloud, "/evio_estimator/point_cloud", self.on_cloud, qos)
        self.create_subscription(Path, "/pose_graph/pose_graph_path", self.on_pg, qos)
        self.create_subscription(Float64MultiArray, "/evio_estimator/diag", self.on_diag, qos)
        self.pub_reset = self.create_publisher(Bool, "/feature_tracker/restart", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.path_skip = 0
        self.path_len = 0
        self.pg_skip = 0
        self.pg_len = 0
        node_ref[0] = self
        self.get_logger().info(f"viewer on http://0.0.0.0:{PORT}/")

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with lock:
            state["odom"] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            state["n_odom"] += 1

    def on_path(self, m):
        # the estimator's Path keeps every pose across a restart; skip those from before the last reset
        with lock:
            self.path_len = len(m.poses)
            state["path"] = path_pts_from(m, self.path_skip)

    def reset(self):
        """Restart the estimator (clears buffers + state, re-initialises) and drop the old trajectory."""
        with lock:
            self.path_skip = self.path_len
            self.pg_skip = self.pg_len
            state["path"], state["pg_path"], state["cloud"] = [], [], []
            state["resets"] = state.get("resets", 0) + 1
        self.pub_reset.publish(Bool(data=True))

    def on_pg(self, m):
        with lock:
            self.pg_len = len(m.poses)
            state["pg_path"] = path_pts_from(m, self.pg_skip)

    def on_cam(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with lock:
            state["cam"] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

    def on_diag(self, m):
        with lock:
            state["diag"] = list(m.data)
            last_diag[0] = time.time()

    def on_log(self, m):
        with lock:
            log_seq[0] += 1
            logs.append([log_seq[0], m.level, m.name, m.msg])

    def on_cloud(self, m):
        pts = [[p.x, p.y, p.z] for p in m.points]
        step = max(1, len(pts) // MAX_CLOUD_PTS)
        with lock:
            state["cloud"] = pts[::step]


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>PL-EVIO pose viewer</title>
<style>html,body{margin:0;height:100%;background:#111;color:#ddd;font:13px monospace}
#hud{position:fixed;top:8px;left:8px;background:#0008;padding:8px;line-height:1.5}
button{margin-top:4px}</style></head><body>
<div id="hud"><b>PL-EVIO poses</b><br>
<span style="color:#0f0">&#9632;</span> VIO path &nbsp;<span style="color:#f0f">&#9632;</span> pose-graph path &nbsp;<span style="color:#ff0">&#9632;</span> map pts<br>
<span id="info">waiting for data...</span><br><span id="diag" style="color:#8cf"></span><br><span id="alive"></span><br>
<button onclick="fit()">fit view</button> <button id="rst" onclick="doReset()" style="background:#a22;color:#fff">reset init</button> <button id="rn" onclick="doRestart()" style="background:#a60;color:#fff">restart node</button> <label><input type="checkbox" id="follow"> follow</label>
<br><label><input type="checkbox" id="showlog" onchange="logbox.style.display=this.checked?'block':'none'"> logs</label>
<select id="logf"><option value="0">all</option><option value="30" selected>warn+</option><option value="EST">estimator</option></select>
<div id="logbox" style="display:none;width:520px;max-width:60vw;height:32vh;overflow:auto;font-size:11px;line-height:1.35;background:#000a;margin-top:4px;padding:4px;white-space:pre-wrap"></div></div>
<div id="cams"><div class="cam"><div>mono (APS)</div><img id="c1"></div><div class="cam"><div>events</div><img id="c2"></div><div class="cam"><div>image tracker</div><img id="c3"></div></div>
<style>#cams{position:fixed;top:8px;right:8px;display:flex;flex-direction:column;gap:6px}.cam{background:#0008;padding:4px}.cam img{width:260px;display:block}.cam div{font-size:11px}</style>
<script>const H=location.hostname,S=t=>`http://${H}:8080/stream?type=mjpeg&quality=70&topic=${t}`;c1.src=S("/davis346/image_raw");c2.src=S("/davis346/renderer/image_raw");c3.src=S("/image_feature_tracker/feature_img")</script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script>
THREE.Object3D.DefaultUp.set(0,0,1);
const scene=new THREE.Scene(), cam=new THREE.PerspectiveCamera(60,innerWidth/innerHeight,0.01,1e6);
const r=new THREE.WebGLRenderer({antialias:true});r.setSize(innerWidth,innerHeight);document.body.appendChild(r.domElement);
cam.position.set(2,-3,2);const ctl=new THREE.OrbitControls(cam,r.domElement);
scene.add(new THREE.GridHelper(20,20).rotateX(Math.PI/2));scene.add(new THREE.AxesHelper(1));
function line(c){const g=new THREE.BufferGeometry();const l=new THREE.Line(g,new THREE.LineBasicMaterial({color:c}));l.frustumCulled=false;scene.add(l);return l}
const vio=line(0x00ff00), pg=line(0xff00ff);
const cloud=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({color:0xffff00,size:3,sizeAttenuation:false}));cloud.frustumCulled=false;scene.add(cloud);
const f=new THREE.Group();const fg=new THREE.BufferGeometry();
const a=[0,0,0, 1,.6,1, 0,0,0, -1,.6,1, 0,0,0, 1,-.6,1, 0,0,0, -1,-.6,1, 1,.6,1, -1,.6,1, -1,.6,1, -1,-.6,1, -1,-.6,1, 1,-.6,1, 1,-.6,1, 1,.6,1];
fg.setAttribute('position',new THREE.Float32BufferAttribute(a,3));f.add(new THREE.LineSegments(fg,new THREE.LineBasicMaterial({color:0xff4040})));scene.add(f);
let pts=[],last=null,logSeq=0,logLines=[];
const LC={10:'#888',20:'#ccc',30:'#fc4',40:'#f66',50:'#f3f'};
function esc(s){return s.replace(/&/g,'&amp;').replace(/</g,'&lt;')}
function drawLogs(){const f=document.getElementById('logf').value;const b=document.getElementById('logbox');
 const keep=logLines.filter(l=>f=='EST'?l[2].includes('evio_estimator'):l[1]>=+f);
 const stick=b.scrollTop+b.clientHeight>=b.scrollHeight-8;
 b.innerHTML=keep.slice(-120).map(l=>'<span style="color:'+(LC[l[1]]||'#ccc')+'">['+l[2]+'] '+esc(l[3])+'</span>').join('\n');
 if(stick)b.scrollTop=b.scrollHeight}
document.addEventListener('change',e=>{if(e.target.id=='logf')drawLogs()});
async function pollLogs(){try{if(document.getElementById('showlog').checked){const r=await (await fetch('/logs?since='+logSeq)).json();
 if(r.logs.length){logSeq=r.seq;logLines=logLines.concat(r.logs).slice(-400);drawLogs()}}}catch(e){}setTimeout(pollLogs,700)}
pollLogs();
async function doRestart(){const b=document.getElementById('rn');b.disabled=true;b.textContent='restarting...';await fetch('/restart_node',{method:'POST'});first=true;setTimeout(()=>{b.disabled=false;b.textContent='restart node'},4000)}
async function doReset(){const b=document.getElementById('rst');b.disabled=true;await fetch('/reset',{method:'POST'});first=true;setTimeout(()=>b.disabled=false,1500)}
function setLine(l,p){l.geometry.setAttribute('position',new THREE.Float32BufferAttribute(p.flat(),3));l.geometry.computeBoundingSphere()}
function fit(){const all=pts.concat(last?[last.slice(0,3)]:[]);if(!all.length)return;
 const b=new THREE.Box3().setFromPoints(all.map(p=>new THREE.Vector3(...p)));const c=b.getCenter(new THREE.Vector3()),s=Math.max(b.getSize(new THREE.Vector3()).length(),0.5);
 ctl.target.copy(c);cam.position.copy(c).add(new THREE.Vector3(s,-s,s*.7));cam.far=s*100;cam.updateProjectionMatrix()}
let first=true;
async function poll(){try{const s=await (await fetch('/state')).json();
 {const a=document.getElementById('alive'),age=s.diag_age;
  a.innerHTML=age==null?'<span style="color:#f55">estimator: no output yet</span>':(age>3?'<span style="color:#f55">estimator: NO OUTPUT for '+age.toFixed(0)+' s (not initialised or crashed)</span>':'<span style="color:#6f6">estimator: alive</span>')}
 pts=s.path;setLine(vio,s.path);setLine(pg,s.pg_path);cloud.geometry.setAttribute('position',new THREE.Float32BufferAttribute(s.cloud.flat(),3));
 if(s.odom){last=s.odom;const c=s.cam||s.odom;f.position.set(...c.slice(0,3));f.quaternion.set(...c.slice(3));
  const sc=Math.max(0.05,(pts.length?new THREE.Box3().setFromPoints(pts.map(p=>new THREE.Vector3(...p))).getSize(new THREE.Vector3()).length():1)*0.03);f.scale.setScalar(sc);
  const d=Math.hypot(...s.odom.slice(0,3));
  document.getElementById('info').textContent=`odom msgs ${s.n_odom} | path ${s.path.length} | map pts ${s.cloud.length} | pos ${s.odom.slice(0,3).map(v=>v.toFixed(2)).join(', ')} | dist from origin ${d.toFixed(1)} m`;
  if(s.diag){const d=s.diag,f=(a)=>a.map(v=>v.toFixed(3)).join(', ');document.getElementById('diag').innerHTML='solver '+(d[0]==1?'NONLINEAR':'INIT')+' | td '+d[1].toFixed(4)+'s<br>acc bias ['+f(d.slice(2,5))+'] m/s2<br>gyro bias ['+f(d.slice(5,8))+'] rad/s<br>ext ypr ['+f(d.slice(8,11))+'] deg<br>ext t ['+f(d.slice(11,14))+'] m'}
  if(first){fit();first=false}
  if(document.getElementById('follow').checked)ctl.target.set(...s.odom.slice(0,3))}
 }catch(e){}setTimeout(poll,150)}
poll();(function loop(){requestAnimationFrame(loop);ctl.update();r.render(scene,cam)})();
addEventListener('resize',()=>{cam.aspect=innerWidth/innerHeight;cam.updateProjectionMatrix();r.setSize(innerWidth,innerHeight)});
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/state"):
            with lock:
                s = dict(state)
                s["diag_age"] = time.time() - last_diag[0] if last_diag[0] else None
                body = json.dumps(s).encode()
            ctype = "application/json"
        elif self.path.startswith("/logs"):
            since = int(self.path.split("since=")[-1]) if "since=" in self.path else 0
            with lock:
                body = json.dumps({"seq": log_seq[0], "logs": [l for l in logs if l[0] > since]}).encode()
            ctype = "application/json"
        else:
            body, ctype = PAGE.encode(), "text/html"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path.startswith("/reset") and node_ref[0] is not None:
            node_ref[0].reset()
            body = b"reset sent"
        elif self.path.startswith("/restart_node") and node_ref[0] is not None:
            # kill the estimator process; the est_loop.sh supervisor respawns it with fresh state
            out = subprocess.run(["pkill", "-x", "evio_estimator"], capture_output=True)
            node_ref[0].reset()                      # also drop the old path in the viewer
            body = b"estimator killed, respawning" if out.returncode == 0 else b"no estimator process found"
        else:
            body = b"unknown"
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


LOG_FILES = ["~/est.log", "~/pg.log", "~/rec_launch.log"]
LVL = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40, "FATAL": 50}


def add_log(line):
    # "[WARN] [1791.2] [evio_estimator]: msg" (ROS2) or "F1013 01:03:16 ... Check failed" (glog)
    line = line.rstrip()
    if not line:
        return
    lvl, name, msg = 20, "", line
    if line.startswith("["):
        tag = line[1:line.find("]")]
        if tag in LVL:
            lvl = LVL[tag]
        parts = line.split("] [", 2)
        if len(parts) == 3 and "]: " in parts[2]:
            name, msg = parts[2].split("]: ", 1)
        elif "]: " in line:
            name, msg = line.split("] ", 1)[1].split("]: ", 1)
            name = name.lstrip("[")
    elif line[:1] in "FE" and line[1:5].isdigit():
        lvl, name = (50 if line[0] == "F" else 40), "glog"
    elif line.startswith("[supervisor]"):
        lvl, name = 40, "supervisor"
    elif line.startswith("[ros2run]"):
        lvl, name = 40, "ros2run"
    with lock:
        log_seq[0] += 1
        logs.append([log_seq[0], lvl, name, msg])


def tail_files():
    handles = {}
    while True:
        for p in LOG_FILES:
            path = os.path.expanduser(p)
            try:
                size = os.path.getsize(path)
                h = handles.get(path)
                if h is None:
                    h = open(path, "r", errors="replace")
                    h.seek(max(0, size - 4000))        # start with a short backlog
                    h.readline()
                    handles[path] = h
                elif size < h.tell():                  # truncated/rotated
                    h.seek(0)
                for line in h.readlines():
                    add_log(line)
            except OSError:
                handles.pop(path, None)
        time.sleep(0.4)


def main():
    threading.Thread(target=tail_files, daemon=True).start()
    rclpy.init()
    node = Viewer()
    threading.Thread(target=ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
