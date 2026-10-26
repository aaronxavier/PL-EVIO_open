#!/usr/bin/env python3
"""Headless web checkerboard calibration for the DAVIS346 mono image.

Run on the Jetson (system numpy needed for cv2, so disable the user site):

    PYTHONNOUSERSITE=1 python3 calib_web.py   # then browse to http://<jetson>:3003/

Live view shows detected corners; Capture stores a view, Calibrate fits both a PINHOLE
(k1,k2,p1,p2) and a fisheye/KANNALA_BRANDT (cv2.fisheye) model and writes YAML snippets.
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

PORT = 3003
TOPIC = "/davis346/image_raw"
PATTERN = (8, 6)          # inner corners
SQUARE = 0.025            # m
OUT = os.path.expanduser("~/davis346_calib")
os.makedirs(OUT + "/frames", exist_ok=True)

lock = threading.Lock()
S = {"auto": True, "last_auto": 0.0, "prev": None, "raw": None, "jpg": None, "found": False, "ncap": 0, "msg": "", "size": None, "result": None}
obj_pts, img_pts = [], []
objp = np.zeros((PATTERN[0] * PATTERN[1], 1, 3), np.float64)
objp[:, 0, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE
last_corners = [None]


def to_gray(m):
    if m.encoding in ("mono16", "16UC1"):
        a16 = np.frombuffer(m.data, ">u2" if m.is_bigendian else "<u2").reshape(m.height, m.width)
        return (a16 >> 8).astype(np.uint8)
    a = np.frombuffer(m.data, np.uint8)
    if m.encoding in ("mono8", "8UC1"):
        return a.reshape(m.height, m.width).copy()
    c = a.reshape(m.height, m.width, 3)
    return cv2.cvtColor(c, cv2.COLOR_RGB2GRAY if m.encoding == "rgb8" else cv2.COLOR_BGR2GRAY)


class N(Node):
    def __init__(self):
        super().__init__("calib_web")
        self.create_subscription(Image, TOPIC, self.cb, QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.t = 0.0

    def cb(self, m):
        if time.time() - self.t < 0.1:
            return
        self.t = time.time()
        g = to_gray(m)
        found, c = cv2.findChessboardCorners(g, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            c = cv2.cornerSubPix(g, c, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        with lock:
            for pts in img_pts:                       # coverage of captured views
                for p in pts[::7, 0]:
                    cv2.circle(vis, (int(p[0]), int(p[1])), 1, (0, 200, 255), -1)
            if found:
                cv2.drawChessboardCorners(vis, PATTERN, c, True)
            vis = cv2.resize(vis, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
            S["jpg"] = cv2.imencode(".jpg", vis)[1].tobytes()
            S["raw"], S["found"], S["size"] = g, bool(found), (m.width, m.height)
            last_corners[0] = c if found else None
            # auto-capture: board held steady (low motion vs previous detection), new pose, rate-limited
            steady = found and S["prev"] is not None and np.abs(S["prev"] - c).mean() < 0.7
            S["prev"] = c if found else None
            go = S["auto"] and steady and time.time() - S["last_auto"] > 0.8
        if go:
            r = capture(min_diff=6.0)
            with lock:
                if r.startswith("captured"):
                    S["last_auto"] = time.time()
                    S["msg"] = "auto " + r


def capture(min_diff=3.0):
    with lock:
        if last_corners[0] is None:
            return "no board detected"
        c, g = last_corners[0].copy(), S["raw"].copy()
        # reject near-duplicate views
        for p in img_pts:
            if np.abs(p - c).mean() < min_diff:
                return "too similar to an earlier view, move the board"
        obj_pts.append(objp.copy())
        img_pts.append(c)
        S["ncap"] = len(img_pts)
    cv2.imwrite("%s/frames/v%03d.png" % (OUT, len(img_pts)), g)
    return "captured %d" % len(img_pts)


def calibrate():
    with lock:
        if len(img_pts) < 10:
            return {"error": "need at least 10 views, have %d" % len(img_pts)}
        w, h = S["size"]
        O = [o.astype(np.float32) for o in obj_pts]
        I = [i.astype(np.float32) for i in img_pts]
    res = {"views": len(I), "size": [w, h]}
    # pinhole, radial-tangential k1,k2,p1,p2 (matches model_type PINHOLE)
    rms, K, D, _, _ = cv2.calibrateCamera(O, I, (w, h), None, None, flags=cv2.CALIB_FIX_K3)
    res["pinhole"] = {"rms": rms, "fx": K[0, 0], "fy": K[1, 1], "cx": K[0, 2], "cy": K[1, 2],
                      "k1": D[0, 0], "k2": D[0, 1], "p1": D[0, 2], "p2": D[0, 3]}
    # fisheye equidistant k2..k5 (matches model_type KANNALA_BRANDT)
    try:
        K2, D2 = np.eye(3), np.zeros((4, 1))
        fl = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        O2 = [o.reshape(-1, 1, 3).astype(np.float64) for o in obj_pts]
        I2 = [i.reshape(-1, 1, 2).astype(np.float64) for i in img_pts]
        rms2, K2, D2, _, _ = cv2.fisheye.calibrate(O2, I2, (w, h), K2, D2, flags=fl)
        res["fisheye"] = {"rms": rms2, "fx": K2[0, 0], "fy": K2[1, 1], "cx": K2[0, 2], "cy": K2[1, 2],
                          "k2": D2[0, 0], "k3": D2[1, 0], "k4": D2[2, 0], "k5": D2[3, 0]}
    except cv2.error as e:
        res["fisheye"] = {"error": str(e).splitlines()[-1]}
    res = json.loads(json.dumps(res, default=float))
    with open(OUT + "/result.json", "w") as f:
        json.dump(res, f, indent=1)
    S["result"] = res
    return res


PAGE = """<!doctype html><meta charset=utf-8><title>DAVIS346 calib</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px monospace;padding:10px}img{width:692px;max-width:100%;display:block}
button{font-size:16px;padding:8px 14px;margin:6px 6px 6px 0}pre{white-space:pre-wrap}</style>
<img id=v><div id=st></div><label><input type=checkbox id=au checked onchange="fetch('/auto?'+(this.checked?1:0),{method:'POST'})"> auto-capture</label> <button onclick=cap()>Capture (space)</button><button onclick=cal()>Calibrate</button>
<pre id=out></pre>
<script>
const v=document.getElementById('v');
function frame(){v.src='/frame.jpg?'+Date.now();}
v.onload=()=>setTimeout(frame,100);v.onerror=()=>setTimeout(frame,500);frame();
async function st(){try{const s=await(await fetch('/status')).json();
 document.getElementById('st').innerHTML='board: '+(s.found?'<b style=color:#0f0>DETECTED</b>':'<span style=color:#f55>not found</span>')+' | views: '+s.ncap+' | '+s.msg}catch(e){}setTimeout(st,300)}st();
async function cap(){const r=await(await fetch('/capture',{method:'POST'})).text();document.getElementById('out').textContent=r}
async function cal(){document.getElementById('out').textContent='calibrating...';const r=await(await fetch('/calibrate',{method:'POST'})).json();document.getElementById('out').textContent=JSON.stringify(r,null,1)}
addEventListener('keydown',e=>{if(e.code=='Space'){e.preventDefault();cap()}});
</script>"""


class H(BaseHTTPRequestHandler):
    def reply(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/frame.jpg"):
            with lock:
                j = S["jpg"]
            self.reply(j or b"", "image/jpeg")
        elif self.path.startswith("/status"):
            with lock:
                s = {"found": S["found"], "ncap": S["ncap"], "msg": S["msg"]}
            self.reply(json.dumps(s).encode(), "application/json")
        else:
            self.reply(PAGE.encode(), "text/html")

    def do_POST(self):
        if self.path.startswith("/auto"):
            with lock:
                S["auto"] = self.path.endswith("1")
            self.reply(b"ok", "text/plain")
        elif self.path.startswith("/capture"):
            m = capture()
            with lock:
                S["msg"] = m
            self.reply(m.encode(), "text/plain")
        else:
            self.reply(json.dumps(calibrate()).encode(), "application/json")

    def log_message(self, *a):
        pass


def main():
    rclpy.init()
    n = N()
    threading.Thread(target=ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever, daemon=True).start()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
