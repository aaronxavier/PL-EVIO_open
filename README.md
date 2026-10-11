<div align="center">

# PL-EVIO：Robust Monocular Event-based Visual Inertial Odometry with Point and Line Features 

**Weipeng Guan**<sup>†</sup>, **Peiyu Chen**<sup>†</sup>, **Yuhan Xie**, **Peng Lu**<sup>*</sup>

**Adaptive Robotic Controls Lab (ArcLab)**, **The University of Hong Kong**.

</div>


[comment]: <> (  <h2 align="center">PAPER</h2>)
  <h3 align="center">
  <a href="https://ieeexplore.ieee.org/abstract/document/10287884">Paper</a> 
  | <a href="https://kwanwaipang.github.io/PL-EVIO/">Website</a> 
  | <a href="https://www.bilibili.com/video/BV12t4y1L7eK/?spm_id_from=333.1387">Demo</a> 
  </h3>

<div align="center">
  <img src="https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/raw/main/PL-EVIO/PLEVIO_flip_3.gif" width="90%" />
</div>


## Abstract
<div align="justify">
Robust state estimation in challenge situations is still an unsolved problem, especially achieving onboard pose feedback control for aggressive motion. 
In this paper, we propose robust and real-time event-based visual-inertial odometry (VIO) that incorporates event, image, and inertial measurements. 
Our approach utilizes line-based event features to provide additional structure and constraint information in human-made scenes, while point-based event and image features complement each other through well-designed feature management. 
To achieve reliable state estimation, we tightly couple the point-based and line-based visual residuals from the event camera, the point-based visual residual from the standard camera, and the residual from IMU pre-integration using a keyframe-based graph optimization framework. 
Experiments in the public benchmark datasets show that our method can achieve superior performance compared with the state-of-the-art image-based or event-based VIO. 
Furthermore, we demonstrate the effectiveness of our pipeline through onboard closed-loop quadrotor aggressive flight and large-scale outdoor experiments. Videos of the evaluations can be found on our website.
</div>


## 1. Prerequisites
### 1.1 ROS2 environment
- Ubuntu 22.04 + ROS2 Humble (recommended), or another ROS2 distro with equivalent APIs.
- CMake toolchain and colcon.

### 1.2 Core third-party dependencies
- Ceres Solver (project was developed around 1.14.0; see [Ceres Installation](http://ceres-solver.org/installation.html)).
- OpenCV and Eigen.
- The repository contains dependency wrappers under `dependences/` used by the original project.

### 1.3 ROS2 packages included in this port
- Core pipeline: `camera_model`, `feature_tracker`, `evio_estimator`, `pose_graph`
- New ROS2 support packages: `dvs_msgs`, `event_packet_decoder`

## 2. Build (ROS2)

```sh
mkdir -p ~/evio2_ws/src
cd ~/evio2_ws/src
git clone git@github.com:ERGlab/PLEVIO.git --recursive

cd ~/evio2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Optional convenience alias:

```sh
alias plevio2build='cd ~/evio2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

## 3. Run on ROS2
We have released rosbag datasets for evaluating PL-EVIO. Dataset introductions are on this
[page](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM#Dataset-for-monocular-evio), and
comparison-result bags are available [here](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/Results_for_comparison.md).

### 3.1 Example: DAVIS346 online pipeline (ROS2)
Use the ROS2 launch file added in this port:

```sh
source ~/evio2_ws/install/setup.bash
ros2 launch evio_estimator plevio_davis346.launch.py
```

Default config path in the launch file points to:

```sh
config/ros2_davis346/davis346.yaml
```

If your workspace path differs, pass launch arguments:

```sh
ros2 launch evio_estimator plevio_davis346.launch.py \
  config_file:=/path/to/PLEVIO/config/ros2_davis346/davis346.yaml \
  vins_folder:=/path/to/PLEVIO/
```

### 3.2 Event packet decode standalone (ROS2)

```sh
source ~/evio2_ws/install/setup.bash
ros2 launch event_packet_decoder event_packet_decoder.launch.py
```

### 3.3 Legacy ROS1 launch files
Some ROS1 launch files are still kept in the repository for reference. They require a ROS1 setup and are not part of the ROS2 runtime flow above.

### 3.4 Run on Your Event Camera
* We recommend to follow the instruction on ([DVS-IMU Calibration and Synchronization](https://arclab-hku.github.io/ecmd/calibration/)) to calibrate your sensors.
* Refer to our [ESVIO](https://github.com/arclab-hku/ESVIO)


<!-- ******************************************************* -->
## Acknowledgement
This work was supported by General Research Fund under Grant 17204222, and in part by the Seed Fund for Collaborative Research and General Funding Scheme-HKU-TCL Joint Research Center for Artificial
Intelligence.
We use ([VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)) as our backbone code. Thanks Dr. Qin Tong, Prof. Shen, etc. very much.

If you find this work is helpful in your research, a simple star or citation of our works should be the best affirmation for us. :blush:

~~~
@article{GWPHKU:PL-EVIO,
  title={PL-EVIO: Robust Monocular Event-based Visual Inertial Odometry with Point and Line Features},
  author={Guan, Weipeng and Chen, Peiyu and Xie, Yuhan and Lu, Peng},
  journal={IEEE Transactions on Automation Science and Engineering},
  volume={21},
  number={4},
  pages={6277--6293},
  year={2023},
  publisher={IEEE}
}
~~~

## License
The source code is released under GPLv3 license. 
We are still working on improving the code reliability. 
If you are interested in our project for commercial purposes, please contact [Dr. Peng LU](https://arclab.hku.hk/People.html) for further communication.
