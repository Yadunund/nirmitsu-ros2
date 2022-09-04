# nirmitsu-ros2: Robotics for everyone
![](https://github.com/Yadunund/nirmitsu-ros2/workflows/build/badge.svg)
![](https://github.com/Yadunund/nirmitsu-ros2/workflows/style/badge.svg)

![nirmitsu_overview](https://user-images.githubusercontent.com/13482049/188318336-18bad790-d12c-4841-89fa-4f20a38dcebb.png)


## nirmitsu_ros2_flowgraph
A programming application to learn robotics concepts in an intuitive way.

### Requirements
* [ROS 2 foxy, galactic, humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Setup
```
mkdir ws_nirmitsu/src -p
cd ws_nirmitsu/src
git clone https://github.com/yadunund/nirmitsu-ros2
cd ~/ws_nirmitsu
source /opt/ros/ROS_DISTRO/setup.bash # where ROS_DISTRO is foxy glactic or humble
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run
```
cd ~/ws_nirmitsu
source install/setup.bash
ros2 run nirmitsu_ros2_flowgraph nirmitsu
```

## Tutorials
Follow instructions [here](docs/workshop.md) to learn how robots move.
