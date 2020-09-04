# Simple Monocular Visual Odometry for ROS
![](./intro.gif)
- Mainly inspired by https://github.com/avisingh599/mono-vo
- Dataset: KITTI 00 sequence
- Extractor: FAST
- Tracker: LK Optical Flow
- Tested Environment: Ubuntu 18.04 LTS & ROS melodic.

# Instruction
- Clone this repository.
- Put this direcotory into your workspace such as `catkin_ws`.
- Build the workspace using `catkin_make`.
- Update ROS packages using `rospack profile`.
- Edit the launch file `launch/mono_vo_ros.launch`
  - Change `fn_kitti` variable to the kitti path in your system.
- Run `roslaunch mono_vo_ros mono_vo_ros.launch`.
