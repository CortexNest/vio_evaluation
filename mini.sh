#! /bin/bash
cd /ROS2_interfaces
source /opt/ros/humble/setup.bash 
rm -rf build
colcon build
source /ROS2_interfaces/install/setup.bash
ros2 topic list

ros2 topic pub --once /baton_mini/stereo3_ctrl system_ctrl/AlgoCtrl "{header: {stamp: {sec: $(date +%s), nanosec: 0}, frame_id: ''}, algo_enable: true, algo_reboot: false, algo_reset: false}"

# 查看算法成功
ros2 topic echo /baton_mini/algo_status