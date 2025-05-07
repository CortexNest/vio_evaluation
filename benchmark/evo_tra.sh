#! /bin/bash

# 设置测试数据集目录名称
dict=$1

# 将mini数据集转换为ROS bag格式
rosbags-convert --src ./$dict/mini --dst mini.bag

# 运行时间戳对齐脚本
python3 mini_timestamp_fit.py

# 从ROS bag中提取轨迹数据并保存为TUM格式
evo_traj bag mini_output.bag  /baton_mini/stereo3/odometry --save_as_tum

# 清理临时文件
rm mini.bag
rm mini_output.bag
mv baton_mini_stereo3_odometry.tum mini.txt

# 从T265相机的bag文件中提取轨迹数据并保存为TUM格式
evo_traj bag ./$dict/realsenset265.bag  /camera/odom/sample  --save_as_tum
mv camera_odom_sample.tum t265.txt

# 可视化比较mini和T265的轨迹，以ground truth为参考
evo_traj tum t265.txt mini.txt -va --ref=./$dict/gt.txt -p  --change_unit mm &

# 计算mini轨迹的APE（绝对位姿误差）并绘图
evo_ape tum ./$dict/gt.txt mini.txt -va --plot --change_unit mm &

# 计算mini轨迹的RPE（相对位姿误差）并绘图
# evo_rpe tum ./$dict/gt.txt mini.txt -va --plot &

# # 计算T265轨迹的APE（绝对位姿误差）并绘图
evo_ape tum ./$dict/gt.txt t265.txt -va --plot --change_unit mm &

# # 计算T265轨迹的RPE（相对位姿误差）并绘图
# evo_rpe tum ./$dict/gt.txt t265.txt -va --plot &