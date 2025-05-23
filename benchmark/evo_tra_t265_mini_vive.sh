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
mv baton_mini_stereo3_odometry.tum ./$dict/mini.txt

# 从T265相机的bag文件中提取轨迹数据并保存为TUM格式
evo_traj bag ./$dict/realsenset265.bag  /camera/odom/sample  --save_as_tum
mv camera_odom_sample.tum ./$dict/t265.txt

python agilex_pose_to_tum.py --input_dir ./$dict/pose/pika --output_file ./$dict/vive.txt
# 可视化比较mini和T265的轨迹，以ground truth为参考
evo_traj tum ./$dict/t265.txt ./$dict/mini.txt ./$dict/vive.txt -va --ref=./$dict/gt.txt -p &

#计算mini轨迹的APE（绝对位姿误差）并绘图
# evo_ape tum ./$dict/gt.txt ./$dict/mini.txt -va -t --change_unit mm &

# #计算mini轨迹的RPE（相对位姿误差）并绘图
# evo_rpe tum ./$dict/gt.txt ./$dict/mini.txt -va --plot --change_unit mm &

# # 计算T265轨迹的APE（绝对位姿误差）并绘图
# evo_ape tum ./$dict/gt.txt ./$dict/t265.txt -va --plot --change_unit mm &

# # 计算T265轨迹的RPE（相对位姿误差）并绘图
# evo_rpe tum ./$dict/gt.txt ./$dict/t265.txt -va --plot --change_unit mm &

# # 计算vive轨迹的APE（绝对位姿误差）并绘图
# evo_ape tum ./$dict/gt.txt ./$dict/vive.txt -va --plot --change_unit mm &

# # 计算vive轨迹的RPE（相对位姿误差）并绘图
# evo_rpe tum ./$dict/gt.txt ./$dict/vive.txt -va --plot --change_unit mm &