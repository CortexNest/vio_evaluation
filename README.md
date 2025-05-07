# 项目简介

本项目用于评估vio设备的位姿精度；主要方法是通过控制睿尔曼Gen72机械臂获取末端位姿作为真值，并将vio设备固定在末端获取测量值，最后使用evo对vio设备进行精度评估；

## 开始

```bash
#python 环境3.9及以上, 通过无线连接到Gen72机械臂
cd ~
git clone https://github.com/CortexNest/vio_evaluation.git
git clone https://github.com/Hessian-matrix/ROS2_interfaces
pip install Robotic_Arm==1.0.1 rosbags evo
cd vio_evaluation

# 将设备重置到一个固定初始状态
python reset_joint7.py

# 启动黑森矩阵mini ros2环境

```bash
cd vio_evaluation

# 启动黑森矩阵
docker run -itd --name ros2 -v ./benchmark:/data -v /$HOME/ROS2_interfaces:/ROS2_interfaces -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -v /etc/localtime:/etc/localtime:ro -v /etc/timezone:/etc/timezone:ro  --privileged --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --net=host -v /dev:/dev osrf/ros:humble-desktop-full

docker cp mini.sh ros2:/ROS2_interfaces
docker exec -it ros2 bash -c  "cd /ROS2_interfaces && chmod +x mini.sh && ./mini.sh"
```

# 获取真值（TUM数据集格式存储）和测量值（realsenset265.bag && benchmark/mini)

```bash
python collect_data.py
```

# 真值可视化

```bash
cd benchmark
mkdir -p testX
sudo chown $USER:$USER -R mini
mv mini testX/
mv ../gt.txt testX/
mv ../realsenset265.bag testX/
evo_tra.sh testX
```


