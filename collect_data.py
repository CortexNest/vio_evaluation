from Robotic_Arm.rm_robot_interface import *
import time
import math
import subprocess
import atexit

class RobotArmController:
    def __init__(self, ip, port=8080, level=3, mode=2):
        """
        初始化并连接机器人
        
        Args:
            ip (str): 机器人的IP地址
            port (int): 端口号，默认8080
            level (int): 连接等级，默认3
            mode (int): 线程模式，默认2 (0:单线程, 1:双线程, 2:三线程)
        """
        self.thread_mode = rm_thread_mode_e(mode)
        self.robot = RoboticArm(self.thread_mode)
        self.handle = self.robot.rm_create_robot_arm(ip, port, level)

        if self.handle.id == -1:
            print("\n连接机器人失败\n")
            exit(1)
        else:
            print(f"\n成功连接到机器人: {self.handle.id}\n")
            
    def move_joint(self, joint_angles, speed=20, radius=0, block=1):
        """
        执行关节运动
        
        Args:
            joint_angles (list): 目标关节角度列表
            speed (int): 关节速度，默认20度/秒
            radius (float): 交融半径，默认0
            block (int): 是否阻塞，1为阻塞，0为非阻塞
        
        Returns:
            int: 返回码，0表示成功
        """
        # return self.robot.rm_movej_canfd(joint_angles, False)
        return self.robot.rm_movej(joint_angles, speed, radius, 0, block)

    def get_current_pose(self):
        """
        获取机器人当前位姿
        
        Returns:
            tuple: (ret, state) 
                - ret: 返回码 (0表示成功)
                - state: 包含位姿信息的字典
        """
        ret, state = self.robot.rm_get_current_arm_state()
        
        if ret == 0:
            current_pose = state["pose"]
            arm_model = rm_robot_arm_model_e.RM_MODEL_GEN_72_E
            force_type = rm_force_type_e.RM_MODEL_RM_B_E
            algo_handle = Algo(arm_model, force_type)
            quaternion = algo_handle.rm_algo_euler2quaternion(current_pose[3:])
            state["quaternion"] = quaternion
            
        return ret, state

    def move_linear(self, target_pose, speed=0.1, block=1):
        """
        执行直线运动
        
        Args:
            target_pose (list): 目标位姿 [x, y, z, rx, ry, rz]
            speed (float): 末端速度，默认0.1m/s
            block (int): 是否阻塞，1为阻塞，0为非阻塞
        
        Returns:
            int: 返回码，0表示成功
        """
        # 将speed转换为整数类型的速度百分比（1-100）
        speed_percentage = int(speed * 100)  # 将m/s转换为百分比
        speed_percentage = max(1, min(100, speed_percentage))  # 限制在1-100范围内
        
        return self.robot.rm_movel(target_pose, speed_percentage, 0, 0, block)

    def is_in_motion(self):
        """
        检查机器人是否在运动中
        
        Returns:
            bool: True表示在运动，False表示停止
        """
        ret, state = self.robot.rm_get_current_arm_state()
        if ret == 0:
            # 这里需要根据实际SDK检查运动状态
            # 假设通过比较当前关节角度和目标角度来判断
            return True
        return False
    
    def disconnect(self):
        """
        断开与机器人的连接
        """
        handle = self.robot.rm_delete_robot_arm()
        if handle == 0:
            print("\n成功断开机器人连接\n")
        else:
            print("\n断开机器人连接失败\n")

def print_pose_info(pose, quaternion, sample_index=None, file=None):
    """
    打印位姿信息并保存到文件(TUM格式)
    
    Args:
        pose (list): 位姿数据 [x, y, z, rx, ry, rz]
        quaternion (list): 四元数数据 [qw, qx, qy, qz]
        sample_index (int, optional): 采样序号
        file: 文件对象
    """
    if sample_index is not None:
        print(f"\n=== 采样点 {sample_index} ===")
        
    # 打印到控制台
    print("位置信息:")
    print(f"X: {pose[0]:.6f} m")
    print(f"Y: {pose[1]:.6f} m")
    print(f"Z: {pose[2]:.6f} m")

    print("\n姿态信息(欧拉角):")
    print(f"Rx: {pose[3]:.6f} rad ({pose[3]*180/3.14159:.2f}°)")
    print(f"Ry: {pose[4]:.6f} rad ({pose[4]*180/3.14159:.2f}°)")
    print(f"Rz: {pose[5]:.6f} rad ({pose[5]*180/3.14159:.2f}°)")

    print("\n姿态信息(四元数):")
    print(f"Qw: {quaternion[0]:.6f}")
    print(f"Qx: {quaternion[1]:.6f}")
    print(f"Qy: {quaternion[2]:.6f}")
    print(f"Qz: {quaternion[3]:.6f}")

    # 保存到文件 (TUM格式)
    if file:
        # TUM格式: timestamp tx ty tz qx qy qz qw
        timestamp = time.time()
        # 注意四元数顺序：TUM格式为qx qy qz qw，而我们的数据为qw qx qy qz
        tum_line = f"{timestamp:.6f} {pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} {quaternion[1]:.6f} {quaternion[2]:.6f} {quaternion[3]:.6f} {quaternion[0]:.6f}\n"
        file.write(tum_line)

def draw_shapes(robot, initial_pose, square_size=0.1, circle_radius=0.05, move_speed=1.0, 
                rotation_angle=0, sampling_rate=0.1, file=None):
    """
    绘制正方形和圆形，支持绕Z轴旋转
    
    Args:
        robot: RobotArmController实例
        initial_pose: 初始位姿
        square_size: 正方形边长（米）
        circle_radius: 圆形半径（米）
        move_speed: 运动速度（0-1）
        rotation_angle: 绕Z轴旋转角度（弧度）
        sampling_rate: 采样频率（秒）
        file: 文件对象，用于保存轨迹数据
    """
    try:
        sample_index = 0
        
        # 第一步：向右移动20cm
        print("\n第一步：向右移动20cm...")
        right_pose = initial_pose.copy()
        right_pose[1] += 0.2  # Y轴正方向移动20cm
        
        ret = robot.move_linear(right_pose, move_speed, 1)
        if ret != 0:
            print(f"右移失败，错误码: {ret}")
            return False
        
        # 开始画正方形
        print("\n开始画正方形...")
        print(f"边长: {square_size*100:.0f}cm")
        print(f"旋转角度: {rotation_angle*180/math.pi:.1f}°")
        print(f"速度: {move_speed*100:.0f}%")
        
        current_pose = right_pose.copy()
        
        # 定义四个目标点
        points = []
        for i in range(4):
            angle = i * math.pi/2
            if rotation_angle == 0:
                # 第一次：在YZ平面
                x = current_pose[0]
                y = current_pose[1] + square_size * math.cos(angle)
                z = current_pose[2] + square_size * math.sin(angle)
            else:
                # 第二次和第三次：在旋转后的平面
                x = current_pose[0] + square_size * math.cos(angle + rotation_angle)
                y = current_pose[1] + square_size * math.sin(angle + rotation_angle)
                z = current_pose[2]
            points.append((x, y, z))
        
        # 执行四段直线运动画正方形
        for i, (target_x, target_y, target_z) in enumerate(points):
            print(f"\n移动到点{i+1}...")
            target_pose = current_pose.copy()
            target_pose[0] = target_x
            target_pose[1] = target_y
            target_pose[2] = target_z
            
            # 使用非阻塞模式开始运动
            ret = robot.move_linear(target_pose, move_speed, 0)
            if ret != 0:
                print(f"移动失败，错误码: {ret}")
                return False
            
            # 在运动过程中采样
            while True:
                ret, state = robot.get_current_pose()
                if ret == 0:
                    current_state = state
                    print_pose_info(state["pose"], state["quaternion"], sample_index, file)
                    sample_index += 1
                    
                    # 检查是否到达目标位置
                    current_x = state["pose"][0]
                    current_y = state["pose"][1]
                    current_z = state["pose"][2]
                    if (abs(current_x - target_x) < 0.001 and  # 1mm的误差
                        abs(current_y - target_y) < 0.001 and
                        abs(current_z - target_z) < 0.001):
                        break
                
                time.sleep(sampling_rate)  # 等待下一次采样
            
            # 更新当前位姿
            current_pose = current_state["pose"]
        
        print("\n正方形轨迹运动完成")
        
        # 返回初始位置
        print("\n返回初始位置...")
        ret = robot.move_linear(initial_pose, move_speed, 1)
        if ret != 0:
            print(f"返回初始位置失败，错误码: {ret}")
            return False
        
        # 开始画圆
        print("\n开始画圆...")
        print(f"半径: {circle_radius*100:.0f}cm")
        print(f"旋转角度: {rotation_angle*180/math.pi:.1f}°")
        
        # 使用36个点近似一个圆（每10度一个点）
        num_points = 36
        current_pose = initial_pose.copy()
        
        for i in range(num_points + 1):  # +1是为了回到起点
            angle = i * (2 * math.pi / num_points)  # 当前角度（弧度）
            
            # 计算圆上的点
            target_pose = current_pose.copy()
            if rotation_angle == 0:
                # 第一次：在YZ平面
                target_pose[1] = initial_pose[1] + circle_radius * math.cos(angle)
                target_pose[2] = initial_pose[2] + circle_radius * math.sin(angle)
            else:
                # 第二次和第三次：在旋转后的平面
                target_pose[0] = initial_pose[0] + circle_radius * math.cos(angle + rotation_angle)
                target_pose[1] = initial_pose[1] + circle_radius * math.sin(angle + rotation_angle)
                target_pose[2] = initial_pose[2]
            
            # 使用非阻塞模式开始运动
            ret = robot.move_linear(target_pose, move_speed, 0)
            if ret != 0:
                print(f"画圆移动失败，错误码: {ret}")
                return False
            
            # 在运动过程中采样
            while True:
                ret, state = robot.get_current_pose()
                if ret == 0:
                    current_state = state
                    print_pose_info(state["pose"], state["quaternion"], sample_index, file)
                    sample_index += 1
                    
                    # 检查是否到达目标位置
                    current_x = state["pose"][0]
                    current_y = state["pose"][1]
                    current_z = state["pose"][2]
                    if (abs(current_x - target_pose[0]) < 0.001 and
                        abs(current_y - target_pose[1]) < 0.001 and
                        abs(current_z - target_pose[2]) < 0.001):
                        break
                
                time.sleep(sampling_rate)  # 等待下一次采样
            
            # 更新当前位姿
            current_pose = current_state["pose"]
        
        print("\n圆形轨迹运动完成")
        
        # 返回初始位置
        print("\n返回初始位置...")
        ret = robot.move_linear(initial_pose, move_speed, 1)
        if ret != 0:
            print(f"返回初始位置失败，错误码: {ret}")
            return False
        
        return True
        
    except Exception as e:
        print(f"发生异常: {str(e)}")
        return False

def main():
    try:
        # Start RealSense camera node
        print("\nStarting RealSense camera node...")
        realsense_process = subprocess.Popen(['roslaunch', 'realsense2_camera', 'rs_t265.launch'])
        
        # Wait for camera initialization
        time.sleep(5)
        
        # Start recording pose data
        print("Starting pose recording...")
        rosbag_process = subprocess.Popen(['rosbag', 'record', '-O', 'realsenset265.bag', '/camera/odom/sample'])

        # 启动ros2 bag录制
        minibag_process = subprocess.Popen(['docker', 'exec', '-it',  'ros2', 'bash', '-c', 'cd /ROS2_interfaces && source /opt/ros/humble/setup.bash && ros2 bag record /baton_mini/stereo3/odometry -o /data/mini'])
        
        # 在程序结束时关闭录制
        def cleanup():
            if minibag_process.poll() is None:  # 检查进程是否还在运行
                docker_exec_process = subprocess.Popen(['docker', 'exec', 'ros2', 'pkill', '-f', 'ros2 bag record'])
                docker_exec_process.wait()
                minibag_process.terminate()
                minibag_process.wait()
        
        # 注册清理函数
        atexit.register(cleanup)

        # Create output file
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"trajectory_{timestamp}.txt"
        with open('gt.txt', 'w') as f:
            robot = RobotArmController("192.168.33.80")
            
            # Define initial joint angles (degrees)
            initial_joints = [
                0,      # Joint 1: 0 degrees
                -90,    # Joint 2: -90 degrees
                0,      # Joint 3: 0 degrees
                -150,   # Joint 4: -150 degrees
                0,      # Joint 5: 0 degrees
                60,     # Joint 6: 60 degrees
                0       # Joint 7: 0 degrees
            ]
            
            # Define motion parameters
            square_size = 0.1    # Square side length 10cm
            circle_radius = 0.05   # Circle radius 5cm
            move_speed = 1.0     # 100% speed
            sampling_rate = 0.1  # 10Hz sampling rate
            
            # First motion: default angle
            print("\nFirst motion: default angle")
            # Move to initial joint angles
            ret = robot.move_joint(initial_joints)  # Speed set to 10 degrees/second
            if ret != 0:
                print(f"Failed to move to initial joint angles, error code: {ret}")
                return
            
            # Get initial pose
            print("Getting initial pose...")
            ret, state = robot.get_current_pose()
            if ret == 0:
                initial_pose = state["pose"]
                print("Initial pose:")
                print_pose_info(initial_pose, state["quaternion"], None, f)
                
                if not draw_shapes(robot, initial_pose, square_size, circle_radius, move_speed, 
                                0, sampling_rate, f):
                    return
                
                # Second motion: rotate 45 degrees
                print("\nSecond motion: rotate 45 degrees")
                # Move to initial joint angles
                ret = robot.move_joint(initial_joints)  # Speed set to 10 degrees/second
                if ret != 0:
                    print(f"Failed to move to initial joint angles, error code: {ret}")
                    return
                
                rotation_angle = math.pi/4  # 45 degrees
                if not draw_shapes(robot, initial_pose, square_size, circle_radius, move_speed, 
                                rotation_angle, sampling_rate, f):
                    return
                
                # Third motion: rotate 90 degrees
                print("\nThird motion: rotate 90 degrees")
                # Move to initial joint angles
                ret = robot.move_joint(initial_joints)  # Speed set to 10 degrees/second
                if ret != 0:
                    print(f"Failed to move to initial joint angles, error code: {ret}")
                    return
                
                rotation_angle = math.pi/2  # 90 degrees
                if not draw_shapes(robot, initial_pose, square_size, circle_radius, move_speed, 
                                rotation_angle, sampling_rate, f):
                    return
                
                print("\nAll motion sequences completed")
                print(f"Trajectory data saved to file: {filename}")
                
            else:
                print(f"Failed to get initial pose, error code: {ret}")
            
    except Exception as e:
        print(f"An exception occurred: {str(e)}")
        
    finally:
        # Stop recording pose data
        print("\nStopping pose recording...")
        rosbag_process.terminate()
        rosbag_process.wait()
        
        # Stop RealSense camera node
        print("Stopping RealSense camera node...")
        realsense_process.terminate()
        realsense_process.wait()
        
        # Disconnect robot
        robot.disconnect()

if __name__ == "__main__":
    main()