import os
import json
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import json
import math
import argparse

def euler_to_quaternion(roll, pitch, yaw):
    # 欧拉角转四元数
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

def convert_vive_to_relative_in_A(vive_file='vive.txt', output_file='vive.txt'):
    """
    直接将vive.txt（B系TUM格式）转换为A系下、以自身初始位置为原点的相对位姿
    """
    # 绕Z轴旋转-180°的四元数
    rot_z_pi = R.from_euler('z', -np.pi)

    with open(vive_file, 'r') as f:
        lines = f.readlines()

    # 获取初始位姿（B系下）
    initial_line = lines[0].strip().split()
    initial_x = float(initial_line[1])
    initial_y = float(initial_line[2])
    initial_z = float(initial_line[3])
    initial_qx = float(initial_line[4])
    initial_qy = float(initial_line[5])
    initial_qz = float(initial_line[6])
    initial_qw = float(initial_line[7])
    initial_rotation = R.from_quat([initial_qx, initial_qy, initial_qz, initial_qw])
    # 先变换到A系
    initial_rotation_A = rot_z_pi * initial_rotation
    initial_rotation_A_matrix = initial_rotation_A.as_matrix()

    with open(output_file, 'w') as f:
        for line in lines:
            data = line.strip().split()
            if len(data) != 8:
                continue
            timestamp = float(data[0])
            x = float(data[1])
            y = float(data[2])
            z = float(data[3])
            qx = float(data[4])
            qy = float(data[5])
            qz = float(data[6])
            qw = float(data[7])

            # 先将B系下的姿态变换到A系
            rot = R.from_quat([qx, qy, qz, qw])
            rot_A = rot_z_pi * rot
            rot_A_matrix = rot_A.as_matrix()

            # 位置：先做相对初始位置（B系下），再旋转到A系
            p_rel_B = np.array([x - initial_x, y - initial_y, z - initial_z])
            p_rel_A = rot_z_pi.apply(p_rel_B)

            # 姿态：相对初始姿态（A系下）
            relative_rot_matrix = rot_A_matrix @ initial_rotation_A_matrix.T
            relative_rot = R.from_matrix(relative_rot_matrix)
            relative_quat = relative_rot.as_quat()

            # 写入TUM格式
            f.write(f"{timestamp:.6f} {p_rel_A[0]:.6f} {p_rel_A[1]:.6f} {p_rel_A[2]:.6f} "
                    f"{relative_quat[0]:.6f} {relative_quat[1]:.6f} {relative_quat[2]:.6f} {relative_quat[3]:.6f}\n")

    print(f"转换完成！数据已保存到 {output_file}")
    print(f"初始位置(B系): ({initial_x:.3f}, {initial_y:.3f}, {initial_z:.3f})")
    print(f"初始四元数(B系): ({initial_qx:.3f}, {initial_qy:.3f}, {initial_qz:.3f}, {initial_qw:.3f})")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert pose jsons to TUM format.")
    parser.add_argument('--input_dir', type=str, required=True, help='Input directory containing json files')
    parser.add_argument('--output_file', type=str, required=False, help='Output file path (default: input_dir/out.txt)')
    args = parser.parse_args()

    input_dir = args.input_dir
    output_file = args.output_file if args.output_file else os.path.join(input_dir, 'vive.txt')

    files = [f for f in os.listdir(input_dir) if f.endswith('.json')]
    files.sort()  # 按时间戳排序

    with open(output_file, 'w') as out:
        for fname in files:
            timestamp = fname.replace('.json', '')
            with open(os.path.join(input_dir, fname), 'r') as f:
                data = json.load(f)
                x, y, z = data['x'], data['y'], data['z']
                roll, pitch, yaw = data['roll'], data['pitch'], data['yaw']
                qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
                out.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
    convert_vive_to_relative_in_A(output_file, output_file)