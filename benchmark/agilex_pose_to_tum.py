import os
import json
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R

def jsons_to_relative_tum(input_dir, output_file):
    """
    读取input_dir下的json文件，直接输出A系下、以自身初始位置为原点的TUM格式轨迹
    """
    # 绕Z轴旋转-180°的四元数
    rot_z_pi = R.from_euler('z', -np.pi)

    files = [f for f in os.listdir(input_dir) if f.endswith('.json')]
    files.sort()  # 按时间戳排序

    # 先读取所有数据
    poses = []
    for fname in files:
        timestamp = fname.replace('.json', '')
        with open(os.path.join(input_dir, fname), 'r') as f:
            data = json.load(f)
            x, y, z = data['x'], data['y'], data['z']
            roll, pitch, yaw = data['roll'], data['pitch'], data['yaw']
            # 注意：from_euler('ZYX', ...) 需传入 [yaw, pitch, roll]
            rot = R.from_euler('ZYX', [yaw, pitch, roll])
            quat = rot.as_quat()
            poses.append((timestamp, x, y, z, quat))

    # 获取初始位姿（B系下）
    initial_timestamp, initial_x, initial_y, initial_z, initial_quat = poses[0]
    initial_rotation = R.from_quat(initial_quat)
    # 先变换到A系
    initial_rotation_A = rot_z_pi * initial_rotation
    initial_rotation_A_matrix = initial_rotation_A.as_matrix()

    with open(output_file, 'w') as out:
        for timestamp, x, y, z, quat in poses:
            # 先将B系下的姿态变换到A系
            rot = R.from_quat(quat)
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
            out.write(f"{timestamp} {p_rel_A[0]:.6f} {p_rel_A[1]:.6f} {p_rel_A[2]:.6f} "
                      f"{relative_quat[0]:.6f} {relative_quat[1]:.6f} {relative_quat[2]:.6f} {relative_quat[3]:.6f}\n")

    print(f"转换完成！数据已保存到 {output_file}")
    print(f"初始位置(B系): ({initial_x:.3f}, {initial_y:.3f}, {initial_z:.3f})")
    print(f"初始四元数(B系): ({initial_quat[0]:.3f}, {initial_quat[1]:.3f}, {initial_quat[2]:.3f}, {initial_quat[3]:.3f})")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert pose jsons to relative TUM format in A frame.")
    parser.add_argument('--input_dir', type=str, required=True, help='Input directory containing json files')
    parser.add_argument('--output_file', type=str, required=False, help='Output file path (default: input_dir/vive.txt)')
    args = parser.parse_args()

    input_dir = args.input_dir
    output_file = args.output_file if args.output_file else os.path.join(input_dir, 'vive.txt')

    jsons_to_relative_tum(input_dir, output_file)