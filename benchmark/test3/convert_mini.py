import numpy as np
from scipy.spatial.transform import Rotation

def convert_to_gt_init(mini_file='mini.txt', output_file='mini.txt', roll_offset_deg=0):
    with open(mini_file, 'r') as f:
        lines = f.readlines()
    # mini.txt初始四元数
    q0 = list(map(float, lines[0].strip().split()[4:8]))
    rot0_inv = Rotation.from_quat(q0).inv()

    roll_offset_rad = np.deg2rad(roll_offset_deg)

    with open(output_file, 'w') as f:
        for line in lines:
            data = line.strip().split()
            if len(data) != 8:
                continue
            timestamp = data[0]
            x, y, z = data[1:4]
            q = list(map(float, data[4:8]))
            rot = Rotation.from_quat(q)
            rot_new = rot * rot0_inv
            # 转欧拉角
            euler = rot_new.as_euler('zyx')
            # 补偿roll
            euler[0] += roll_offset_rad
            # 转回四元数
            rot_new_offset = Rotation.from_euler('zyx', euler)
            q_new = rot_new_offset.as_quat()
            f.write(f"{timestamp} {x} {y} {z} {q_new[0]} {q_new[1]} {q_new[2]} {q_new[3]}\n")

if __name__ == "__main__":
    convert_to_gt_init()