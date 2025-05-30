import numpy as np
from scipy.spatial.transform import Rotation

def convert_gt_to_relative(input_file='gt.txt', output_file='gt_relative.txt'):
    """
    将gt.txt中的绝对位姿转换为相对于初始位置的位姿
    
    Args:
        input_file (str): 输入文件名
        output_file (str): 输出文件名
    """
    # 读取文件
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    # 获取初始位姿
    initial_line = lines[0].strip().split()
    initial_timestamp = float(initial_line[0])
    initial_x = float(initial_line[1])
    initial_y = float(initial_line[2])
    initial_z = float(initial_line[3])
    initial_qx = float(initial_line[4])
    initial_qy = float(initial_line[5])
    initial_qz = float(initial_line[6])
    initial_qw = float(initial_line[7])
    
    # 创建初始姿态的旋转矩阵
    initial_rotation = Rotation.from_quat([initial_qx, initial_qy, initial_qz, initial_qw])
    initial_rotation_matrix = initial_rotation.as_matrix()
    
    # 写入转换后的数据
    with open(output_file, 'w') as f:
        for line in lines:
            data = line.strip().split()
            if len(data) != 8:  # 确保数据格式正确
                continue
                
            timestamp = float(data[0])
            x = float(data[1])
            y = float(data[2])
            z = float(data[3])
            qx = float(data[4])
            qy = float(data[5])
            qz = float(data[6])
            qw = float(data[7])
            
            # 计算相对位置
            relative_x = x - initial_x
            relative_y = y - initial_y
            relative_z = z - initial_z
            
            # 计算相对姿态
            current_rotation = Rotation.from_quat([qx, qy, qz, qw])
            current_rotation_matrix = current_rotation.as_matrix()
            
            # 计算相对旋转矩阵：R_relative = R_current * R_initial^T
            relative_rotation_matrix = current_rotation_matrix @ initial_rotation_matrix.T
            
            # 转换回四元数
            relative_rotation = Rotation.from_matrix(relative_rotation_matrix)
            relative_quat = relative_rotation.as_quat()  # 返回 [x, y, z, w] 格式
            
            # 写入TUM格式: timestamp tx ty tz qx qy qz qw
            f.write(f"{timestamp:.6f} {relative_x:.6f} {relative_y:.6f} {relative_z:.6f} "
                   f"{relative_quat[0]:.6f} {relative_quat[1]:.6f} {relative_quat[2]:.6f} {relative_quat[3]:.6f}\n")
    
    print(f"转换完成！数据已保存到 {output_file}")
    print(f"初始位置: ({initial_x:.3f}, {initial_y:.3f}, {initial_z:.3f})")
    print(f"初始四元数: ({initial_qx:.3f}, {initial_qy:.3f}, {initial_qz:.3f}, {initial_qw:.3f})")

if __name__ == "__main__":
    convert_gt_to_relative()