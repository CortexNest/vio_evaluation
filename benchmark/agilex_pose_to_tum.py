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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert pose jsons to TUM format.")
    parser.add_argument('--input_dir', type=str, required=True, help='Input directory containing json files')
    parser.add_argument('--output_file', type=str, required=False, help='Output file path (default: input_dir/out.txt)')
    args = parser.parse_args()

    input_dir = args.input_dir
    output_file = args.output_file if args.output_file else os.path.join(input_dir, 'out.txt')

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