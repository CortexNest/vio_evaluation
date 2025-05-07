#!/usr/bin/env python3
import rosbag
from std_msgs.msg import Header
from Crypto.Cipher import AES

def modify_header_stamp(input_bag, output_bag, topic_to_modify):
    """
    将指定 topic 的 header.stamp 修改为消息的记录时间
    
    Args:
        input_bag (str): 输入 bag 文件路径
        output_bag (str): 输出 bag 文件路径
        topic_to_modify (str): 需要修改的 topic 名称
    """
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic == topic_to_modify:
                # 将 header.stamp 修改为消息的记录时间 t
                msg.header.stamp = t
            outbag.write(topic, msg, t)

if __name__ == "__main__":
    input_bag = 'mini.bag'  # 输入 bag 文件
    output_bag = 'mini_output.bag'  # 输出 bag 文件
    topic_to_modify = '/baton_mini/stereo3/odometry'  # 需要修改的 topic
    
    modify_header_stamp(input_bag, output_bag, topic_to_modify)
    print(f"修改完成，新 bag 文件已保存为: {output_bag}")