#!/usr/bin/env python3

import rospy
import csv
import os
from geometry_msgs.msg import Vector3Stamped
from datetime import datetime

# 全局变量存储数据
data_list = []

def callback(msg):
    timestamp = msg.header.stamp.to_sec()
    x, y, z = msg.vector.x, msg.vector.y, msg.vector.z
    data_list.append([timestamp, x, y, z])
    rospy.loginfo("Received: t=%.3f, x=%.3f, y=%.3f, z=%.3f", timestamp, x, y, z)

def save_data():
    if not data_list:
        rospy.logwarn("No data received. Nothing to save.")
        return

    filename = f"adc_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    filepath = os.path.join(os.getcwd(), filename)

    try:
        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'x', 'y', 'z'])
            writer.writerows(data_list)
        rospy.loginfo(f"✔ Saved {len(data_list)} records to: {filepath}")
    except Exception as e:
        rospy.logerr(f"❌ Failed to save CSV: {e}")

def main():
    rospy.init_node('adc_logger', anonymous=True)
    rospy.Subscriber('/triple_adc_value', Vector3Stamped, callback)

    # 注册 Ctrl+C 退出时的回调函数
    rospy.on_shutdown(save_data)

    rospy.loginfo("Recording data from /triple_adc_value. Press Ctrl+C to stop and save.")

    rospy.spin()  # 阻塞直到 Ctrl+C

if __name__ == '__main__':
    main()
