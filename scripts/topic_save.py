#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64  # 示例类型，根据实际情况修改
import os

class MultiTopicCollector:
    def __init__(self):
        # 准备存储多个topic数据的变量
        # 如果需要存储大量历史数据，可改成 list 或 deque
        self.topic1_data = None
        self.topic2_data = None
        # ... 也可以添加更多

        # 初始化订阅者(根据实际topic名称与类型进行修改)
        self.sub1 = rospy.Subscriber("/topic1", Float64, self.callback_topic1)
        self.sub2 = rospy.Subscriber("/topic2", Float64, self.callback_topic2)
        # ... 如有更多topic, 继续添加

    def callback_topic1(self, msg):
        # 从msg中取出数据，存到内存变量
        self.topic1_data = msg.data

    def callback_topic2(self, msg):
        self.topic2_data = msg.data

    # 如果有更多回调函数，就根据实际需求添加

def main():
    rospy.init_node("multi_topic_logger", anonymous=True)

    # 创建数据采集/存储对象
    collector = MultiTopicCollector()

    # 文件路径(示例: 当前工作目录下 data.txt)
    # 也可用绝对路径, 或者结合datetime自动命名
    file_path = os.path.join(os.getcwd(), "data.txt")

    # 打开文件（这里演示用a模式: 追加写）
    # 如果你想每次都覆盖, 可以用 'w'
    f = open(file_path, "a")

    # 设定写文件的循环频率, 例如 10Hz
    rate = rospy.Rate(10)

    # 在主循环中不断写数据
    while not rospy.is_shutdown():
        # 取出最新的topic数据(若回调还没来过, 可能还是None)
        data1 = collector.topic1_data
        data2 = collector.topic2_data

        # 如果不想写None，可做判断后再写
        # 这里简单演示一下:
        if data1 is not None and data2 is not None:
            # 自定义写入格式(例如"data1 data2\n")
            line = "{:.3f} {:.3f}\n".format(data1, data2)
            f.write(line)

        # 也可以选择把所有数据放在一个list里, 在这里整批写
        # 或者写csv格式: line = "{},{}\n".format(data1, data2)

        # 刷新缓冲(视需求,也可以不开,提高性能)
        f.flush()

        # 控制循环频率
        rate.sleep()

    # 程序退出时, 关闭文件
    f.close()

if __name__ == '__main__':
    main()