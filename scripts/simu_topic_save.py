#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import os
import numpy as np

# 示例消息类型：假设发布的是 Float64MultiArray，里面 data=[x,y,z,vx,vy,vz].
from test_controller.msg import QSLSState

class MultiDimDataCollector:
    def __init__(self):
        # 用于保存历史数据的列表
        # 每个元素示例: (time, [x, y, z], [vx, vy, vz]) —— 具体形状可按需改动
        self.observer = []
        self.feedback = []
        self.time = []

        # 订阅两个话题 (根据你实际的话题和消息类型来改)
        self.sub_qsls_state = rospy.Subscriber("/X250/myqsls_state",
                                          QSLSState,
                                          self.callback_qsls_state)
        self.sub_qsls_feedback = rospy.Subscriber("/X250/qsls_state",
                                            QSLSState,
                                            self.callback_qsls_feedback)

        # 临时存储最新值
        self.observer_data = {"pQ":[0,0,0], "vQ":[0,0,0], "pL":[0,0,0], "vL":[0,0,0], "q":[0,0,1], "w":[0,0,0], "quat":[0,0,0,1], "bL":[0,0,0], "bQ":[0,0,0]}
        self.feedback_data = {"pQ":[0,0,0], "vQ":[0,0,0], "pL":[0,0,0], "vL":[0,0,0], "q":[0,0,1], "w":[0,0,0], "quat":[0,0,0,1], "bL":[0,0,0], "bQ":[0,0,0]}

    def callback_qsls_state(self, msg: QSLSState):
        """
        假设 msg.data = [x, y, z, vx, vy, vz].
        你需要根据自己的实际消息结构来修改解析方式。
        """
        self.observer_data = {
            "pQ": [msg.pQ.x,msg.pQ.y,msg.pQ.z],
            "vQ": [msg.vQ.x,msg.vQ.y,msg.vQ.z],
            "pL": [msg.pL.x,msg.pL.y,msg.pL.z],
            "vL": [msg.vL.x,msg.vL.y,msg.vL.z],
            "q": [msg.q.x,msg.q.y,msg.q.z],
            "w": [msg.w.x,msg.w.y,msg.w.z],
            "quat": [msg.quat.w,msg.quat.x,msg.quat.y,msg.quat.z],
            "bL": [msg.bL.x,msg.bL.y,msg.bL.z],
            "bQ": [msg.bQ.x,msg.bQ.y,msg.bQ.z],
        }

    def callback_qsls_feedback(self, msg: QSLSState):

        self.feedback_data = {
            "pQ": [msg.pQ.x,msg.pQ.y,msg.pQ.z],
            "vQ": [msg.vQ.x,msg.vQ.y,msg.vQ.z],
            "pL": [msg.pL.x,msg.pL.y,msg.pL.z],
            "vL": [msg.vL.x,msg.vL.y,msg.vL.z],
            "q": [msg.q.x,msg.q.y,msg.q.z],
            "w": [msg.w.x,msg.w.y,msg.w.z],
            "quat": [msg.quat.w,msg.quat.x,msg.quat.y,msg.quat.z],
            "bL": [msg.bL.x,msg.bL.y,msg.bL.z],
            "bQ": [msg.bQ.x,msg.bQ.y,msg.bQ.z],
        }

    def record_data(self):
        """
        在主循环里以固定频率调用，用当前最新数据生成一条记录。
        """
        # 记录无人机数据
        if self.observer_data is not None:
            pQ = self.observer_data["pQ"]
            vQ = self.observer_data["vQ"]
            pL = self.observer_data["pL"]
            vL = self.observer_data["vL"]
            q = self.observer_data["q"]
            w = self.observer_data["w"]
            quat = self.observer_data["quat"]
            bQ = self.observer_data["bQ"]
            bL = self.observer_data["bL"]
            self.observer.append((pQ, vQ, pL, vL, q, w, quat, bQ, bL))
            

        # 记录负载数据
        if self.feedback_data is not None:
            pQ = self.feedback_data["pQ"]
            vQ = self.feedback_data["vQ"]
            pL = self.feedback_data["pL"]
            vL = self.feedback_data["vL"]
            q = self.feedback_data["q"]
            w = self.feedback_data["w"]
            quat = self.feedback_data["quat"]
            bQ = self.feedback_data["bQ"]
            bL = self.feedback_data["bL"]
            self.feedback.append((pQ, vQ, pL, vL, q, w, quat, bQ, bL))
            # rospy.loginfo("pos: %s, vel: %s", pos, vel)

        self.time.append(rospy.Time.now().to_sec())


def main():
    rospy.init_node("multi_topic_save", anonymous=True)
    collector = MultiDimDataCollector()

    # 设定数据记录频率 (Hz)，例如 10Hz
    rate = rospy.Rate(100)

    # 主循环：定时记录最新值
    while not rospy.is_shutdown():
        collector.record_data()
        rate.sleep()

    # ========== 当跳出循环(程序结束)后，把所有记录写进 CSV 文件 ==========
    csv_file_path = os.path.join(os.getcwd(), "multi_dim_data.csv")

    # 由于要在同一个 CSV 中写无人机和负载数据，我们需要先把它们对齐或分别写行。
    
    # 先把各自的记录拆分到 numpy 数组(或直接在for里写都行)
    # times = []
    observer_pQ = []
    observer_vQ = []
    observer_pL = []
    observer_vL = []
    observer_q = []
    observer_w = []
    observer_quat = []
    observer_bQ = []
    observer_bL = []

    for (pQ, vQ, pL, vL, q, w, quat, bQ, bL) in collector.observer:
        observer_pQ.append(pQ)
        observer_vQ.append(vQ)
        observer_pL.append(pL)
        observer_vL.append(vL)
        observer_q.append(q)
        observer_w.append(w)
        observer_quat.append(quat)
        observer_bQ.append(bQ)
        observer_bL.append(bL)

    # 转成 numpy 数组
    times = np.array(collector.time)               # shape: (N,)
    observer_pQ = np.array(observer_pQ)       # shape: (N,3)
    observer_vQ = np.array(observer_vQ)     # shape: (N,3)
    observer_pL = np.array(observer_pL)       # shape: (N,3)
    observer_vL = np.array(observer_vL)     # shape: (N,3)
    observer_q = np.array(observer_q)       # shape: (N,3)
    observer_w = np.array(observer_w)     # shape: (N,3)
    observer_quat = np.array(observer_quat)     # shape: (N,4)
    observer_bQ = np.array(observer_bQ)     # shape: (N,3)
    observer_bL = np.array(observer_bL)     # shape: (N,3)
    # rospy.loginfo("observer_pQ size: %s", observer_pQ.shape)
    # rospy.loginfo("observer_vQ size: %s", observer_vQ.shape)
    
    feedback_pQ = []
    feedback_vQ = []
    feedback_pL = []
    feedback_vL = []
    feedback_q = []
    feedback_w = []
    feedback_quat = []
    feedback_bQ = []
    feedback_bL = []

    for (pQ, vQ, pL, vL, q, w, quat, bQ, bL) in collector.feedback:
        feedback_pQ.append(pQ)
        feedback_vQ.append(vQ)
        feedback_pL.append(pL)
        feedback_vL.append(vL)
        feedback_q.append(q)
        feedback_w.append(w)
        feedback_quat.append(quat)
        feedback_bQ.append(bQ)
        feedback_bL.append(bL)

    # rospy.loginfo("feedback_pQ : %s", feedback_pQ)
    feedback_pQ = np.array(feedback_pQ)
    feedback_vQ = np.array(feedback_vQ)
    feedback_pL = np.array(feedback_pL)
    feedback_vL = np.array(feedback_vL)
    feedback_q = np.array(feedback_q)
    feedback_w = np.array(feedback_w)
    feedback_quat = np.array(feedback_quat)
    feedback_bQ = np.array(feedback_bQ)
    feedback_bL = np.array(feedback_bL)

    # 如果两个数据序列长度不同，就看你想怎么处理：
    # 这里简单地按照最短长度对齐，进行合并写
    # n = min(len(times), len(payload_times))
    n = len(times)
    with open(csv_file_path, 'w', newline='') as f:
        writer = csv.writer(f)
        # 写表头 (你可以修改列名称)
        writer.writerow([
            "time", "observer_pQ_x", "observer_pQ_y", "observer_pQ_z",
            "observer_vQ_x", "observer_vQ_y", "observer_vQ_z",
            "observer_pL_x", "observer_pL_y", "observer_pL_z",
            "observer_vL_x", "observer_vL_y", "observer_vL_z",
            "observer_q_x", "observer_q_y", "observer_q_z",
            "observer_w_x", "observer_w_y", "observer_w_z",
            "observer_quat_w", "observer_quat_x", "observer_quat_y", "observer_quat_z",
            "observer_bQ_x", "observer_bQ_y", "observer_bQ_z",
            "observer_bL_x", "observer_bL_y", "observer_bL_z",
            "feedback_pQ_x", "feedback_pQ_y", "feedback_pQ_z",
            "feedback_vQ_x", "feedback_vQ_y", "feedback_vQ_z",
            "feedback_pL_x", "feedback_pL_y", "feedback_pL_z",
            "feedback_vL_x", "feedback_vL_y", "feedback_vL_z",
            "feedback_q_x", "feedback_q_y", "feedback_q_z",
            "feedback_w_x", "feedback_w_y", "feedback_w_z",
            "feedback_quat_w", "feedback_quat_x", "feedback_quat_y", "feedback_quat_z",
            "feedback_bQ_x", "feedback_bQ_y", "feedback_bQ_z",
            "feedback_bL_x", "feedback_bL_y", "feedback_bL_z",

        ])
        rospy.loginfo("feedback_pQ: %s", feedback_pQ)
        for i in range(n):
                # 将多维向量摊平写入 CSV 的一行
                row = [
                    times[i],
                    observer_pQ[i][0], observer_pQ[i][1], observer_pQ[i][2],
                    observer_vQ[i][0], observer_vQ[i][1], observer_vQ[i][2],
                    observer_pL[i][0], observer_pL[i][1], observer_pL[i][2],
                    observer_vL[i][0], observer_vL[i][1], observer_vL[i][2],
                    observer_q[i][0], observer_q[i][1], observer_q[i][2],
                    observer_w[i][0], observer_w[i][1], observer_w[i][2],
                    observer_quat[i][0], observer_quat[i][1], observer_quat[i][2], observer_quat[i][3],
                    observer_bQ[i][0], observer_bQ[i][1], observer_bQ[i][2],
                    observer_bL[i][0], observer_bL[i][1], observer_bL[i][2],
                    feedback_pQ[i][0], feedback_pQ[i][1], feedback_pQ[i][2],
                    feedback_vQ[i][0], feedback_vQ[i][1], feedback_vQ[i][2],
                    feedback_pL[i][0], feedback_pL[i][1], feedback_pL[i][2],
                    feedback_vL[i][0], feedback_vL[i][1], feedback_vL[i][2],
                    feedback_q[i][0], feedback_q[i][1], feedback_q[i][2],
                    feedback_w[i][0], feedback_w[i][1], feedback_w[i][2],
                    feedback_quat[i][0], feedback_quat[i][1], feedback_quat[i][2], feedback_quat[i][3],
                    feedback_bQ[i][0], feedback_bQ[i][1], feedback_bQ[i][2],
                    feedback_bL[i][0], feedback_bL[i][1], feedback_bL[i][2],
                ]
                writer.writerow(row)

    rospy.loginfo("CSV data saved to: %s", csv_file_path)

if __name__ == '__main__':
    main()