#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped
import curses
import time

S = np.empty((3, 0))  # 原始ADC数据
N = np.empty((3, 0))  # 用户输入的标签（0, 0, x）
latest_vector = None

def callback(msg):
    global latest_vector
    latest_vector = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

# 在 curses 中输入字符串，带默认值
def input_string(stdscr, prompt, default=""):
    curses.echo()
    stdscr.addstr(2, 0, " " * 50)  # 清除旧内容
    stdscr.addstr(2, 0, prompt)
    stdscr.refresh()
    input_str = stdscr.getstr(2, len(prompt)).decode("utf-8").strip()
    curses.noecho()
    return input_str if input_str != "" else default

def main(stdscr):
    global S, N

    rospy.init_node('triple_adc_listener', anonymous=True)
    rospy.Subscriber('/triple_adc_value', Vector3Stamped, callback)
    rate = rospy.Rate(100)

    curses.curs_set(0)
    stdscr.clear()
    stdscr.addstr(0, 0, "Listening... Press [SPACE] to record label, [Ctrl+C] to exit.")
    stdscr.refresh()

    last_input_value = "0.0"

    try:
        while not rospy.is_shutdown():
            key = stdscr.getch()
            if key == 32:  # 空格键
                if latest_vector is not None:
                    S = np.hstack((S, latest_vector.reshape(3, 1)))

                prompt = f"Enter label value (default: {last_input_value}): "
                user_input = input_string(stdscr, prompt, last_input_value)

                try:
                    value = float(user_input)
                    last_input_value = user_input  # 更新默认值
                except ValueError:
                    stdscr.addstr(3, 0, "Invalid input, using 0.0")
                    value = 0.0

                new_vec = np.array([[0], [0], [value]])
                N = np.hstack((N, new_vec))

                stdscr.clear()
                stdscr.addstr(0, 0, f"Saved data. Matrix S shape: {S.shape}, N shape: {N.shape}")
                stdscr.refresh()

            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        rospy.signal_shutdown("User terminated")

    return S, N

if __name__ == '__main__':
    try:
        S_mat, N_mat = curses.wrapper(main)
        print("\n[CTRL+C] Program exited.")
        print("Final ADC matrix S shape:", S_mat.shape)
        print("S =\n", S_mat)
        print("Final label matrix N shape:", N_mat.shape)
        print("N =\n", N_mat)
    except Exception as e:
        print("Unhandled exception:", e)
