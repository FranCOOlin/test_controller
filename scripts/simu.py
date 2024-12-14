#!/usr/bin/env python3

import rospy
import numpy as np
# from geometry_msgs.msg import Twist, Pose, Vector3
from test_controller.msg import UAVCommand, UAVState
# from nav_msgs.msg import Odometry

# 引入 QSLS 模型
from scipy.integrate import ode
from numpy.linalg import norm

def S(vec):
    return np.array([0, -vec[2, 0], vec[1, 0],
                     vec[2, 0], 0, -vec[0, 0],
                     -vec[1, 0], vec[0, 0], 0]).reshape(3, 3)

class Quadrotor:
    def __init__(self, initState, dt=1/300, mq=0.33):
        self.initState = np.array(initState, dtype=np.float64)
        self.currentState = np.array(initState, dtype=np.float64)
        self.solver = ode(self.fun)
        self.solver.set_integrator('dop853')
        self.dt = dt
        self.mq = mq
        self.g = 9.81

    def simu(self, action):
        self.solver.set_initial_value(self.currentState, 0.0)
        self.solver.set_f_params(action, self.mq, self.g)
        self.currentState = self.solver.integrate(self.dt)
        if self.solver.successful():
            return self.currentState
        else:
            return None

    def fun(self, t, state, action, mq, g):
        e3 = np.array([0, 0, 1]).reshape([3, 1])
        pq = state[0:3].reshape([3, 1])
        vq = state[3:6].reshape([3, 1])
        quadAtt = state[6:10]
        R = np.array([
            [-2 * (quadAtt[2]**2 + quadAtt[3]**2) + 1, 2 * (quadAtt[1] * quadAtt[2] - quadAtt[3] * quadAtt[0]), 2 * (quadAtt[1] * quadAtt[3] + quadAtt[2] * quadAtt[0])],
            [2 * (quadAtt[1] * quadAtt[2] + quadAtt[3] * quadAtt[0]), -2 * (quadAtt[1]**2 + quadAtt[3]**2) + 1, 2 * (quadAtt[2] * quadAtt[3] - quadAtt[1] * quadAtt[0])],
            [2 * (quadAtt[1] * quadAtt[3] - quadAtt[2] * quadAtt[0]), 2 * (quadAtt[2] * quadAtt[3] + quadAtt[1] * quadAtt[0]), -2 * (quadAtt[1]**2 + quadAtt[2]**2) + 1]
        ])
        quadAtt = quadAtt.reshape([4, 1])
        T = action[0]
        Omega = np.array(action[1:4]).reshape([3, 1])
        f = - T * np.dot(R, e3)
        dot_vq = 1 / mq * f+ g * e3
        dot_pq = vq
        dot_quadAtt = 0.5 * np.dot(np.hstack([np.vstack([0, Omega]), np.vstack([-Omega.T, -S(Omega)])]), quadAtt)
        fun = np.concatenate([dot_pq, dot_vq, dot_quadAtt])
        return fun

    def reset(self):
        self.currentState = self.initState
        return self.currentState


class UAVSimulatorNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('uav_simulator', anonymous=True)

        # 初始状态：位置、速度、方向等
        init_state = [0, 0, 1, 0, 0, 0, 1, 0, 0, 0]
        self.simu_model = Quadrotor(init_state)

        # 订阅控制输入
        self.control_sub = rospy.Subscriber('/uav/control', UAVCommand, self.control_callback)

        # 发布状态
        self.state_pub = rospy.Publisher('/uav/state', UAVState, queue_size=10)

        # 控制输入
        self.control_input = [9.81 * (0.32), 0, 0, 0]  # 默认推力平衡重力

        # 仿真频率
        self.rate = rospy.Rate(300)

    def control_callback(self, msg: UAVCommand):
        # 更新控制输入
        thrust = msg.thrust
        torque = [msg.omega.x, msg.omega.y, msg.omega.z]
        self.control_input = [thrust] + torque

    def run(self):
        while not rospy.is_shutdown():
            # 仿真下一步
            state = self.simu_model.simu(self.control_input)

            # 发布状态
            if state is not None:
                stateTopic = UAVState()
                stateTopic.position.x = state[0]
                stateTopic.position.y = state[1]
                stateTopic.position.z = state[2]
                stateTopic.velocity.x = state[3]
                stateTopic.velocity.y = state[4]
                stateTopic.velocity.z = state[5]
                stateTopic.attitude.w = state[6]
                stateTopic.attitude.x = state[7]
                stateTopic.attitude.y = state[8]
                stateTopic.attitude.z = state[9]
                self.state_pub.publish(stateTopic)
                # odom_msg.header.stamp = rospy.Time.now()
                # self.state_pub.publish(odom_msg)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        simulator = UAVSimulatorNode()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
