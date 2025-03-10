#!/usr/bin/env python3

import rospy
import numpy as np
from test_controller.msg import UAVCommand, QSLSState
from scipy.integrate import ode
from numpy.linalg import norm




def S(vec):
    return np.array([0, -vec[2, 0], vec[1, 0],
                     vec[2, 0], 0, -vec[0, 0],
                     -vec[1, 0], vec[0, 0], 0]).reshape(3, 3)

class QSLS():
    def __init__(self, initState, dt=1/200, mq=0.618, ml=0.51, l=0.6):
        self.initState = np.array(initState, dtype=np.float64)
        self.currentState = np.array(initState, dtype=np.float64)
        self.solver = ode(self.fun)
        self.solver.set_integrator('dop853')
        self.dt = dt
        self.ml = ml
        self.mq = mq
        self.g = 9.81
        self.l = l

    def simu(self, action):
        self.solver.set_initial_value(self.currentState, 0.0)
        self.solver.set_f_params(action, self.ml, self.mq, self.g, self.l)
        self.currentState = self.solver.integrate(self.dt)
        if self.solver.successful():
            return self.currentState
        else:
            return None

    def fun(self, t, state, action, ml, mq, g, l):
        e3 = np.array([0, 0, 1]).reshape([3, 1])
        pl = state[0:3].reshape([3, 1])
        vl = state[3:6].reshape([3, 1])
        q = state[6:9].reshape([3, 1])
        w = state[9:12].reshape([3, 1])
        quadAtt = state[12:16]
        R = np.array([
            [-2 * (quadAtt[2]**2 + quadAtt[3]**2) + 1, 2 * (quadAtt[1] * quadAtt[2] - quadAtt[3] * quadAtt[0]), 2 * (quadAtt[1] * quadAtt[3] + quadAtt[2] * quadAtt[0])],
            [2 * (quadAtt[1] * quadAtt[2] + quadAtt[3] * quadAtt[0]), -2 * (quadAtt[1]**2 + quadAtt[3]**2) + 1, 2 * (quadAtt[2] * quadAtt[3] - quadAtt[1] * quadAtt[0])],
            [2 * (quadAtt[1] * quadAtt[3] - quadAtt[2] * quadAtt[0]), 2 * (quadAtt[2] * quadAtt[3] + quadAtt[1] * quadAtt[0]), -2 * (quadAtt[1]**2 + quadAtt[2]**2) + 1]
        ])
        quadAtt = quadAtt.reshape([4, 1])
        # quaternion : w, x, y, z
        T = action[0]
        Omega = np.array(action[1:4]).reshape([3, 1])
        f = - T * np.dot(R, e3)
        dot_w = - 1/(mq*l)*np.dot(S(q),f)
        dot_q = -np.dot(S(q),w)
        dot_vl = 1/(mq+ml)*q*np.dot(q.T,f) - mq*l/(mq+mq)*norm(w)**2*q + g*e3
        dot_pl = vl
        dot_quadAtt = 0.5 * np.dot(np.hstack([np.vstack([0, Omega]), np.vstack([-Omega.T, -S(Omega)])]), quadAtt)
        f = np.concatenate([dot_pl, dot_vl, dot_q, dot_w, dot_quadAtt])
        return f

    def reset(self):
        self.currentState = self.initState
        return self.currentState


class QSLSSimulatorNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('qsls_simulator', anonymous=True)
        uav_id = rospy.get_param('~uav_id', "")
        if uav_id == "":
            rospy.logerr("Please specify the UAV ID")
            # exit(-1)
        else:
            rospy.loginfo("UAV ID: %s", uav_id)
        
        # 初始状态
        init_state = [0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0]
        self.simuRate = 300
        self.mq = 0.618
        self.ml = 0.51
        self.l = 0.6
        self.simu_model = QSLS(init_state,dt=1/self.simuRate,mq=self.mq, ml=self.ml,l=self.l)

        # 订阅控制输入
        self.control_sub = rospy.Subscriber(uav_id + '/control', UAVCommand, self.control_callback)

        # 发布状态
        self.state_pub = rospy.Publisher(uav_id + '/qsls_state', QSLSState, queue_size=10)

        # 控制输入
        self.control_input = [self.simu_model.g*(self.simu_model.mq+self.simu_model.ml), 0, 0, 0]
        # 仿真频率
        self.rate = rospy.Rate(self.simuRate)  # Adjust frequency as needed

    def control_callback(self, msg: UAVCommand):
        # 更新控制输入
        thrust = msg.thrust
        omega = [msg.omega.x, msg.omega.y, msg.omega.z]
        self.control_input = [thrust] + omega

    def run(self):
        while not rospy.is_shutdown():
            # 仿真下一步
            state = self.simu_model.simu(self.control_input)

            #归一化q
            q = state[6:9].reshape([3, 1])
            q = q/norm(q)
            state[6:9] = q.reshape([3,])
            # 发布状态
            if state is not None:
                stateTopic = QSLSState()
                stateTopic.pL.x = state[0]
                stateTopic.pL.y = state[1]
                stateTopic.pL.z = state[2]
                stateTopic.vL.x = state[3]
                stateTopic.vL.y = state[4]
                stateTopic.vL.z = state[5]
                stateTopic.q.x = state[6]
                stateTopic.q.y = state[7]
                stateTopic.q.z = state[8]
                stateTopic.w.x = state[9]
                stateTopic.w.y = state[10]
                stateTopic.w.z = state[11]
                pL = np.array([state[0], state[1], state[2]])
                vL = np.array([state[3], state[4], state[5]])
                q = np.array([state[6], state[7], state[8]])
                w = np.array([state[9], state[10], state[11]])
                pQ = pL - self.l * q
                stateTopic.pQ.x = pQ[0]
                stateTopic.pQ.y = pQ[1]
                stateTopic.pQ.z = pQ[2]
                vQ = vL - np.cross(w, self.l * q)
                stateTopic.vQ.x = vQ[0]
                stateTopic.vQ.y = vQ[1]
                stateTopic.vQ.z = vQ[2]
                stateTopic.quat.w = state[12]
                stateTopic.quat.x = state[13]
                stateTopic.quat.y = state[14]
                stateTopic.quat.z = state[15]
                # rospy.loginfo("State: %s", stateTopic)
                self.state_pub.publish(stateTopic)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        simulator = QSLSSimulatorNode()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
