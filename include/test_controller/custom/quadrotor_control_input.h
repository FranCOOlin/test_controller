#ifndef CUSTOM_QUADROTOR_CONTROL_INPUT_H
#define CUSTOM_QUADROTOR_CONTROL_INPUT_H

#include "test_controller/common/control_input.h"
#include <eigen3/Eigen/Dense>
namespace common {

class QuadrotorControlInput : public common::ControlInput {
public:
    double thrust;               // 控制输入：推力
    Eigen::Vector3d omega;       // 控制输入：角速度（或旋转速度）
    double mavlink_thrust;    // 归一化推力
    Eigen::Vector3d mavlink_omega; // 角速度
    // 构造函数：初始化 thrust 和 omega
    QuadrotorControlInput()
        : thrust(0), omega(Eigen::Vector3d::Zero()), mavlink_thrust(0), mavlink_omega(Eigen::Vector3d::Zero()){}

};

}

#endif // CUSTOM_QUADROTOR_CONTROL_INPUT_H
