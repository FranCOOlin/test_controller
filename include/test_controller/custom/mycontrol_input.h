#ifndef CUSTOM_MYCONTROL_INPUT_H
#define CUSTOM_MYCONTROL_INPUT_H

#include "test_controller/common/control_input.h"
#include <eigen3/Eigen/Dense>

namespace common {

class MyControlInput : public common::ControlInput {
public:
    double thrust;               // 控制输入：推力
    Eigen::Vector3d omega;       // 控制输入：角速度（或旋转速度）

    // 构造函数：初始化 thrust 和 omega
    MyControlInput()
        : thrust(0), omega(Eigen::Vector3d::Zero()){}

};

} // namespace custom

#endif // CUSTOM_MYCONTROL_INPUT_H
