#ifndef MY_STATE_H
#define MY_STATE_H

#include "test_controller/common/state.h"
#include <eigen3/Eigen/Dense>

namespace common {

class MyState : public State {
public:
    Eigen::Vector3d p;     // 位置向量
    Eigen::Vector3d vi;     // Inertial frame velocity
    Eigen::Vector3d vb;     // Body frame velocity
    Eigen::Vector4d q;     // 四元数
    Eigen::Vector3d euler; // 欧拉角
    Eigen::Matrix3d R;     // 旋转矩阵

    MyState()
        : p(Eigen::Vector3d::Zero()),
        vi(Eigen::Vector3d::Zero()),
        vb(Eigen::Vector3d::Zero()),
        q(Eigen::Vector4d::Zero()),
        euler(Eigen::Vector3d::Zero()),
        R(Eigen::Matrix3d::Identity())
    { }

    // 将 update() 改名为 setState()
    // 参数采用按值传递（传入拷贝）
    void setState(  Eigen::Vector3d _p,
                    Eigen::Vector3d _vi,
                    Eigen::Vector3d _vb,
                    Eigen::Vector4d _q,
                    Eigen::Matrix3d _R,
                    Eigen::Vector3d _euler)
    {
        p = _p;
        vi = _vi;
        vb = _vb;
        q = _q;
        R = _R;
        euler = _euler;
        updated = true;
    }
};

} // namespace common

#endif // MY_STATE_H
