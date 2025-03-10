#ifndef CUSTOM_QSLS_STATE_H
#define CUSTOM_QSLS_STATE_H

#include "test_controller/common/state.h"
#include <eigen3/Eigen/Dense>

namespace common {

class QSLSState : public State {
public:
    // Load
    Eigen::Vector3d pL;     // Load position
    Eigen::Vector3d vL;     // Load velocity
    Eigen::Vector3d q;      // Load direction
    Eigen::Vector3d w;  // Load angular velocity
    // Quadrotor
    Eigen::Vector3d pQ;     // Quadrotor position
    Eigen::Vector3d vQ;     // Quadrotor velocity
    Eigen::Quaterniond quat;     // Quaternion
    Eigen::Vector3d euler; // Euler angles
    Eigen::Matrix3d R;     // Rotation matrix   
    Eigen::Vector3d omega;  // Quadrotor angular speed(not used in angular rate cmd mode)

    //Disturbance
    Eigen::Vector3d bL;
    Eigen::Vector3d bQ;

    QSLSState()
        : pL(Eigen::Vector3d::Zero()),
        vL(Eigen::Vector3d::Zero()),
        q(Eigen::Vector3d(0,0,1)),
        w(Eigen::Vector3d::Zero()),
        pQ(Eigen::Vector3d(0,0,-0.6)),
        vQ(Eigen::Vector3d::Zero()),
        quat(Eigen::Quaterniond::Identity()),
        euler(Eigen::Vector3d::Zero()),
        R(Eigen::Matrix3d::Identity()),
        omega(Eigen::Vector3d::Zero()),
        bL(Eigen::Vector3d::Zero()),
        bQ(Eigen::Vector3d::Zero())
    { }

    // 将 update() 改名为 setState()
    // 参数采用按值传递（传入拷贝）
    void setState(  Eigen::Vector3d _pL,
                    Eigen::Vector3d _vL,
                    Eigen::Vector3d _q,
                    Eigen::Vector3d _omega,
                    Eigen::Vector3d _pQ,
                    Eigen::Vector3d _vQ,
                    Eigen::Quaterniond _quat,
                    Eigen::Vector3d _euler,
                    Eigen::Matrix3d _R,
                    Eigen::Vector3d _bL,
                    Eigen::Vector3d _bQ)
    {
        pL = _pL;
        vL = _vL;
        q = _q;
        omega = _omega;
        pQ = _pQ;
        vQ = _vQ;
        quat = _quat;
        euler = _euler;
        R = _R;
        bL = _bL;
        bQ = _bQ;
    }
};

} // namespace common

#endif // MY_STATE_H
