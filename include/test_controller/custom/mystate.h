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
    Eigen::Vector3d acceleration; // 加速度向量

    MyState()
        : position(Eigen::Vector3d::Zero()),
          velocity(Eigen::Vector3d::Zero()),
          acceleration(Eigen::Vector3d::Zero())
    { }

    // 将 update() 改名为 setState()
    // 参数采用按值传递（传入拷贝）
    void setState(Eigen::Vector3d pos,
                  Eigen::Vector3d vel,
                  Eigen::Vector3d acc)
    {
        position = pos;
        velocity = vel;
        acceleration = acc;
        updated = true;
    }
};

} // namespace common

#endif // MY_STATE_H
