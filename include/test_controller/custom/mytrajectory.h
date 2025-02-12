#ifndef CUSTOM_MY_TRAJECTORY_H
#define CUSTOM_MY_TRAJECTORY_H

#include "test_controller/common/trajectory.h"

namespace common {

// 派生类：MyTrajectory
class MyTrajectory : public Trajectory {
private:
    Eigen::VectorXd waypoints;  // 使用 Eigen::VectorXd 存储 waypoints

public:
    MyTrajectory() : waypoints(Eigen::VectorXd(0)) {}

    // 重写基类的 setWaypoints 函数
    void setWaypoints(const Eigen::VectorXd& points) override {
        waypoints = points;  // 直接赋值
    }

    // 获取当前的 waypoints
    Eigen::VectorXd getWaypoints() const override {
        return waypoints;
    }

    ~MyTrajectory() = default;
};

} // namespace common

#endif // CUSTOM_MY_TRAJECTORY_H
