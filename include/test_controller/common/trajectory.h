#ifndef COMMON_TRAJECTORY_H
#define COMMON_TRAJECTORY_H

#include <eigen3/Eigen/Dense>

namespace common {

// 基类：Trajectory
class Trajectory {
public:
    // 定义一个虚函数，允许派生类设置 waypoints
    virtual void setWaypoints(const Eigen::VectorXd& points) = 0;

    // 获取当前的 waypoints
    virtual Eigen::VectorXd getWaypoints() const = 0;

    virtual ~Trajectory() = default;
};

} // namespace common

#endif // COMMON_TRAJECTORY_H
