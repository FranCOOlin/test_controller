#ifndef COMMON_TRAJECTORY_H
#define COMMON_TRAJECTORY_H

#include <eigen3/Eigen/Dense>

namespace common {

// 基类：Trajectory
class Trajectory {
public:
    // Eigen::VectorXd waypoints;  // 使用 Eigen::VectorXd 存储 waypoints
    // 定义一个虚函数，允许派生类设置 waypoints
    virtual void setWaypoints(const Eigen::VectorXd& points) = 0;

    // 获取当前的 waypoints
    virtual void getWaypoints(std::vector<Eigen::Vector3d>& waypoints) const = 0;

    virtual ~Trajectory() = default;
};

} // namespace common

#endif // COMMON_TRAJECTORY_H
