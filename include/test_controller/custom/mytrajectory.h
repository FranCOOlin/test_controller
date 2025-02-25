#ifndef CUSTOM_MY_TRAJECTORY_H
#define CUSTOM_MY_TRAJECTORY_H

#include "test_controller/common/trajectory.h"

namespace common {

// 派生类：MyTrajectory
class MyTrajectory : public Trajectory {


public:
    Eigen::Vector3d pd;
    Eigen::Vector3d dpd;
    Eigen::Vector3d d2pd;
    Eigen::Vector3d d3pd;
    MyTrajectory():
        pd(Eigen::Vector3d(0,0,-1)),
        dpd(Eigen::Vector3d::Zero()),
        d2pd(Eigen::Vector3d::Zero()),
        d3pd(Eigen::Vector3d::Zero())
    {}

    // 重写基类的 setWaypoints 函数
    void setWaypoints(const Eigen::VectorXd& points) override {
        pd = points.segment(0, 3);
        dpd = points.segment(3, 3);
        d2pd = points.segment(6, 3);
        d3pd = points.segment(9, 3);
    }

    // 获取当前的 waypoints
    void getWaypoints(std::vector<Eigen::Vector3d>& waypoints) const override {
        // 将目标位置、速度、加速度添加到 waypoints 向量中
        waypoints.push_back(pd);
        waypoints.push_back(dpd);
        waypoints.push_back(d2pd);
        waypoints.push_back(d3pd);
    }

    ~MyTrajectory() = default;
};

} // namespace common

#endif // CUSTOM_MY_TRAJECTORY_H
