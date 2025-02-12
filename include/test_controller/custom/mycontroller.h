#ifndef CONTROLLER_MYCONTROLLER_H
#define CONTROLLER_MYCONTROLLER_H

#include "test_controller/controller/controller.h"
#include <eigen3/Eigen/Dense>

namespace controller {

// 示例派生类 MyController，实现了 update() 函数
class MyController : public Controller {
public:
  MyController(common::Params &_params, common::State &_state, common::Trajectory &_trajectory)
    : Controller(_params, _state, _trajectory)
  { }

  virtual ~MyController() { }

  // 实现 update()，计算控制信号（此处仅为示例：返回目标航点与当前位置的差值组成的 1 维向量）
  virtual Eigen::VectorXd update() override {
    Eigen::VectorXd output(1);
    if (!trajectory.waypoints.empty()) {
      output(0) = trajectory.waypoints[0] - state.controller_pos;
    } else {
      output(0) = 0.0;
    }
    return output;
  }
};

} // namespace controller

#endif // CONTROLLER_MYCONTROLLER_H
