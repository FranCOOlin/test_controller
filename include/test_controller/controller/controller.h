#ifndef CONTROLLER_CONTROLLER_H
#define CONTROLLER_CONTROLLER_H

#include "test_controller/common/params.h"
#include "test_controller/common/state.h"
#include "test_controller/common/trajectory.h"
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

namespace controller {

// 抽象基类 Controller，统一保存 Params、State、Trajectory 引用，以及注册编号。
// update() 函数为纯虚函数，必须在派生类中实现。
class Controller {
public:

  // 控制器注册编号，由调度器在注册时设置
  int registerId;

  // 构造函数：接收 Params、State、Trajectory 的引用
  Controller()
    : registerId(-1)
  { }

  // 虚析构函数
  virtual ~Controller() { }

  // 纯虚函数 update()：返回 Eigen::VectorXd 类型的控制信号
  // 用户必须在派生类中实现此函数来定义具体的控制律
  virtual void update() = 0;

  virtual void initialize() {};
};

} // namespace controller

#endif // CONTROLLER_CONTROLLER_H
