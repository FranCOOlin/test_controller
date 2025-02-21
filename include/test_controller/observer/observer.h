#ifndef OBSERVER_OBSERVER_H
#define OBSERVER_OBSERVER_H

#include "test_controller/common/params.h"
#include "test_controller/common/measurement.h"
#include "test_controller/common/state.h"
#include "test_controller/common/integrator.hpp"
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

namespace observer {

// 抽象基类 Observer。
// update() 函数为纯虚函数，必须在派生类中实现，用于估计系统状态。
class Observer {
public:
  int register_id;
  double last_update_time;
  Observer()
    : register_id(-1), last_update_time(ros::Time::now().toSec())
  { }

  // 虚析构函数
  virtual ~Observer() { }

  // 纯虚函数 update()：用于估计系统状态，并根据需要更新 `state`
  // 用户必须在派生类中实现此函数来定义具体的估计算法
  virtual void update() = 0;

  // 可选的，观测器初始化函数
  virtual void initialize(){};
};

} // namespace observer

#endif // OBSERVER_OBSERVER_H

