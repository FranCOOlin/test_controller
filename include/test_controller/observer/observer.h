#ifndef OBSERVER_OBSERVER_H
#define OBSERVER_OBSERVER_H

#include "common/integrator.h"
#include "common/params.h"
#include "common/state.h"
#include <ros/ros.h>

namespace observer {

class Observer {
public:
  common::Integrator* integrator;

  Observer() {
    integrator = new common::Integrator();
  }
  ~Observer() {
    delete integrator;
  }

  // 模拟传感器数据读取
  double readSensor() {
    return 42.0;  // 示例返回值
  }

  // 简单积分观测算法
  double observe(double dt) {
    double measurement = readSensor();
    double observed = integrator->integrate(measurement, dt);
    ROS_DEBUG("Observer: measurement=%f, observed=%f", measurement, observed);
    return observed;
  }

  // 更新统一 State 中的 observer_state
  void update(common::State* state, double dt, const common::Params* params) {
    state->observer_state = observe(dt);
  }
};

} // namespace observer

#endif // OBSERVER_OBSERVER_H
