#ifndef CONTROLLER_SCHEDULER_H
#define CONTROLLER_SCHEDULER_H

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "test_controller/controller/controller.h"  // 此头文件中定义了基类 Controller，其 update() 返回类型为 Eigen::VectorXd

namespace controller {

class ControllerScheduler {
public:
  int current_mode;  // 当前控制器编号，即目标 Controller 对象的 registerId
  // 存储 Controller* 的向量，要求所有控制器对象均继承自 Controller 且 update() 返回 Eigen::VectorXd
  std::vector<Controller*> controllers;

  ControllerScheduler() : current_mode(0) {}

  // 注册 Controller 对象，将其添加到容器中，并将该 Controller 的 registerId 设置为其在容器中的索引
  // 返回该注册编号
  int registerController(Controller* ctrl) {
    controllers.push_back(ctrl);
    int index = controllers.size() - 1;
    ctrl->registerId = index;
    ROS_INFO("Controller registered with id: %d, total controllers: %lu", index, controllers.size());
    return index;
  }

  // 切换当前 Controller：直接传入目标 Controller 对象的引用，调度器内部将 current_mode 设置为该对象的 registerId
  void switchController(Controller &ctrl) {
    current_mode = ctrl.registerId;
    ROS_INFO("Switched to controller with registerId: %d", current_mode);
  }

  // run() 函数直接调用当前控制器的 update() 函数，不需要传入额外参数，
  // 并返回控制信号，其类型为 Eigen::VectorXd
  Eigen::VectorXd run() {
    if (controllers.empty()) {
      ROS_WARN("No controllers registered in ControllerScheduler.");
      return Eigen::VectorXd();  // 返回空向量
    }
    int idx = current_mode;
    if (idx < 0 || idx >= static_cast<int>(controllers.size())) {
      ROS_WARN("Current mode %d is out of range. Using controller 0.", idx);
      idx = 0;
    }
    return controllers[idx]->update();
  }
};

} // namespace controller

#endif // CONTROLLER_SCHEDULER_H
