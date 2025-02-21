#ifndef OBSERVER_SCHEDULER_H
#define OBSERVER_SCHEDULER_H

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "test_controller/observer/observer.h" 

namespace observer {

class ObserverScheduler {
public:
  int current_mode;  // 当前观测器编号，即目标 Observer 对象的 registerId
  // 存储 Observer* 的向量，要求所有 Observer 对象均继承自 Observer
  std::vector<Observer*> observers;

  ObserverScheduler() : current_mode(0) {}

  // 注册 Observer 对象，将其添加到容器中，并将该 Observer 的 registerId 设置为其在容器中的索引
  // 返回该注册编号
  int registerObserver(Observer* obs) {
    observers.push_back(obs);
    int index = observers.size() - 1;
    obs->register_id = index;
    ROS_INFO("Observer registered with id: %d, total observers: %lu", index, observers.size());
    return index;
  }

  // 切换当前 Observer：直接传入目标 Observer 对象的引用，调度器内部将 current_mode 设置为该对象的 registerId
  void switchObserver(Observer &obs) {
    current_mode = obs.register_id;
    ROS_INFO("Switched to observer with registerId: %d", current_mode);
  }

  // run() 函数直接调用当前观察器的 update() 函数，不需要传入额外参数
  void run() {
    if (observers.empty()) {
      ROS_WARN("No observers registered in ObserverScheduler.");
    }
    int idx = current_mode;
    if (idx < 0 || idx >= static_cast<int>(observers.size())) {
      ROS_WARN("Current mode %d is out of range. Using observer 0.", idx);
      idx = 0;
    }
    observers[idx]->update();
  }
};

} // namespace observer

#endif // OBSERVER_SCHEDULER_H
