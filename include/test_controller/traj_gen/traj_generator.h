#ifndef TRAJ_GEN_TRAJ_GENERATOR_H
#define TRAJ_GEN_TRAJ_GENERATOR_H

#include "common/params.h"
#include "common/state.h"
#include "common/trajectory.h"
#include <ros/ros.h>
#include <string>
#include <vector>

namespace traj_gen {

class TrajectoryGenerator {
public:
  common::Params* params;
  common::State* state;
  common::Trajectory* trajectory; // 内部生成的轨迹对象

  TrajectoryGenerator(common::Params* _params, common::State* _state)
    : params(_params), state(_state)
  {
    trajectory = new common::Trajectory();
  }

  ~TrajectoryGenerator() {
    delete trajectory;
  }

  // 根据轨迹类型生成对应轨迹（默认实现）
  void generateTrajectory(const std::string& type) {
    trajectory->traj_type = type;
    trajectory->waypoints = {0.0, params->traj_gen_speed,
                             params->traj_gen_speed * 2,
                             params->traj_gen_speed * 3};
    ROS_INFO("Trajectory generated (default): type=%s", type.c_str());
  }
};

} // namespace traj_gen

#endif // TRAJ_GEN_TRAJ_GENERATOR_H
