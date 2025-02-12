#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

// MAVROS 消息
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>

#include <functional>
#include <utility>  // std::ref
#include <eigen3/Eigen/Dense>

// 引入各模块头文件
#include "test_controller/common/params.h"          // Params
#include "test_controller/common/state.h"           // State
#include "test_controller/controller/controller.h"  // Controller 抽象基类
#include "test_controller/controller/controller_scheduler.h"  // ControllerScheduler 类
#include "test_controller/custom/mycontroller.h"   // MyController 派生类
#include "test_controller/custom/myparams.h"       // MyParams 派生类
#include "test_controller/custom/mystate.h"        // MyState 派生类
#include "test_controller/custom/mytrajectory.h"      // MyTrajectory 派生类
#include "test_controller/custom/mycontroller.h"           // 用户自定义控制函数 myControlFunction 的声明

// ---------- 全局变量 ----------
// 全局 MAVROS 状态，由 MAVROS 状态话题回调更新
mavros_msgs::State current_state;

// ---------- MAVROS 状态回调 ----------
void status_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
    ROS_INFO("MAVROS state updated: mode=%s, armed=%d", msg->mode.c_str(), msg->armed);
}

// ---------- Offboard/Arming 定时器回调 ----------
// 以引用方式传入 MAVROS 服务客户端
void offboardArmCallback(const ros::TimerEvent &event,
                         ros::ServiceClient &set_mode_client,
                         ros::ServiceClient &arming_client)
{
  if (current_state.mode != "OFFBOARD") {
    ROS_INFO("Switching to OFFBOARD mode");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
      ROS_INFO("OFFBOARD enabled");
    }
  }
  else if (current_state.mode == "OFFBOARD" && !current_state.armed) {
    ROS_INFO("Arming vehicle");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
      ROS_INFO("Vehicle armed");
    }
  }
}

// ---------- 其它回调函数 ----------

void stateCallback(const std_msgs::Float64::ConstPtr &msg, common::State &state)
{
  // state.controller_pos = msg->data;
  ROS_INFO("Control state updated: pos = %f", state.controller_pos);
}

void trajCallback(const std_msgs::Float64MultiArray::ConstPtr &msg, common::MyTrajectory &trajectory)
{
  trajectory.setWaypoints(Eigen::Map<Eigen::VectorXd>(msg->data.data(), msg->data.size()));
  ROS_INFO("Trajectory updated: received %lu waypoints", trajectory.waypoints.size());
}

void trajSwitchCallback(const std_msgs::String::ConstPtr &msg, common::Trajectory &trajectory)
{
  // trajectory.traj_type = msg->data;
  ROS_INFO("Trajectory switched: new type = %s", trajectory.traj_type.c_str());
}

void controllerSWCallback(const std_msgs::Int32::ConstPtr &msg,
                          controller::ControllerScheduler &scheduler)
{
  // 这里直接传入目标 Controller 对象，调用 switchController
  switch (msg->data) {
  case 0:
    scheduler.switchController(*scheduler.controllers[0]);
    ROS_INFO("Switched to controller with registerId: %d", *scheduler.controllers[0]->registerId);
    break;
  default:
    ROS_WARN("Invalid controller switch command: %d", msg->data);
    return;
  }
  
}

// ---------- main() ----------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh("~");

  // 读取 UAV ID 参数（实际系统可能有多个 UAV）
  std::string uav_id;
  ros::param::get("~uav_id", uav_id);
  if (uav_id.empty()) {
    ROS_ERROR("uav_id not set");
    return -1;
  }

  // 订阅 MAVROS 状态话题，话题名称为 uav_id + "/mavros/state"
  ros::Subscriber status_sub = nh.subscribe<mavros_msgs::State>(uav_id + "/mavros/state", 10, status_cb);

  // 创建统一对象（栈变量）
  common::MyParams params;
  common::MyState state;
  common::MyTrajectory trajectory;

  // 加载参数文件
  if (!params.loadFromFile("json/myparams.json")) {
    ROS_ERROR("Failed to load parameters from file.");
    return -1;
  }

  // 打印加载的参数（仅示例）
  ROS_INFO("Loaded parameters:");
  ROS_INFO("controller/kp = %f", params.controller_kp);
  ROS_INFO("controller/ki = %f", params.controller_ki);
  ROS_INFO("controller/kd = %f", params.controller_kd);
  ROS_INFO("observer/noise_std = %f", params.observer_noise_std);
  ROS_INFO("traj_gen/speed = %f", params.traj_gen_speed);
  ROS_INFO("custom/param1 = %f", params.custom_param1);
  ROS_INFO("custom/param2 = %f", params.custom_param2);

  // 初始化 MyController 对象（作为 Controller 的派生类），传入对象引用及用户自定义控制函数 myControlFunction
  controller::MyController myCtrl(params, state, trajectory);
  
  // 初始化 ControllerScheduler 对象（栈变量），先注册 Controller，再调用 switchController
  controller::ControllerScheduler scheduler;
  int regId = scheduler.registerController(&myCtrl);
  scheduler.switchController(myCtrl); // 直接传入 Controller 对象

  // 创建 MAVROS 服务客户端
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  // 初始化控制输入 Publisher
  ros::Publisher control_pub = nh.advertise<std_msgs::Float64>("control_input", 10);

  // 订阅其它话题，使用 std::bind 和 std::ref 传入对象引用
  ros::Subscriber state_sub_local = nh.subscribe<std_msgs::Float64>("state", 10,
    std::bind(stateCallback, std::placeholders::_1, std::ref(state)));
  ros::Subscriber traj_sub = nh.subscribe<std_msgs::Float64MultiArray>("trajectory", 10,
    std::bind(trajCallback, std::placeholders::_1, std::ref(trajectory)));
  ros::Subscriber traj_switch_sub = nh.subscribe<std_msgs::String>("trajswitch", 10,
    std::bind(trajSwitchCallback, std::placeholders::_1, std::ref(trajectory)));
  // 这里直接传 Controller 对象引用
  ros::Subscriber controller_sw_sub = nh.subscribe<std_msgs::Int32>("controller_sw", 10,
    std::bind(controllerSWCallback, std::placeholders::_1, std::ref(params), std::ref(scheduler), std::ref(myCtrl)));

  // 利用 ROS 定时器实现 offboard/arming 切换，每 5 秒触发一次
  ros::Timer offboard_arm_timer = nh.createTimer(ros::Duration(5.0),
    std::bind(offboardArmCallback, std::placeholders::_1, std::ref(set_mode_client), std::ref(arming_client)));

  ros::Rate rate(10);
  double dt = 0.1;

  while (ros::ok())
  {
    ros::spinOnce();

    // 计算误差：目标航点（若存在）与当前 State 的差值
    double error = 0.0;
    if (!trajectory.waypoints.empty()) {
      double target = trajectory.waypoints[0];
      error = target - state.controller_pos;
    }

    // 调用 ControllerScheduler 的 run() 接口，不需要传入参数，直接调用当前控制器的 update()
    Eigen::VectorXd control_signal = scheduler.run();

    // 将 Eigen::VectorXd 的第一个元素作为控制信号发布（根据实际需要转换）
    std_msgs::Float64 ctrl_msg;
    ctrl_msg.data = (control_signal.size() > 0) ? control_signal(0) : 0.0;
    control_pub.publish(ctrl_msg);
    ROS_INFO("Published control signal: %f", ctrl_msg.data);

    rate.sleep();
  }

  return 0;
}
