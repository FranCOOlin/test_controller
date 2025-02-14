#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

// MAVROS 消息
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

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
// ROS 消息
#include <test_controller/UAVState.h>
#include <test_controller/UAVCommand.h>

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

void stateCallback(const std_msgs::Float64MultiArray::ConstPtr &msg, common::MyState &state)
{
  // state.controller_pos = msg->data;
  if(msg->data.size() != 10){
    ROS_WARN("Invalid state message: expected 3 elements, received %lu", msg->data.size());
    return;
  }
  Eigen::Map<const Eigen::VectorXd> state_vector(msg->data.data(), msg->data.size());

  state.p = state_vector.segment(0, 3);
  state.vi = state_vector.segment(3, 3);
  state.q = state_vector.segment(6, 4);
  state.R = Eigen::Quaterniond(state.q(0), state.q(1), state.q(2), state.q(3)).toRotationMatrix(); //q: w x y z
  state.euler = state.R.eulerAngles(2, 1, 0);
  state.updated = true;
  // state.setState(state_vector.segment(0, 3), state_vector.segment(3, 3), state_vector.segment(6, 3),);
  // ROS_INFO("Control state updated: pos = %f", state.controller_pos);
}
void simuStateCallback(const std_msgs::Float64MultiArray::ConstPtr &msg, common::MyState &state)
{
  // state.controller_pos = msg->data;
  if(msg->data.size() != 10){
    ROS_WARN("Invalid state message: expected 3 elements, received %lu", msg->data.size());
    return;
  }
  Eigen::Map<const Eigen::VectorXd> state_vector(msg->data.data(), msg->data.size());

  state.p = state_vector.segment(0, 3);
  state.vi = state_vector.segment(3, 3);
  state.q = state_vector.segment(6, 4);
  state.R = Eigen::Quaterniond(state.q(0), state.q(1), state.q(2), state.q(3)).toRotationMatrix(); //q: w x y z
  state.euler = state.R.eulerAngles(2, 1, 0);
  state.updated = true;
  // ROS_INFO("State updated: pos = [%f, %f, %f]", state.p(0), state.p(1), state.p(2));
  // state.setState(state_vector.segment(0, 3), state_vector.segment(3, 3), state_vector.segment(6, 3),);
  // ROS_INFO("Control state updated: pos = %f", state.controller_pos);
}

void trajCallback(const std_msgs::Float64MultiArray::ConstPtr &msg, common::MyTrajectory &trajectory)
{
  // trajectory.setWaypoints(Eigen::Map<Eigen::VectorXd>(msg->data.data(), msg->data.size()));
  // ROS_INFO("Trajectory updated: received %lu waypoints", trajectory.waypoints.size());
}

void trajSwitchCallback(const std_msgs::String::ConstPtr &msg, common::Trajectory &trajectory)
{
  // trajectory.traj_type = msg->data;
  // ROS_INFO("Trajectory switched: new type = %s", trajectory.traj_type.c_str());
}

void controllerSWCallback(const std_msgs::Int32::ConstPtr &msg,
                          controller::ControllerScheduler &scheduler)
{
  // 这里直接传入目标 Controller 对象，调用 switchController
  switch (msg->data) {
  case 0:
    scheduler.switchController(*scheduler.controllers[0]);
    ROS_INFO("Switched to controller with registerId: %d", scheduler.controllers[0]->registerId);
    break;
  default:
    ROS_WARN("Invalid controller switch command: %d", msg->data);
    return;
  }
  
}
int sendCommand(const Eigen::VectorXd &command, ros::Publisher &local_rate_pub, ros::Publisher &local_thrust_pub)
{
  if(command.size() != 4){
    ROS_ERROR("Invalid control command: expected 4 elements, received %lu", command.size());
    return -1;
  }
  geometry_msgs::TwistStamped rate;
  mavros_msgs::Thrust thrust;
  thrust.thrust = command(0);
  rate.twist.angular.y = command(1);
  rate.twist.angular.x = command(2);
  rate.twist.angular.z = command(3);
  local_rate_pub.publish(rate);
  local_thrust_pub.publish(thrust);
  return 0;
}

// ---------- main() ----------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh("");
  // 读取参数
  std::string uav_id;
  std::string file_path;
  bool simu;
  int controller_rate;
  ros::param::get("~uav_id", uav_id);// 读取当前命名空间下的 uav_id
  ros::param::get("~simulation", simu);
  ros::param::get("~file_path", file_path);
  ros::param::get("~controller_rate", controller_rate);
  ROS_INFO("Simulation: %s", simu ? "true" : "false");
  if (uav_id.empty()) {
    ROS_ERROR("uav_id not set");
    return -1;
  }


  // 创建统一对象
  common::MyParams params;
  common::MyState state;
  common::MyTrajectory trajectory;
  common::MyControlInput control_input;

  // 加载参数文件
  if (!params.loadFromFile(file_path)) {
    ROS_ERROR("Failed to load parameters from file.");
    return -1;
  }

  // 打印加载的参数（仅示例）
  ROS_INFO("%s Loaded parameters:", uav_id.c_str());
  ROS_INFO("%s controller/kp = %f", uav_id.c_str(), params.kp);
  ROS_INFO("%s controller/kv = %f", uav_id.c_str(), params.kv);
  ROS_INFO("%s controller/kr = %f", uav_id.c_str(), params.kr);
  ROS_INFO("%s controller/hr = %f", uav_id.c_str(), params.hr);

  // 初始化 MyController 对象（作为 Controller 的派生类），传入对象引用及用户自定义控制函数 myControlFunction
  controller::MyController myCtrl(params, state, trajectory, control_input);
  
  // 初始化 ControllerScheduler 对象（栈变量），先注册 Controller，再调用 switchController
  controller::ControllerScheduler scheduler;
  scheduler.registerController(&myCtrl);
  scheduler.switchController(myCtrl); // 直接传入 Controller 对象

  if(!simu){
    ROS_INFO("Experiment mode");

    // 创建 MAVROS 服务客户端
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_id + "mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_id + "mavros/cmd/arming");
    // Wait for the services to be available
    ros::service::waitForService(uav_id + "/mavros/set_mode");
    ros::service::waitForService(uav_id + "/mavros/cmd/arming");


    // 初始化控制输入 Publisher
    ros::Publisher local_rate_pub = nh.advertise<geometry_msgs::TwistStamped>(uav_id + "/mavros/setpoint_attitude/cmd_vel", 10);
    ros::Publisher local_thrust_pub = nh.advertise<mavros_msgs::Thrust>(uav_id + "/mavros/setpoint_attitude/thrust", 10);
    ros::Publisher control_pub = nh.advertise<test_controller::UAVCommand>(uav_id + "/control", 10);

    // 订阅 MAVROS 状态话题，话题名称为 uav_id + "/mavros/state"，需要px4.launch也在uav_idgroup下
    ros::Subscriber status_sub = nh.subscribe<mavros_msgs::State>(uav_id + "/mavros/state", 10, status_cb);
    
    // 订阅其它话题，使用 std::bind 和 std::ref 传入对象引用
    ros::Subscriber state_sub_local = nh.subscribe<std_msgs::Float64MultiArray>(uav_id+"/state", 10, std::bind(stateCallback, std::placeholders::_1, std::ref(state)));
    ros::Subscriber traj_sub = nh.subscribe<std_msgs::Float64MultiArray>("trajectory", 10, std::bind(trajCallback, std::placeholders::_1, std::ref(trajectory)));
    ros::Subscriber traj_switch_sub = nh.subscribe<std_msgs::String>("trajswitch", 10, std::bind(trajSwitchCallback, std::placeholders::_1, std::ref(trajectory)));
    // 这里直接传 Controller 对象引用
    ros::Subscriber controller_sw_sub = nh.subscribe<std_msgs::Int32>("controller_sw", 10, std::bind(controllerSWCallback, std::placeholders::_1, std::ref(scheduler)));


    // 利用 ROS 定时器实现 offboard/arming 切换，每 5 秒触发一次
    ros::Timer offboard_arm_timer = nh.createTimer(ros::Duration(5.0), std::bind(offboardArmCallback, std::placeholders::_1, std::ref(set_mode_client), std::ref(arming_client)));

    ros::Rate Rate(controller_rate);

    geometry_msgs::TwistStamped rate;
    mavros_msgs::Thrust thrust;
    while (ros::ok())
    {
      ros::spinOnce();
      // 运行控制器
      Eigen::VectorXd control_signal(4);
      scheduler.run();
      control_signal(0) = control_input.thrust;
      control_signal(1) = control_input.omega(0);
      control_signal(2) = control_input.omega(1);
      control_signal(3) = control_input.omega(2);
      // 发送控制指令
      sendCommand(control_signal, local_rate_pub, local_thrust_pub);
      Rate.sleep();
    }
  }
  else{
    ROS_INFO("Simulation mode");

    // 初始化控制输入 Publisher
    ros::Publisher control_pub = nh.advertise<test_controller::UAVCommand>(uav_id + "/control", 10);

    ros::Subscriber state_sub_local = nh.subscribe<std_msgs::Float64MultiArray>(uav_id + "/state", 10, std::bind(simuStateCallback, std::placeholders::_1, std::ref(state)));
    
    // 订阅其它话题，使用 std::bind 和 std::ref 传入对象引用
   
    ros::Subscriber traj_sub = nh.subscribe<std_msgs::Float64MultiArray>("trajectory", 10,
      std::bind(trajCallback, std::placeholders::_1, std::ref(trajectory)));
    ros::Subscriber traj_switch_sub = nh.subscribe<std_msgs::String>("trajswitch", 10,
      std::bind(trajSwitchCallback, std::placeholders::_1, std::ref(trajectory)));
    // 这里直接传 Controller 对象引用
    ros::Subscriber controller_sw_sub = nh.subscribe<std_msgs::Int32>("controller_sw", 10,
      std::bind(controllerSWCallback, std::placeholders::_1, std::ref(scheduler)));

    ros::Rate Rate(controller_rate);

    geometry_msgs::TwistStamped rate;
    mavros_msgs::Thrust thrust;
    while (ros::ok())
    {
      ros::spinOnce();
      // 运行控制器      
      scheduler.run();
      // 发送控制指令
      test_controller::UAVCommand command_msg;
      command_msg.thrust = control_input.thrust;
      command_msg.omega.x = control_input.omega(0);
      command_msg.omega.y = control_input.omega(1);
      command_msg.omega.z = control_input.omega(2);
      control_pub.publish(command_msg);
      // ROS_INFO("Published command: thrust = %f, omega = [%f, %f, %f]", control_signal(0), control_signal(1), control_signal(2), control_signal(3));
      Rate.sleep();
    }
  }

  return 0;
}
