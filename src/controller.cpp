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
#include <utility> // std::ref
#include <eigen3/Eigen/Dense>

// 引入各模块头文件
#include "test_controller/common/params.h"                   // Params
#include "test_controller/common/state.h"                    // State
#include "test_controller/controller/controller.h"           // Controller 抽象基类
#include "test_controller/controller/controller_scheduler.h" // ControllerScheduler 类
#include "test_controller/custom/quadrotor_ganyu.h"          // MyController 派生类
#include "test_controller/custom/saturated_backstepping.h"   // MyController 派生类
#include "test_controller/custom/system_params.h"            // SystemParams 派生类
#include "test_controller/custom/quadrotor_state.h"          // MyState 派生类
#include "test_controller/custom/qsls_state.h"               // MyState 派生类
#include "test_controller/custom/mytrajectory.h"             // MyTrajectory 派生类
// ROS 消息
#include <test_controller/UAVState.h>
#include <test_controller/UAVCommand.h>
#include <test_controller/QSLSState.h>
#include <test_controller/TrajPoint.h>


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
  if (current_state.mode != "OFFBOARD")
  {
    ROS_INFO("Switching to OFFBOARD mode");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
      ROS_INFO("OFFBOARD enabled");
    }
  }
  if (current_state.mode == "OFFBOARD" && !current_state.armed)
  {
    ROS_INFO("Arming vehicle");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
      ROS_INFO("Vehicle armed");
    }
  }
}

// ---------- 其它回调函数 ----------

void stateCallback(const std_msgs::Float64MultiArray::ConstPtr &msg, common::QuadrotorState &state)
{
  // state.controller_pos = msg->data;
  if (msg->data.size() != 10)
  {
    ROS_WARN("Invalid state message: expected 3 elements, received %lu", msg->data.size());
    return;
  }
  Eigen::Map<const Eigen::VectorXd> state_vector(msg->data.data(), msg->data.size());

  state.p = state_vector.segment(0, 3);
  state.vi = state_vector.segment(3, 3);
  state.q = Eigen::Quaterniond(state_vector(6), state_vector(7), state_vector(8), state_vector(9)); // q: w x y z
  state.R = state.q.toRotationMatrix(); // q: w x y z
  state.euler = state.R.eulerAngles(2, 1, 0);
  state.updated = true;
  bool updated = state.updated;
  // state.setState(state_vector.segment(0, 3), state_vector.segment(3, 3), state_vector.segment(6, 3),);
  // ROS_INFO("Control state updated: pos = %+.5f,%+.5f,%+.5f", state.p(0),state.p(1),state.p(2));
}
void simuStateCallback(const std_msgs::Float64MultiArray::ConstPtr &msg, common::QuadrotorState &state)
{
  // state.controller_pos = msg->data;
  if (msg->data.size() != 10)
  {
    ROS_WARN("Invalid state message: expected 3 elements, received %lu", msg->data.size());
    return;
  }
  Eigen::Map<const Eigen::VectorXd> state_vector(msg->data.data(), msg->data.size());

  state.p = state_vector.segment(0, 3);
  state.vi = state_vector.segment(3, 3);
  state.q = Eigen::Quaterniond(state_vector(6), state_vector(7), state_vector(8), state_vector(9)); // q: w x y z
  state.R = state.q.toRotationMatrix(); // q: w x y z
  state.euler = state.R.eulerAngles(2, 1, 0);
  state.updated = true;
  // ROS_INFO("Quadrotor State updated: pos = [%+.5f, %+.5f, %+.5f]", state.p(0), state.p(1), state.p(2));
  // state.setState(state_vector.segment(0, 3), state_vector.segment(3, 3), state_vector.segment(6, 3),);
  // ROS_INFO("Control state updated: pos = %f", state.controller_pos);
}
void QSLSStateCallback(const test_controller::QSLSState::ConstPtr &msg, common::QSLSState &state)
{
  state.pL = Eigen::Vector3d(msg->pL.x, msg->pL.y, msg->pL.z);
  state.vL = Eigen::Vector3d(msg->vL.x, msg->vL.y, msg->vL.z);
  state.q = Eigen::Vector3d(msg->q.x, msg->q.y, msg->q.z);
  state.w = Eigen::Vector3d(msg->w.x, msg->w.y, msg->w.z);
  state.pQ = Eigen::Vector3d(msg->pQ.x, msg->pQ.y, msg->pQ.z);
  state.vQ = Eigen::Vector3d(msg->vQ.x, msg->vQ.y, msg->vQ.z);
  state.quat = Eigen::Quaterniond(msg->quat.w, msg->quat.x, msg->quat.y, msg->quat.z);
  state.R = state.quat.toRotationMatrix();
  state.bL = Eigen::Vector3d(msg->bL.x, msg->bL.y, msg->bL.z);
  state.bQ = Eigen::Vector3d(msg->bQ.x, msg->bQ.y, msg->bQ.z);
  state.updated = true;
  // ROS_INFO("QSLS State updated: pos = [%+.5f, %+.5f, %+.5f]", state.pL(0), state.pL(1), state.pL(2));
}
void simuQSLSStateCallback(const test_controller::QSLSState::ConstPtr &msg, common::QSLSState &state)
{
  state.pL = Eigen::Vector3d(msg->pL.x, msg->pL.y, msg->pL.z);
  state.vL = Eigen::Vector3d(msg->vL.x, msg->vL.y, msg->vL.z);
  state.q = Eigen::Vector3d(msg->q.x, msg->q.y, msg->q.z);
  state.w = Eigen::Vector3d(msg->w.x, msg->w.y, msg->w.z);
  state.pQ = Eigen::Vector3d(msg->pQ.x, msg->pQ.y, msg->pQ.z);
  state.vQ = Eigen::Vector3d(msg->vQ.x, msg->vQ.y, msg->vQ.z);
  state.quat = Eigen::Quaterniond(msg->quat.w, msg->quat.x, msg->quat.y, msg->quat.z);
  state.R = state.quat.toRotationMatrix();
  state.bL = Eigen::Vector3d(msg->bL.x, msg->bL.y, msg->bL.z);
  state.bQ = Eigen::Vector3d(msg->bQ.x, msg->bQ.y, msg->bQ.z);
  state.updated = true;
  // ROS_INFO("QSLS State updated: pos = [%+.5f, %+.5f, %+.5f]", state.pL(0), state.pL(1), state.pL(2));
}

void trajCallback(const test_controller::TrajPoint::ConstPtr &msg, common::MyTrajectory &trajectory)
{
  Eigen::Map<const Eigen::VectorXd> state_vector(msg->data.data(), msg->data.size());
  trajectory.pd = state_vector.segment(0, 3);
  trajectory.dpd = state_vector.segment(3, 3);
  trajectory.d2pd = state_vector.segment(6, 3);
  trajectory.d3pd = state_vector.segment(9, 3);
  trajectory.d4pd = state_vector.segment(12, 3);
  trajectory.d5pd = state_vector.segment(15, 3);
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
  switch (msg->data)
  {
  case 0: // ganyu controller
    scheduler.switchController(*scheduler.controllers[0]);
    break;
  case 1: // qsls controller
    if (scheduler.current_mode == 0)
    { // 仅支持从0切换到1
      auto SaturatedBacksteppingPtr = static_cast<controller::SaturatedBackstepping *>(scheduler.controllers[1]);
      auto NokovFilterPtr = static_cast<controller::QuadrotorControllerGanYu *>(scheduler.controllers[0]);
      // 控制器部分不需要做过多操作
      scheduler.switchController(*scheduler.controllers[1]);
    }
    break;
  default:
    ROS_WARN("Invalid controller switch command: %d", msg->data);
    return;
  }
}
int sendCommandMavros(const Eigen::VectorXd &command, ros::Publisher &local_rate_pub, ros::Publisher &local_thrust_pub)
{
  if (command.size() != 4)
  {
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
  // ROS_INFO("Control thrust updated: thrust = %f",thrust.thrust);
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh("");
  // 读取传入参数开始
  std::string uav_id;
  std::string file_path;
  bool simu;
  int controller_rate;
  ros::param::get("~uav_id", uav_id);
  ros::param::get("~simulation", simu);
  ros::param::get("~file_path", file_path);
  ros::param::get("~controller_rate", controller_rate);
  ROS_INFO("Simulation: %s", simu ? "true" : "false");
  if (uav_id.empty())
  {
    ROS_ERROR("uav_id not set");
    return -1;
  }
  // 读取传入参数结束

  // 创建统一对象
  common::SystemParams params;
  common::QuadrotorState quadrotor_state;
  common::QSLSState qsls_state;
  common::MyTrajectory trajectory;
  common::QuadrotorControlInput control_input;

  // 加载参数文件
  if (!params.loadFromFile(file_path))
  {
    ROS_ERROR("Failed to load parameters from file.");
    return -1;
  }

  // 打印加载的参数（仅示例）
  ROS_INFO("%s Loaded parameters:", uav_id.c_str());
  ROS_INFO("%s controller/quadrotor/kp = %f", uav_id.c_str(), params.quadrotor_kp);
  ROS_INFO("%s controller/quadrotor/kv = %f", uav_id.c_str(), params.quadrotor_kv);
  ROS_INFO("%s controller/quadrotor/kr = %f", uav_id.c_str(), params.quadrotor_kr);
  ROS_INFO("%s controller/quadrotor/hr = %f", uav_id.c_str(), params.quadrotor_hr);

  // // For QSLS saturated backstepping controller
  // double QSLS_bar_mQ;
  // double QSLS_bar_mL;
  // double QSLS_l;
  // double QSLS_k1;
  // double QSLS_beta;
  // double QSLS_ks1;
  // double QSLS_k2;
  // double QSLS_ks2;
  // double QSLS_hq;
  // double QSLS_kq;
  // double QSLS_hw;
  // double QSLS_kw;
  // double QSLS_hr;
  // double QSLS_kr;

  ROS_INFO("%s controller/qsls/bar_mQ = %f", uav_id.c_str(), params.QSLS_bar_mQ);
  ROS_INFO("%s controller/qsls/bar_mQ = %f", uav_id.c_str(), params.QSLS_bar_mQ);
  ROS_INFO("%s controller/qsls/l = %f", uav_id.c_str(), params.QSLS_l);
  ROS_INFO("%s controller/qsls/k1 = %f", uav_id.c_str(), params.QSLS_k1);
  ROS_INFO("%s controller/qsls/beta = %f", uav_id.c_str(), params.QSLS_beta);
  ROS_INFO("%s controller/qsls/ks1 = %f", uav_id.c_str(), params.QSLS_ks1);
  ROS_INFO("%s controller/qsls/k2 = %f", uav_id.c_str(), params.QSLS_k2);
  ROS_INFO("%s controller/qsls/ks2 = %f", uav_id.c_str(), params.QSLS_ks2);
  ROS_INFO("%s controller/qsls/hq = %f", uav_id.c_str(), params.QSLS_hq);
  ROS_INFO("%s controller/qsls/kq = %f", uav_id.c_str(), params.QSLS_kq);
  ROS_INFO("%s controller/qsls/hw = %f", uav_id.c_str(), params.QSLS_hw);
  ROS_INFO("%s controller/qsls/kw = %f", uav_id.c_str(), params.QSLS_kw);
  ROS_INFO("%s controller/qsls/hr = %f", uav_id.c_str(), params.QSLS_hr);
  ROS_INFO("%s controller/qsls/kr = %f", uav_id.c_str(), params.QSLS_kr);

  // 初始化 MyController 对象（作为 Controller 的派生类），传入对象引用及用户自定义控制函数 myControlFunction
  controller::QuadrotorControllerGanYu quadrotor_ctrl(params, quadrotor_state, trajectory, control_input);
  controller::SaturatedBackstepping qsls_ctrl(params, qsls_state, trajectory, control_input);
  // 初始化 ControllerScheduler 对象（栈变量），先注册 Controller，再调用 switchController
  controller::ControllerScheduler scheduler;
  scheduler.registerController(&quadrotor_ctrl);
  scheduler.registerController(&qsls_ctrl);
  scheduler.switchController(quadrotor_ctrl);

  if (!simu)
  {
    ROS_INFO("Experiment mode");

    // 创建 MAVROS 服务客户端
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_id + "/mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_id + "/mavros/cmd/arming");
    // 等待services to be available
    ros::service::waitForService(uav_id + "/mavros/set_mode");
    ros::service::waitForService(uav_id + "/mavros/cmd/arming");

    // 初始化控制输入 Publisher
    ros::Publisher local_rate_pub = nh.advertise<geometry_msgs::TwistStamped>(uav_id + "/mavros/setpoint_attitude/cmd_vel", 10);
    ros::Publisher local_thrust_pub = nh.advertise<mavros_msgs::Thrust>(uav_id + "/mavros/setpoint_attitude/thrust", 10);
    ros::Publisher control_pub = nh.advertise<test_controller::UAVCommand>(uav_id + "/control", 10);

    // 订阅 MAVROS 状态话题，话题名称为 uav_id + "/mavros/state"，需要px4.launch也在uav_id的ns下
    ros::Subscriber status_sub = nh.subscribe<mavros_msgs::State>(uav_id + "/mavros/state", 10, status_cb);

    // 订阅状态反馈话题，使用 std::bind 和 std::ref 传入对象引用
    ros::Subscriber state_sub = nh.subscribe<std_msgs::Float64MultiArray>(uav_id + "/quadrotor_state", 10, std::bind(stateCallback, std::placeholders::_1, std::ref(quadrotor_ctrl.state)));
    ros::Subscriber qsls_state_sub = nh.subscribe<test_controller::QSLSState>(uav_id + "/qsls_state", 10, std::bind(QSLSStateCallback, std::placeholders::_1, std::ref(qsls_ctrl.state)));

    // 订阅轨迹话题
    ros::Subscriber traj_sub = nh.subscribe<test_controller::TrajPoint>(uav_id + "/trajectory", 10, std::bind(trajCallback, std::placeholders::_1, std::ref(quadrotor_ctrl.trajectory)));

    // 订阅其他话题
    ros::Subscriber traj_switch_sub = nh.subscribe<std_msgs::String>(uav_id + "/trajswitch", 10, std::bind(trajSwitchCallback, std::placeholders::_1, std::ref(quadrotor_ctrl.trajectory)));
    ros::Subscriber controller_sw_sub = nh.subscribe<std_msgs::Int32>(uav_id + "/controller_sw", 10, std::bind(controllerSWCallback, std::placeholders::_1, std::ref(scheduler)));

    // 利用 ROS 定时器实现 offboard/arming 切换，每 5 秒触发一次
    ros::Timer offboard_arm_timer = nh.createTimer(ros::Duration(5.0), std::bind(offboardArmCallback, std::placeholders::_1, std::ref(set_mode_client), std::ref(arming_client)));
    // 设置循环执行频率
    ros::Rate Rate(controller_rate);

    while (ros::ok())
    {
      ros::spinOnce();
      // 运行控制器
      Eigen::VectorXd control_signal(4);
      scheduler.run();
      // 控制器同时只能发布一个控制指令(因为执行对象只有一个)
      control_signal(0) = control_input.mavlink_thrust;
      control_signal(1) = control_input.mavlink_omega(0);
      control_signal(2) = control_input.mavlink_omega(1);
      control_signal(3) = control_input.mavlink_omega(2);
      // 发送控制指令到飞控
      sendCommandMavros(control_signal, local_rate_pub, local_thrust_pub);
      
      test_controller::UAVCommand command_msg;
      command_msg.thrust = control_input.thrust;
      command_msg.omega.x = control_input.omega(0);
      command_msg.omega.y = control_input.omega(1);
      command_msg.omega.z = control_input.omega(2);
      // 发送控制指令到仿真环境
      control_pub.publish(command_msg);
      Rate.sleep();
    }
  }
  else
  {
    ROS_INFO("Simulation mode");

    // 初始化控制输入 Publisher
    ros::Publisher control_pub = nh.advertise<test_controller::UAVCommand>(uav_id + "/control", 10);
    // 订阅状态反馈话题，使用 std::bind 和 std::ref 传入对象引用
    ros::Subscriber state_sub = nh.subscribe<std_msgs::Float64MultiArray>(uav_id + "/quadrotor_state", 10, std::bind(stateCallback, std::placeholders::_1, std::ref(quadrotor_ctrl.state)));
    ros::Subscriber qsls_state_sub = nh.subscribe<test_controller::QSLSState>(uav_id + "/qsls_state", 10, std::bind(simuQSLSStateCallback, std::placeholders::_1, std::ref(qsls_ctrl.state)));

    // 订阅轨迹话题
    ros::Subscriber traj_sub = nh.subscribe<test_controller::TrajPoint>("trajectory", 10,std::bind(trajCallback, std::placeholders::_1, std::ref(quadrotor_ctrl.trajectory)));

    // 订阅其它话题
    ros::Subscriber controller_sw_sub = nh.subscribe<std_msgs::Int32>(uav_id + "/controller_sw", 10,std::bind(controllerSWCallback, std::placeholders::_1, std::ref(scheduler)));
    ros::Subscriber traj_switch_sub = nh.subscribe<std_msgs::String>("trajswitch", 10,std::bind(trajSwitchCallback, std::placeholders::_1, std::ref(quadrotor_ctrl.trajectory)));

    ros::Rate Rate(controller_rate);

    geometry_msgs::TwistStamped rate;
    mavros_msgs::Thrust thrust;
    while (ros::ok())
    {
      ros::spinOnce();
      // 运行控制器
      qsls_ctrl.control_input.thrust = qsls_ctrl.params.g * (qsls_ctrl.params.QSLS_bar_mL + qsls_ctrl.params.QSLS_bar_mQ); // 先置为重力
      double t_start = ros::Time::now().toSec();
      scheduler.run();
      double t_end = ros::Time::now().toSec();
      ROS_INFO("Controller time: %f", t_end - t_start);
      // 发送控制指令
      test_controller::UAVCommand command_msg;
      command_msg.thrust = control_input.thrust;
      command_msg.omega.x = control_input.omega(0);
      command_msg.omega.y = control_input.omega(1);
      command_msg.omega.z = control_input.omega(2);
      control_pub.publish(command_msg);
      // ROS_INFO("Curent Time: %f",ros::Time::now().toSec());
      // ROS_INFO("Published command: thrust = %f, omega = [%f, %f, %f]", command_msg.thrust, command_msg.omega.x, command_msg.omega.y, command_msg.omega.z);
      Rate.sleep();
    }
  }

  return 0;
}
