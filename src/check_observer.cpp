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
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <functional>
#include <utility> // std::ref
#include <eigen3/Eigen/Dense>
#include <deque>

// 引入各模块头文件
#include "test_controller/observer/observer.h"
#include "test_controller/observer/observer_scheduler.h"
#include "test_controller/custom/nokov_filter.h"
#include "test_controller/custom/kalman_qsls_observer.h"
#include "test_controller/custom/system_params.h"
#include "test_controller/custom/quadrotor_state.h"
#include "test_controller/custom/mytrajectory.h"
#include "test_controller/common/convert.h"
// ROS 消息
#include <test_controller/UAVState.h>
#include <test_controller/UAVCommand.h>
#include <test_controller/QSLSState.h>

// Callback function for the first topic
void observeCallback(const test_controller::QSLSState::ConstPtr& msg, common::QSLSState& state)
{
    state.pQ = Eigen::Vector3d(msg->pQ.x, msg->pQ.y, msg->pQ.z);
    state.vQ = Eigen::Vector3d(msg->vQ.x, msg->vQ.y, msg->vQ.z);
    state.pL = Eigen::Vector3d(msg->pL.x, msg->pL.y, msg->pL.z);
    state.vL = Eigen::Vector3d(msg->vL.x, msg->vL.y, msg->vL.z);
    state.bQ = Eigen::Vector3d(msg->bQ.x, msg->bQ.y, msg->bQ.z);
    state.bL = Eigen::Vector3d(msg->bL.x, msg->bL.y, msg->bL.z);
    state.q = Eigen::Vector3d(msg->q.x, msg->q.y, msg->q.z);
    state.w = Eigen::Vector3d(msg->w.x, msg->w.y, msg->w.z);
    // ROS_INFO("Received: pQ = [%+.5f, %+.5f, %+.5f], vQ = [%+.5f, %+.5f, %+.5f], pL = [%+.5f, %+.5f, %+.5f], vL = [%+.5f, %+.5f, %+.5f], bQ = [%+.5f, %+.5f, %+.5f], bL = [%+.5f, %+.5f, %+.5f], q = [%+.5f, %+.5f, %+.5f], w = [%+.5f, %+.5f, %+.5f]", state.pQ(0), state.pQ(1), state.pQ(2), state.vQ(0), state.vQ(1), state.vQ(2), state.pL(0), state.pL(1), state.pL(2), state.vL(0), state.vL(1), state.vL(2), state.bQ(0), state.bQ(1), state.bQ(2), state.bL(0), state.bL(1), state.bL(2), state.q(0), state.q(1), state.q(2), state.w(0), state.w(1), state.w(2));
}

// Callback function for the second topic
void myObserveCallback(const test_controller::QSLSState::ConstPtr& msg, common::QSLSState& state)
{
    state.pQ = Eigen::Vector3d(msg->pQ.x, msg->pQ.y, msg->pQ.z);
    state.vQ = Eigen::Vector3d(msg->vQ.x, msg->vQ.y, msg->vQ.z);
    state.pL = Eigen::Vector3d(msg->pL.x, msg->pL.y, msg->pL.z);
    state.vL = Eigen::Vector3d(msg->vL.x, msg->vL.y, msg->vL.z);
    state.bQ = Eigen::Vector3d(msg->bQ.x, msg->bQ.y, msg->bQ.z);
    state.bL = Eigen::Vector3d(msg->bL.x, msg->bL.y, msg->bL.z);
    state.q = Eigen::Vector3d(msg->q.x, msg->q.y, msg->q.z);
    state.w = Eigen::Vector3d(msg->w.x, msg->w.y, msg->w.z);
}

//--------main function--------
int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "observer_node");
    ros::NodeHandle nh;
    // 读取传入参数开始
    std::string uav_id;
    std::string file_path;
    bool simu;
    int observer_rate;
    ros::param::get("~uav_id", uav_id); // 读取当前命名空间下的 uav_id
    ros::param::get("~simulation", simu);
    ros::param::get("~file_path", file_path);
    ros::param::get("~observer_rate", observer_rate);
    // 读取传入参数结束

    common::QSLSState qsls_state;
    common::QSLSState my_qsls_state;

    ros::Subscriber observe_sub = nh.subscribe<test_controller::QSLSState>(uav_id + "/qsls_state", 10, std::bind(observeCallback, std::placeholders::_1, std::ref(qsls_state)));
    ros::Subscriber my_observe_sub = nh.subscribe<test_controller::QSLSState>(uav_id + "/myqsls_state", 10, std::bind(myObserveCallback, std::placeholders::_1, std::ref(my_qsls_state)));

    ros::Rate Rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
        // 运行观测器
        ROS_INFO("Difference: pQ = [%+.5f, %+.5f, %+.5f], vQ = [%+.5f, %+.5f, %+.5f], pL = [%+.5f, %+.5f, %+.5f], vL = [%+.5f, %+.5f, %+.5f], bQ = [%+.5f, %+.5f, %+.5f], bL = [%+.5f, %+.5f, %+.5f], q = [%+.5f, %+.5f, %+.5f], w = [%+.5f, %+.5f, %+.5f]", qsls_state.pQ(0) - my_qsls_state.pQ(0), qsls_state.pQ(1) - my_qsls_state.pQ(1), qsls_state.pQ(2) - my_qsls_state.pQ(2), qsls_state.vQ(0) - my_qsls_state.vQ(0), qsls_state.vQ(1) - my_qsls_state.vQ(1), qsls_state.vQ(2) - my_qsls_state.vQ(2), qsls_state.pL(0) - my_qsls_state.pL(0), qsls_state.pL(1) - my_qsls_state.pL(1), qsls_state.pL(2) - my_qsls_state.pL(2), qsls_state.vL(0) - my_qsls_state.vL(0), qsls_state.vL(1) - my_qsls_state.vL(1), qsls_state.vL(2) - my_qsls_state.vL(2), qsls_state.bQ(0) - my_qsls_state.bQ(0), qsls_state.bQ(1) - my_qsls_state.bQ(1), qsls_state.bQ(2) - my_qsls_state.bQ(2), qsls_state.bL(0) - my_qsls_state.bL(0), qsls_state.bL(1) - my_qsls_state.bL(1), qsls_state.bL(2) - my_qsls_state.bL(2), qsls_state.q(0) - my_qsls_state.q(0), qsls_state.q(1) - my_qsls_state.q(1), qsls_state.q(2) - my_qsls_state.q(2), qsls_state.w(0) - my_qsls_state.w(0), qsls_state.w(1) - my_qsls_state.w(1), qsls_state.w(2) - my_qsls_state.w(2));
        // ROS_INFO("bQ: %+.5f, %+.5f, %+.5f", qsls_state.bQ(0), qsls_state.bQ(1), qsls_state.bQ(2));
        // ROS_INFO("my bQ: %+.5f, %+.5f, %+.5f", my_qsls_state.bQ(0), my_qsls_state.bQ(1), my_qsls_state.bQ(2));
        // Rate.sleep();
    }

return 0;
}