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
#include <utility>  // std::ref
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




void observerSWCallback(const std_msgs::Int32::ConstPtr& msg, observer::ObserverScheduler &scheduler)
{
    switch (msg->data) {
        case 0: // ganyu controller
          scheduler.switchObserver(*scheduler.observers[0]);
          break;
        case 1: // qsls controller
          // 切换时观测器状态重置
          if(scheduler.current_mode==0){// 仅支持从0切换到1
            auto KalmanQSLSObserverPtr = static_cast<observer::KalmanQSLSObserver*>(scheduler.observers[1]);
            auto NokovFilterPtr = static_cast<observer::NokovFilter*>(scheduler.observers[0]);
            KalmanQSLSObserverPtr->state.pQ = NokovFilterPtr->state.p;
            KalmanQSLSObserverPtr->state.vQ = Eigen::Vector3d::Zero();
            KalmanQSLSObserverPtr->state.quat = Eigen::Quaterniond(NokovFilterPtr->state.q);
            KalmanQSLSObserverPtr->state.pL = KalmanQSLSObserverPtr->state.pQ+KalmanQSLSObserverPtr->params.QSLS_l*Eigen::Vector3d(0,0,1);
            KalmanQSLSObserverPtr->state.vL = Eigen::Vector3d::Zero();
            scheduler.switchObserver(*scheduler.observers[1]);
          }
          else{
            ROS_WARN("Invalid observer switch from %d to %d", scheduler.current_mode, msg->data);
            return;
          }
          break;
        default:
          ROS_WARN("Invalid observer switch command: %d", msg->data);
          return;
        }
}


void feedbackCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, common::NokovWithForce &measurement)
{
    measurement.p = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    measurement.attitude = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}
void simuFeedbackCallback(const test_controller::UAVState::ConstPtr& msg, common::NokovWithForce &measurement)
{
    // 从消息中提取位置和姿态
    measurement.p = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    measurement.attitude = Eigen::Quaterniond(msg->attitude.w, msg->attitude.x, msg->attitude.y, msg->attitude.z);
    // ROS_INFO("Feedback received: p = %f %f %f, q = %f %f %f %f", measurement.p(0), measurement.p(1), measurement.p(2), measurement.attitude.w(), measurement.attitude.x(), measurement.attitude.y(), measurement.attitude.z());
}

void simuQSLSpQFeedbackCallback(const test_controller::UAVState::ConstPtr& msg, common::NokovWithForce &measurement)
{
    // 从消息中提取位置和姿态
    measurement.p = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    measurement.attitude = Eigen::Quaterniond(msg->attitude.w, msg->attitude.x, msg->attitude.y, msg->attitude.z);
    // ROS_INFO("Feedback received: p = %+.5f %+.5f %+.5f, q = %+.5f %+.5f %+.5f %+.5f", measurement.p(0), measurement.p(1), measurement.p(2), measurement.attitude.w(), measurement.attitude.x(), measurement.attitude.y(), measurement.attitude.z());
}

void simuQSLSForceFeedbackCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg, common::NokovWithForce &measurement)
{
    // 从消息中提取位置和姿态
    measurement.fc = measurement.attitude.toRotationMatrix()*Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z);
    measurement.time = msg->header.stamp.toSec();
    // ROS_INFO("cable force received: fc = %+.5f %+.5f %+.5f, time: %+.5f", measurement.fc(0), measurement.fc(1), measurement.fc(2), measurement.time);
    measurement.updated = true;
}

void controlInputCallback(const test_controller::UAVCommand::ConstPtr& msg, common::QuadrotorControlInput &control_input)
{
    control_input.thrust = msg->thrust;
    control_input.omega = Eigen::Vector3d(msg->omega.x, msg->omega.y, msg->omega.z);
    // ROS_INFO("Control input received: thrust = %f, omega = [%f, %f, %f]", control_input.thrust, control_input.omega(0), control_input.omega(1), control_input.omega(2));
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
    ros::param::get("~uav_id", uav_id);// 读取当前命名空间下的 uav_id
    ros::param::get("~simulation", simu);
    ros::param::get("~file_path", file_path);
    ros::param::get("~observer_rate", observer_rate);
    // 读取传入参数结束

    // 创建一个话题发布者，发布类型为 test_controller::UAVState
    ros::Publisher pub = nh.advertise<test_controller::UAVState>("observer_topic", 10);

    // 创建统一对象
    common::SystemParams params;
    common::QSLSState qsls_state;
    common::QuadrotorState state;
    common::NokovWithForce measurement;
    common::QuadrotorControlInput control_input;

    // 加载参数文件
    if (!params.loadFromFile(file_path))
    {
        ROS_ERROR("Failed to load parameters from file.");
        return -1;
    }

    // 初始化 NokovFilter 对象（作为 Observer 的派生类）
    observer::NokovFilter myObs(params, state, measurement, control_input, simu);
    observer::KalmanQSLSObserver qsls_obs(params, qsls_state, measurement, control_input);
    // 初始化 ObserverScheduler 对象（栈变量），先注册 Observer，再调用 switchObserver
    observer::ObserverScheduler scheduler;
    scheduler.registerObserver(&myObs);
    scheduler.registerObserver(&qsls_obs);
    scheduler.switchObserver(myObs); // 直接传入 Observer 对象


    if(!simu){
        ROS_INFO("Experiment mode");
        // 初始化 Publisher
        ros::Publisher observe_pub = nh.advertise<std_msgs::Float64MultiArray>(uav_id + "/state", 10);
        // 订阅动捕反馈话题
        std::string nokov_topic = "/vrpn_client_node/" + uav_id + "/pose";
        ros::Subscriber feedback_sub = nh.subscribe<geometry_msgs::PoseStamped>(nokov_topic, 10, std::bind(feedbackCallback, std::placeholders::_1, std::ref(myObs.measurement)));

        // 订阅控制输入 
        ros::Subscriber control_input_sub = nh.subscribe<test_controller::UAVCommand>(uav_id + "/control", 10, std::bind(controlInputCallback, std::placeholders::_1, std::ref(myObs.control_input)));
        
        // 订阅其它话题，使用 std::bind 和 std::ref 传入对象引用
        ros::Subscriber observer_sw_sub = nh.subscribe<std_msgs::Int32>(uav_id + "/observer_sw", 10,
        std::bind(observerSWCallback, std::placeholders::_1, std::ref(scheduler)));

        ros::Rate Rate(observer_rate);
        while (ros::ok())
        {
            ros::spinOnce();
            // 运行观测器

            scheduler.run();
            // 发送状态反馈
            
            std_msgs::Float64MultiArray state_msg;
            std::vector<Eigen::VectorXd> vecs = {state.p, state.vi, state.q};
            state_msg = common::vectorXdToFloat64MultiArray(vecs);
            observe_pub.publish(state_msg);
            Rate.sleep();
        }
    }
    else{
        ROS_INFO("Simulation mode");
        // 初始化控制输入 Publisher
        ros::Publisher observe_pub = nh.advertise<std_msgs::Float64MultiArray>(uav_id + "/quadrotor_state", 10);
        ros::Publisher qsls_observe_pub = nh.advertise<test_controller::QSLSState>(uav_id + "/qsls_state", 10);
        // 订阅反馈话题,仅从仿真环境中获取无人机的位置与姿态
        std::string feedback_topic = uav_id + "/quadrotor_feedback";
        ros::Subscriber quadrotor_feedback_sub = nh.subscribe<test_controller::UAVState>(feedback_topic, 10, std::bind(simuFeedbackCallback, std::placeholders::_1, std::ref(myObs.measurement)));

        ros::Subscriber qsls_feedback_sub = nh.subscribe<test_controller::UAVState>(feedback_topic, 10, std::bind(simuQSLSpQFeedbackCallback, std::placeholders::_1, std::ref(qsls_obs.measurement)));

        // 订阅控制输入 
        ros::Subscriber control_input_sub = nh.subscribe<test_controller::UAVCommand>(uav_id + "/control", 10, std::bind(controlInputCallback, std::placeholders::_1, std::ref(myObs.control_input)));

        ros::Subscriber qsls_control_input_sub = nh.subscribe<test_controller::UAVCommand>(uav_id + "/control", 10, std::bind(controlInputCallback, std::placeholders::_1, std::ref(qsls_obs.control_input)));
        

        ros::Subscriber cable_force_sub = nh.subscribe<geometry_msgs::Vector3Stamped>(uav_id + "/cable_force", 10, std::bind(simuQSLSForceFeedbackCallback, std::placeholders::_1, std::ref(qsls_obs.measurement)));
        // 订阅其它话题，使用 std::bind 和 std::ref 传入对象引用
        ros::Subscriber observer_sw_sub = nh.subscribe<std_msgs::Int32>(uav_id + "/controller_sw", 10,
        std::bind(observerSWCallback, std::placeholders::_1, std::ref(scheduler)));
        ros::Rate Rate(observer_rate);
        while (ros::ok())
        {
            ros::spinOnce();
            // 运行观测器

            scheduler.run();
            // 发布无人机状态
            if(scheduler.current_mode==myObs.register_id){
            std_msgs::Float64MultiArray state_msg;
            std::vector<Eigen::VectorXd> vecs = {myObs.state.p, myObs.state.vi, state.q};
            // ROS_INFO("State updated: pos = [%f, %f, %f]", state.p(0), state.p(1), state.p(2));
            state_msg = common::vectorXdToFloat64MultiArray(vecs);
            observe_pub.publish(state_msg);
            }
            // 发布 QSLS 状态
            if (scheduler.current_mode == qsls_obs.register_id)
            {            
            test_controller::QSLSState qsls_state_msg;
            qsls_state_msg.pQ.x = qsls_obs.state.pQ(0);
            qsls_state_msg.pQ.y = qsls_obs.state.pQ(1);
            qsls_state_msg.pQ.z = qsls_obs.state.pQ(2);
            qsls_state_msg.vQ.x = qsls_obs.state.vQ(0);
            qsls_state_msg.vQ.y = qsls_obs.state.vQ(1);
            qsls_state_msg.vQ.z = qsls_obs.state.vQ(2);
            qsls_state_msg.pL.x = qsls_obs.state.pL(0);
            qsls_state_msg.pL.y = qsls_obs.state.pL(1);
            qsls_state_msg.pL.z = qsls_obs.state.pL(2);
            qsls_state_msg.vL.x = qsls_obs.state.vL(0);
            qsls_state_msg.vL.y = qsls_obs.state.vL(1);
            qsls_state_msg.vL.z = qsls_obs.state.vL(2);
            qsls_state_msg.q.x = qsls_obs.state.q(0);
            qsls_state_msg.q.y = qsls_obs.state.q(1);
            qsls_state_msg.q.z = qsls_obs.state.q(2);
            qsls_state_msg.w.x = qsls_obs.state.w(0);
            qsls_state_msg.w.y = qsls_obs.state.w(1);
            qsls_state_msg.w.z = qsls_obs.state.w(2);
            qsls_state_msg.bQ.x = qsls_obs.state.bQ(0);
            qsls_state_msg.bQ.y = qsls_obs.state.bQ(1);
            qsls_state_msg.bQ.z = qsls_obs.state.bQ(2);
            qsls_state_msg.bL.x = qsls_obs.state.bL(0);
            qsls_state_msg.bL.y = qsls_obs.state.bL(1);
            qsls_state_msg.bL.z = qsls_obs.state.bL(2);
            qsls_state_msg.quat.w = qsls_obs.state.quat.w();
            qsls_state_msg.quat.x = qsls_obs.state.quat.x();
            qsls_state_msg.quat.y = qsls_obs.state.quat.y();
            qsls_state_msg.quat.z = qsls_obs.state.quat.z();
            qsls_observe_pub.publish(qsls_state_msg);
            }

            Rate.sleep();
        }

    }

    return 0;
}