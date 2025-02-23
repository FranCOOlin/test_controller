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
#include <deque>

// 引入各模块头文件
#include "test_controller/common/params.h"          // Params
#include "test_controller/common/state.h"           // State
#include "test_controller/observer/observer.h"  // Observer 抽象基类
#include "test_controller/observer/observer_scheduler.h"  // ControllerScheduler 类
#include "test_controller/custom/myobserver.h"   // MyObserver 派生类
#include "test_controller/custom/myparams.h"       // MyParams 派生类
#include "test_controller/custom/mystate.h"        // MyState 派生类
#include "test_controller/custom/mytrajectory.h"      // MyTrajectory 派生类
#include "test_controller/common/convert.h"
// ROS 消息
#include <test_controller/UAVState.h>
#include <test_controller/UAVCommand.h>




void observerSWCallback(const std_msgs::Int32::ConstPtr& msg, observer::ObserverScheduler &scheduler)
{
    int observer_id = msg->data;  // 获取消息中的控制编号
    ROS_INFO("Switching to observer with ID: %d", observer_id);

    // 通过给定的 observer_id 查找并切换观察器
    if (observer_id >= 0) {
        // 假设 ObserverScheduler 通过 ID 找到正确的 observer
        // 切换当前观察器
        scheduler.switchObserver(*scheduler.observers[observer_id]);
    } else {
        ROS_WARN("Invalid observer ID received: %d", observer_id);
    }
}


void feedbackCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, common::MyMeasurement &measurement)
{
    measurement.p = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    measurement.attitude = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}
void simuFeedbackCallback(const test_controller::UAVState::ConstPtr& msg, common::MyMeasurement &measurement)
{
    // 从消息中提取位置和姿态
    measurement.p = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    measurement.attitude = Eigen::Quaterniond(msg->attitude.w, msg->attitude.x, msg->attitude.y, msg->attitude.z);
    // ROS_INFO("Feedback received: p = %f %f %f, q = %f %f %f %f", measurement.p(0), measurement.p(1), measurement.p(2), measurement.attitude.w(), measurement.attitude.x(), measurement.attitude.y(), measurement.attitude.z());
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
    common::MyParams params;
    common::MyState state;
    common::MyMeasurement measurement;
    common::MyControlInput control_input;

    // 加载参数文件
    if (!params.loadFromFile(file_path))
    {
        ROS_ERROR("Failed to load parameters from file.");
        return -1;
    }

    // 初始化 MyObserver 对象（作为 Observer 的派生类）
    observer::MyObserver myObs(params, state, measurement, control_input, simu);

    // 初始化 ObserverScheduler 对象（栈变量），先注册 Observer，再调用 switchObserver
    observer::ObserverScheduler scheduler;
    scheduler.registerObserver(&myObs);
    scheduler.switchObserver(myObs); // 直接传入 Observer 对象


    if(!simu){
        ROS_INFO("Experiment mode");
        // 初始化 Publisher
        ros::Publisher observe_pub = nh.advertise<std_msgs::Float64MultiArray>(uav_id + "/state", 10);
        // 订阅动捕反馈话题
        std::string nokov_topic = "/vrpn_client_node/" + uav_id + "/pose";
        ros::Subscriber feedback_sub = nh.subscribe<geometry_msgs::PoseStamped>(nokov_topic, 10, std::bind(feedbackCallback, std::placeholders::_1, std::ref(myObs.measurement)));
        
        // 订阅其它话题，使用 std::bind 和 std::ref 传入对象引用
        ros::Subscriber observer_sw_sub = nh.subscribe<std_msgs::Int32>(uav_id + "observer_sw", 10,
        std::bind(observerSWCallback, std::placeholders::_1, std::ref(scheduler)));


        // 用于存储滤波窗口
        std::deque<Eigen::Vector3d> position_window;
        std::deque<Eigen::Vector3d> velocity_window;
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
        ros::Publisher observe_pub = nh.advertise<std_msgs::Float64MultiArray>(uav_id + "/state", 10);
        // 订阅反馈话题
        std::string feedback_topic = uav_id + "/feedback";
        ros::Subscriber feedback_sub = nh.subscribe<test_controller::UAVState>(feedback_topic, 10, std::bind(simuFeedbackCallback, std::placeholders::_1, std::ref(myObs.measurement)));
        
        // 订阅其它话题，使用 std::bind 和 std::ref 传入对象引用
        ros::Subscriber observer_sw_sub = nh.subscribe<std_msgs::Int32>(uav_id + "observer_sw", 10,
        std::bind(observerSWCallback, std::placeholders::_1, std::ref(scheduler)));

        Eigen::Vector3d p_old(0, 0, 0);          // 上次的位置
        Eigen::Matrix3d R_old = Eigen::Matrix3d::Identity(); // 上次的旋转矩阵
        Eigen::Vector3d vi_old(0, 0, 0);         // 上次的速度
        Eigen::Vector3d omega_old(0, 0, 0);      // 上次的角速度
        int frameNo_old = 0;              // 上次的帧编号
        // 用于存储滤波窗口
        std::deque<Eigen::Vector3d> position_window;
        std::deque<Eigen::Vector3d> velocity_window;
        ros::Rate Rate(observer_rate);
        while (ros::ok())
        {
            ros::spinOnce();
            // 运行观测器

            scheduler.run();
            // 发送状态反馈
            
            std_msgs::Float64MultiArray state_msg;
            std::vector<Eigen::VectorXd> vecs = {state.p, state.vi, state.q};
            // ROS_INFO("State updated: pos = [%f, %f, %f]", state.p(0), state.p(1), state.p(2));
            state_msg = common::vectorXdToFloat64MultiArray(vecs);
            observe_pub.publish(state_msg);
            Rate.sleep();
        }

    }

    return 0;
}