#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <test_controller/UAVCommand.h>
#include <utility>  // std::ref

// Callback function for the first topic
void controlCallback(const test_controller::UAVCommand::ConstPtr& msg, test_controller::UAVCommand& control_input)
{
    control_input.thrust = msg->thrust;
    control_input.omega.x = msg->omega.x;
    control_input.omega.y = msg->omega.y;
    control_input.omega.z = msg->omega.z;
}

// Callback function for the second topic
void mycontrolCallback(const test_controller::UAVCommand::ConstPtr& msg, test_controller::UAVCommand& control_input)
{
    control_input.thrust = msg->thrust;
    control_input.omega.x = msg->omega.x;
    control_input.omega.y = msg->omega.y;
    control_input.omega.z = msg->omega.z;
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "check_controller");
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
    if (uav_id.empty()) {
      ROS_ERROR("uav_id not set");
      return -1;
    }
    // Subscribe to the first topic
    test_controller::UAVCommand control_input;
    test_controller::UAVCommand my_control_input;
    ros::Subscriber controller_sub = nh.subscribe<test_controller::UAVCommand>(uav_id+"/control", 10, std::bind(controlCallback, std::placeholders::_1, std::ref(control_input)));

    ros::Subscriber mycontroller_sub = nh.subscribe<test_controller::UAVCommand>(uav_id+"/mycontrol", 10, std::bind(mycontrolCallback, std::placeholders::_1, std::ref(my_control_input)));

    // Spin to keep the node running and processing callbacks
    ros::Rate rate(800);
    while (ros::ok())
    {   

        ROS_INFO("controller: %+.5f,%+.5f,%+.5f,%+.5f",control_input.thrust,control_input.omega.x,control_input.omega.y,control_input.omega.z);
        ROS_INFO("mycontroller: %+.5f,%+.5f,%+.5f,%+.5f",my_control_input.thrust,my_control_input.omega.x,my_control_input.omega.y,my_control_input.omega.z);
        ROS_INFO("difference: %+.5f,%+.5f,%+.5f,%+.5f",control_input.thrust-my_control_input.thrust,control_input.omega.x-my_control_input.omega.x,control_input.omega.y-my_control_input.omega.y,control_input.omega.z-my_control_input.omega.z);
        ros::spinOnce();
        rate.sleep();
    }
    

    ros::spin();

    return 0;
}