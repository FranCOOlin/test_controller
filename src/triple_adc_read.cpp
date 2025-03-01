#include <ros/ros.h>
#include <std_msgs/String.h>
#include <test_controller/Float64MultiArrayWithHeader.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "test_controller/triple_adc/dev_gpio.h"
#include "test_controller/triple_adc/dev_hardware_spi.h"
#include "test_controller/triple_adc/dev_config.h"
#include "test_controller/triple_adc/ADS1263.h"

#define spi_device "/dev/spidev0.1"
// 全局发布者，用于在回调函数中发布消息
ros::Publisher pub;


int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "template_node");
    ros::NodeHandle nh;


      // 读取传入参数开始
    std::string uav_id = "";
    std::string file_path;
    bool simu;
    int controller_rate;
    ros::param::get("~uav_id", uav_id);
    ros::param::get("~simulation", simu);
    ros::param::get("~file_path", file_path);
    ros::param::get("~controller_rate", controller_rate);
    ROS_INFO("Simulation: %s", simu ? "true" : "false");
    // if (uav_id.empty()) {
    //     ROS_ERROR("uav_id not set");
    //     return -1;
    // }

    // 创建发布者，发布到 "output_topic" 话题
    // pub = nh.advertise<test_controller::Float64MultiArrayWithHeader>(uav_id+"/triple_adc_value", 10);
    pub = nh.advertise<geometry_msgs::Vector3Stamped>(uav_id+"/triple_adc_value", 10);

    
    ros::Rate loop_rate(100);  // 10 Hz

    DEV_HARDWARE_SPI_begin(spi_device);
    int START_PIN = DEV_GPIO26;
    DEV_GPIO_INIT(START_PIN, DEV_GPIO_OUTPUT,0);
    ADS1263 adc1(DEV_GPIO22, DEV_GPIO27,1.25);
    ADS1263 adc2(DEV_GPIO23, DEV_GPIO28,1.25);
    ADS1263 adc3(DEV_GPIO24, DEV_GPIO29,1.25);


    geometry_msgs::Vector3Stamped msg;

    
    while (ros::ok())
    {
        ros::spinOnce();
        DEV_GPIO_Write(START_PIN, DEV_GPIO_HIGH);
        adc1.waitDRDY();
        double value1 = adc1.readADC1Data();
        double value2 = adc2.readADC1Data();
        double value3 = adc3.readADC1Data();
        ROS_INFO("ADC1 Value: %+5f, ADC2 Value: %+5f, ADC3 Value: %+5f", value1, value2, value3);
        msg.header.stamp = ros::Time::now();
        msg.vector.x=value1;
        msg.vector.y=value2;
        msg.vector.z=value3;
        // msg.state.data.push_back(value1);
        // msg.state.data.push_back(value2);
        // msg.state.data.push_back(value3);
        pub.publish(msg);
        DEV_GPIO_Write(START_PIN, DEV_GPIO_LOW);
        loop_rate.sleep();
    }

    return 0;
}
