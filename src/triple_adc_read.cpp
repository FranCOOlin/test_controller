#include <ros/ros.h>
#include <std_msgs/String.h>

#include "test_controller/triple_adc/dev_gpio.h"
#include "test_controller/triple_adc/dev_hardware_spi.h"
#include "test_controller/triple_adc/dev_config.h"
#include "test_controller/triple_adc/ADS1263.h"

#define spi_device "/dev/spidev0.1"
// 全局发布者，用于在回调函数中发布消息
ros::Publisher pub;

// 订阅回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("收到消息: %s", msg->data.c_str());
    // 示例处理：简单地在接收到的消息前加上前缀，然后发布出去
    std_msgs::String out_msg;
    out_msg.data = "处理后: " + msg->data;
    pub.publish(out_msg);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "template_node");
    ros::NodeHandle nh;

    // 创建发布者，发布到 "output_topic" 话题
    pub = nh.advertise<std_msgs::String>("output_topic", 10);

    // 创建订阅者，订阅 "input_topic" 话题
    ros::Subscriber sub = nh.subscribe("input_topic", 10, chatterCallback);

    ros::Rate loop_rate(10);  // 10 Hz

    DEV_HARDWARE_SPI_begin(spi_device);
    DEV_GPIO_INIT(DEV_GPIO26, DEV_GPIO_OUTPUT,0);
    ADS1263 adc1(DEV_GPIO22, DEV_GPIO27);
    adc1.initADC1(ADS1263_20SPS);
    
    while (ros::ok())
    {
        ros::spinOnce();
        DEV_GPIO_Write(DEV_GPIO26, 1);
        adc1.softReset();
        loop_rate.sleep();
    }

    return 0;
}
