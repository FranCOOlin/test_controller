#include <ros/ros.h>
#include <test_controller/Float64MultiArrayWithHeader.h>
#include <cmath>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;

    // 创建发布者
    ros::Publisher pub = nh.advertise<test_controller::Float64MultiArrayWithHeader>("sin_topic", 10);

    // 设置发布频率
    ros::Rate loop_rate(200);  // 10 Hz

    while (ros::ok()) {
        // 当前时间 t
        double t = ros::Time::now().toSec();  // 获取当前时间戳（秒）

        // 计算 sin(t)，并填充 12 维的向量
        test_controller::Float64MultiArrayWithHeader msg;
        msg.state.data.resize(12);  // Resize to 12 dimensions
        for (int i = 0; i < 12; ++i) {
            msg.state.data[i] = std::sin(t);  // 对每个维度，sin(t) 是相同的
        }

        // 加入时间戳
        msg.header.stamp = ros::Time::now();
        // 发布消息
        pub.publish(msg);

        // 打印调试信息
        // ROS_INFO_STREAM("Publishing sin(t): " << msg.data);

        // 睡眠直到下次循环
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}