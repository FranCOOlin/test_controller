#include <ros/ros.h>
#include <test_controller/Float64MultiArrayWithHeader.h>  // 引入新定义的消息类型
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "test_controller/common/integrator.hpp"
#include <eigen3/Eigen/Dense>

using namespace boost::numeric::odeint;
using namespace Eigen;

// 定义状态类型
// typedef std::vector<double> state_type;

// 定义4个Eigen向量，用来存储计算的dx/dt的分量
Vector3d eigendx, eigendy, eigendz, eigendq;

// 定义右侧方程 dx/dt = sin(t) 的类
class SinODE {
public:
    // 重载函数调用运算符，作为ODE方程的右侧，需要计算dx/dt，此处使用eigen作为示例
    void operator()(const state_type &x, state_type &dxdt, const double t) {
        // 将x[0..2]对应的dx/dt分量设为sin(t)
        for (int i = 0; i < 3; i++) {
            eigendx(i) = std::sin(t);  // 对应x[0..2]分量
        }
        // 将x[3..5]对应的dx/dt分量设为sin(t)
        for (int i = 3; i < 6; i++) {
            eigendy(i - 3) = std::sin(t);  // 对应x[3..5]分量
        }
        // 将x[6..8]对应的dx/dt分量设为sin(t)
        for (int i = 6; i < 9; i++) {
            eigendz(i - 6) = std::sin(t);  // 对应x[6..8]分量
        }
        // 将x[9..11]对应的dx/dt分量设为sin(t)
        for (int i = 9; i < 12; i++) {
            eigendq(i - 9) = std::sin(t);  // 对应x[9..11]分量
        }

        // 创建12x12单位矩阵，模拟一种状态转换
        MatrixXd eye = MatrixXd::Identity(12, 12);
        VectorXd eigendX(12);
        
        // 将各个部分的dx/dt按顺序连接成一个12维的向量
        eigendX << eigendx, eigendy, eigendz, eigendq;
        
        // 通过单位矩阵与eigendX相乘（这里相当于对dx/dt的计算做变换）
        eigendX = eye * eigendX;
        
        // 将计算后的结果赋值给dxdt
        for (size_t i = 0; i < x.size(); ++i) {
            dxdt[i] = eigendX[i];  // 将计算的dx/dt传回
        }
    }
};

// 定义ROS发布者
ros::Publisher pub;

// 定义状态量向量x，初始值为12维的零向量
state_type x(12);

// 定义ROS时间变量
ros::Time prev_time;

// 创建SinODE对象，定义ODE方程的右侧
SinODE sin_ode;

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle nh;

    // 创建一个话题发布者，发布类型为test_controller::Float64MultiArrayWithHeader
    pub = nh.advertise<test_controller::Float64MultiArrayWithHeader>("integral_topic", 10);

    // 设置时间步长（每步时间），并设定更新频率
    const double dt = 1. / 800;
    ros::Rate loop_rate(1 / dt);  // 每秒钟800次更新

    // 初始化x为0
    x.assign(12, 0.0);

    // 创建一个积分器，使用Runge-Kutta-Fehlberg78算法
    common::Integrator<boost::numeric::odeint::runge_kutta_fehlberg78<state_type>> integrator(sin_ode, 0.01);

    while (ros::ok()) {
        // 获取当前时间
        ros::Time curr_time = ros::Time::now();
        double t = curr_time.toSec();  // 获取当前时间戳（秒）
        double dt = (curr_time - prev_time).toSec();  // 计算时间差（步长）

        // 初始化时（第一次循环），设置x的初始值
        if (prev_time.isZero()) {
            prev_time = curr_time;
            x.assign(12, -std::cos(t));  // 用cos(t)初始化状态
            continue;
        }

        // 调用积分器进行一步积分
        integrator.step(x, prev_time.toSec(), dt);

        // 创建消息并将积分后的结果存入msg
        test_controller::Float64MultiArrayWithHeader msg;
        state_type y(12);  // 用来存储状态更新后的值
        for (int i = 0; i < 12; ++i) {
            y[i] = -std::cos(t) - x[i];  // 计算误差并填充消息数据
        }

        // 将计算得到的状态数据填充到消息中
        msg.state.data = y;

        // 添加时间戳到消息头
        msg.header.stamp = ros::Time::now();

        // 发布消息
        pub.publish(msg);

        // 更新prev_time为当前时间
        prev_time = curr_time;

        // 等待下一次循环
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 进入ROS事件循环
    ros::spin();

    return 0;
}
