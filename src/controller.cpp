#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <test_controller/UAVState.h>
#include <test_controller/UAVCommand.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace Eigen;
ros::Publisher command_pub;
ros::Subscriber state_sub;
// 无人机参数
double mq = 0.33;
double g = 9.81;
double kp = 7.65, kv = 3.6, kr = 120.0, hr = 30.0;
VectorXd p1 = (VectorXd(3) << -1.11674940231383e-06, 0.00216553259709083, -0.0276877882890898).finished();
VectorXd p2 = (VectorXd(3) << 0.000413683067484175, 0.932875787982657, 0).finished();
VectorXd p3 = (VectorXd(3) << 0.000343767264533347,1.05504806357145,0).finished();
VectorXd p4 = (VectorXd(3) << 0.000770252131194202,0.866138866957217,0).finished();
bool use_polyval = true;

// 限制函数
double clamp(double x, double min, double max) {
    return x < min ? min : (x > max ? max : x);
}

// 多项式计算
double polyval(double x, const VectorXd& p) {

    // 多项式计算 y = polyval(p1, x)
    double y = 0.0;
    int degree = p.size() - 1;
    for (int i = 0; i < p.size(); ++i) {
        y += p[i] * std::pow(x, degree - i);
    }

    return y;
}

// 符号函数
template <typename T>
int sign(T value) {
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}


// 交叉乘积矩阵
Matrix3d S(const Vector3d& vec) {
    Matrix3d mat;
    mat <<  0,       -vec(2),  vec(1),
            vec(2),  0,       -vec(0),
           -vec(1),  vec(0),   0;
    return mat;
}

// 投影矩阵
Matrix3d PI(const Vector3d& vec) {
    return Matrix3d::Identity() - vec * vec.transpose();
}

// 控制算法
void Controller(double& T, Vector3d& omega, const Vector3d& pd, const Vector3d& dpd, const Vector3d& d2pd,
                const Vector3d& d3pd, const Vector3d& p, const Vector3d& v, const Matrix3d& R, double yaw,
                double mq, double g, const Vector3d& e3, double kp, double kv, double kr, double hr) {

    // 计算位置和速度误差
    double b =0.5;
    Vector3d zp = p - pd;
    double nzp = zp.norm();
    Vector3d zv = v - dpd;
    double nzv = zv.norm();
    Vector3d u = -kp * zp - kv * zv;
    Vector3d Fd = mq * (u - g * e3 + d2pd);
    // ROS_INFO("Fd: %f %f %f", Fd(0), Fd(1), Fd(2));
    double Td = Fd.norm();
    Vector3d r3d = -Fd / Td;
    Vector3d r3 = R*e3;
    T = Td*r3d.dot(r3);
    // ROS_INFO("r3d: %f %f %f", r3d(0), r3d(1), r3d(2));
    // ROS_INFO("r3: %f %f %f", r3(0), r3(1), r3(2));
    Vector3d F = -T*R*e3;
    Vector3d dzv = F/mq + g*e3 - d2pd;
    Vector3d dFd = mq*(-kp*zv - kv*dzv + d3pd);
    Vector3d dr3d = S(r3d)*S(r3d)*dFd/Fd.norm();
    Vector3d zr = r3 - r3d;
    omega = - S(e3)*S(e3)*(R.transpose()*S(r3d)*dr3d + kr/hr*S(e3)*R.transpose()*r3d + Td/(mq*hr)*S(e3)*R.transpose()*(b*zp+zv)) - 1.5*abs(yaw)*e3;


    
}

void controllCallback(const test_controller::UAVState::ConstPtr& state_msg) {
            // 解析输入状态
            Vector3d position(state_msg->position.x, state_msg->position.y, state_msg->position.z);
            Vector3d velocity(state_msg->velocity.x, state_msg->velocity.y, state_msg->velocity.z);
            Quaterniond attitude(
                state_msg->attitude.w,
                state_msg->attitude.x,
                state_msg->attitude.y,
                state_msg->attitude.z
            );
            Matrix3d R = attitude.toRotationMatrix();
            Vector3d e3(0, 0, 1);


            // 期望状态
            Vector3d pd(2, 2, 1), dpd(0, 0, 0), d2pd(0, 0, 0), d3pd(0, 0, 0);

            // 控制量
            double T;
            Vector3d omega;

            // 调用控制算法
            Controller(T, omega, pd, dpd, d2pd, d3pd, position, velocity, R, 0.0,
                       mq, g, e3, kp, kv, kr, hr);

            // 发布控制指令
            test_controller::UAVCommand command_msg;
            if(use_polyval) {
                command_msg.thrust = T;
                // command_msg.thrust = clamp(polyval(T,p1),0,1);
                command_msg.omega.x = -clamp(sign(omega(0))*polyval(abs(omega(0)*180/3.141592654), p2),-360,360);
                command_msg.omega.y = -clamp(sign(omega(1))*polyval(abs(omega(1)*180/3.141592654), p3),-360,360);
                command_msg.omega.z = clamp(0*sign(omega(2))*polyval(abs(omega(2)*180/3.141592654), p4),-180,180);
                command_pub.publish(command_msg);
            }
            else {
                command_msg.thrust = T;
                command_msg.omega.x = omega(0);
                command_msg.omega.y = omega(1);
                command_msg.omega.z = omega(2);
                command_pub.publish(command_msg);
            }
            ROS_INFO("Published command: thrust = %f, omega = [%f, %f, %f]", T, command_msg.omega.x, command_msg.omega.y, command_msg.omega.z);

        }

// ROS 主程序
int main(int argc, char** argv) {
    ros::init(argc, argv, "test_controllerler");
    ros::NodeHandle nh;

    // 发布和订阅
    command_pub = nh.advertise<test_controller::UAVCommand>("/uav/control", 10);
    state_sub = nh.subscribe<test_controller::UAVState>("/uav/state", 10,controllCallback);
    
    ros::spin();
    return 0;
}
