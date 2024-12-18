#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <test_controller/UAVState.h>
#include <test_controller/UAVCommand.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <string>

using namespace Eigen;
ros::Publisher local_rate_pub;
ros::Publisher local_thrust_pub;
ros::Subscriber status_sub;
ros::Subscriber state_sub;
ros::Time last_req;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::State current_state;
// 无人机参数
double mq = 0.316;
double g = 9.81;
double kp = 9.75, kv = 3.6, kr = 120.0, hr = 30.0;
Vector3d p1(-1.11674940231383e-06, 0.00216553259709083, -0.0276877882890898);
Vector3d p2(0.000413683067484175, 0.932875787982657, 0);
Vector3d p3(0.000343767264533347,1.05504806357145,0);
Vector3d p4(0.000770252131194202,0.866138866957217,0);
bool use_polyval = true;
double start_time = 0;

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
    T = Td*r3d.dot(r3) - 0.25;
    // ROS_INFO("r3d: %f %f %f", r3d(0), r3d(1), r3d(2));
    Vector3d F = -T*R*e3;
    Vector3d dzv = F/mq + g*e3 - d2pd;
    Vector3d dFd = mq*(-kp*zv - kv*dzv + d3pd);
    Vector3d dr3d = S(r3d)*S(r3d)*dFd/Fd.norm();
    Vector3d zr = r3 - r3d;
    omega = - S(e3)*S(e3)*(R.transpose()*S(r3d)*dr3d + kr/hr*S(e3)*R.transpose()*r3d + Td/(mq*hr)*S(e3)*R.transpose()*(b*zp+zv)) - 0.1*(yaw-1.57)*e3;
    // ROS_INFO("yaw: %f", yaw);


    
}
void status_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
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
            //ROS_INFO("R: %f %f %f %f %f %f %f %f %f", R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2));
            Vector3d euler = attitude.toRotationMatrix().eulerAngles(2, 1, 0);
            Vector3d e3(0, 0, 1);
            double time = ros::Time::now().toSec();

            // 期望状态 
            double A=0.0,B=0.0,a=0.8,b=0.8;
            Vector3d pd(A*sin((time-start_time)*a),B*cos((time-start_time)*b), -0.8), dpd(A*a*cos((time-start_time)*a), -B*b*sin((time-start_time)*b), 0), d2pd(-A*a*a*sin((time-start_time)*a), -B*b*b*cos((time-start_time)*b), 0), d3pd(-A*a*a*a*cos((time-start_time)*a), B*b*b*b*sin((time-start_time)*b), 0);

            // 控制量
            double T;
            Vector3d omega;

            // 调用控制算法
            Controller(T, omega, pd, dpd, d2pd, d3pd, position, velocity, R, euler(0),
                       mq, g, e3, kp, kv, kr, hr);

            // 发布控制指令
            geometry_msgs::TwistStamped rate;
            mavros_msgs::Thrust thrust;
            if(use_polyval) {
                //thrust.thrust = T;
                thrust.thrust = clamp(polyval(T*1000/9.8,p1),0,1);
                //ROS_INFO("Thrust:%f",T);
                rate.twist.angular.y = clamp(sign(omega(0))*polyval(abs(omega(0)), p2),-3.14,3.14);
                rate.twist.angular.x = clamp(sign(omega(1))*polyval(abs(omega(1)), p3),-3.14,3.14);
                rate.twist.angular.z = clamp(0*sign(omega(2))*polyval(abs(omega(2)), p4),-3.14,3.14);
            }
            else {
                thrust.thrust = T;
                rate.twist.angular.x = omega(0);
                rate.twist.angular.y = omega(1);
                rate.twist.angular.z = omega(2);
            }
            local_rate_pub.publish(rate);
            local_thrust_pub.publish(thrust);
             // Check if we need to switch to OFFBOARD mode
            if ( (ros::Time::now() - last_req) > ros::Duration(5.0)) {
                if(current_state.mode != "OFFBOARD"){
                    ROS_INFO("Switching to OFFBOARD mode");  
                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        ROS_INFO("OFFBOARD enabled");
                    }
                    last_req = ros::Time::now();
                }
                
            } 
            //if(current_state.mode == "OFFBOARD"){
            if(current_state.mode == "OFFBOARD" && (ros::Time::now() - last_req) > ros::Duration(5.0)){
                // If not armed, try to arm
                    if(!current_state.armed){
                        ROS_INFO("Arming vehicle");
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                            ROS_INFO("Vehicle armed");
                        }
                        last_req = ros::Time::now();
                    }
                
            }

            //ROS_INFO("Published command: thrust = %f, omega = [%f, %f, %f]", T, command_msg.omega.x, command_msg.omega.y, command_msg.omega.z);

        }

// ROS 主程序
int main(int argc, char** argv) {
    ros::init(argc, argv, "test_controllerler");
    ros::NodeHandle nh;
    std::string uav_id;
    ros::param::get("~uav_id", uav_id);  // 读取当前命名空间下的 uav_id
    if (uav_id.empty()) {
        ROS_ERROR("uav_id not set");
        return -1;
    }
    // 发布和订阅
    local_rate_pub = nh.advertise<geometry_msgs::TwistStamped>(uav_id + "/mavros/setpoint_attitude/cmd_vel", 10);
    local_thrust_pub = nh.advertise<mavros_msgs::Thrust>(uav_id + "/mavros/setpoint_attitude/thrust", 10);
    state_sub = nh.subscribe<test_controller::UAVState>("/uav/state", 10,controllCallback);
    status_sub = nh.subscribe(uav_id + "/mavros/state", 10, status_cb);
    // Service clients
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_id + "/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_id + "/mavros/set_mode");

    // Wait for the services to be available
    ros::service::waitForService(uav_id + "/mavros/cmd/arming");
    ros::service::waitForService(uav_id + "/mavros/set_mode");

    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    last_req = ros::Time::now();
    start_time = ros::Time::now().toSec();
    
    ros::spin();
    return 0;
}
