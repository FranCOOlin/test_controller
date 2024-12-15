#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <test_controller/UAVState.h>
#include <eigen3/Eigen/Dense>
#include <deque>

using namespace Eigen;

// 全局变量用于存储历史值
Vector3d p_old(0, 0, 0);          // 上次的位置
Matrix3d R_old = Matrix3d::Identity(); // 上次的旋转矩阵
Vector3d vi_old(0, 0, 0);         // 上次的速度
Vector3d omega_old(0, 0, 0);      // 上次的角速度
int frameNo_old = 0;              // 上次的帧编号

// 全局变量用于存储滤波窗口
std::deque<Vector3d> position_window;
std::deque<Vector3d> velocity_window;

// 中值计算函数（适用于 Vector3d）
Vector3d calculateMedian(std::deque<Vector3d>& window) {
    if (window.size() < 5) return window.back();  // 如果窗口未满，返回最新值

    Vector3d median;
    for (int i = 0; i < 3; ++i) { // 对每个维度分别计算中值
        std::vector<double> values;
        for (const auto& vec : window) {
            values.push_back(vec[i]);
        }
        std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
        median[i] = values[values.size() / 2];
    }
    return median;
}

// 滑动窗口更新函数
template <typename T>
void updateWindow(std::deque<T>& window, const T& new_value) {
    if (window.size() >= 5) {
        window.pop_front();  // 移除最旧的值
    }
    window.push_back(new_value);  // 添加新值
}


// 四元数转旋转矩阵
Matrix3d q2R(const Quaterniond& quat) {
    return quat.toRotationMatrix();
}

// 计算当前状态
void calculateState(Vector3d& p, Vector3d& vi, Vector3d& vb, Matrix3d& R, Vector3d& omega,
                    const Vector3d& p_current, const Quaterniond& q_current, int frameNo, double framerate, bool zDown = true) {
    // 固定旋转矩阵 R1
    Matrix3d R1;
    R1 << 1, 0,  0,
          0, -1, 0,
          0,  0, -1;

    // 当前旋转矩阵
    if (zDown)
    {
        R = q2R(q_current);
        p = p_current;
    }
    else{
        R = R1 * q2R(q_current) * R1.transpose();
        p = R1 * p_current;
    }
    

    // 时间增量
    double delta_t = (frameNo - frameNo_old) / framerate;

    // 如果 delta_t 太小，返回上次的状态
    if (delta_t < std::numeric_limits<double>::epsilon()) {
        p = p_old;
        vi = vi_old;
        vb = R.transpose() * vi_old;
        omega = omega_old;
        ROS_WARN("Delta time too small, returning old state");
        return;
    }
    // 计算惯性系速度
    vi = (p_current - p_old) / delta_t;

    // 计算旋转矩阵导数 R_dot = (R - R_old) / delta_t
    Matrix3d skew_omega = R.transpose() * (R - R_old) / delta_t;

    // 提取角速度 omega
    omega(0) = -skew_omega(1, 2);
    omega(1) = skew_omega(0, 2);
    omega(2) = -skew_omega(0, 1);

    // 计算体坐标系速度
    vb = R.transpose() * vi;

    // 更新全局历史值
    p_old = p_current;
    R_old = R;
    vi_old = vi;
    omega_old = omega;
    frameNo_old = frameNo;
}

// ROS 回调函数
void feedbackCallback(const test_controller::UAVState::ConstPtr& feedback_msg, ros::Publisher& state_pub, int& frame_counter, double framerate) {
    // 当前位置
    Vector3d p_current(feedback_msg->position.x, feedback_msg->position.y, feedback_msg->position.z);

    // 当前姿态（四元数）
    Quaterniond q_current(feedback_msg->attitude.w, feedback_msg->attitude.x, feedback_msg->attitude.y, feedback_msg->attitude.z);

    // 当前帧编号
    int frameNo = frame_counter++;

    // 计算无人机状态
    Vector3d p, vi, vb, omega;
    Matrix3d R;
    calculateState(p, vi, vb, R, omega, p_current, q_current, frameNo, framerate);
    // 更新滑动窗口
    updateWindow(position_window, p);
    updateWindow(velocity_window, vi);
    // 计算中值
    Vector3d position_median = calculateMedian(position_window);
    Vector3d velocity_median = calculateMedian(velocity_window);
    // 发布新状态
    test_controller::UAVState state_msg;
    state_msg.position.x = position_median(0);
    state_msg.position.y = position_median(1);
    state_msg.position.z = position_median(2);

    state_msg.velocity.x = velocity_median(0);
    state_msg.velocity.y = velocity_median(1);
    state_msg.velocity.z = velocity_median(2);

    state_msg.attitude.w = q_current.w();
    state_msg.attitude.x = q_current.x();
    state_msg.attitude.y = q_current.y();
    state_msg.attitude.z = q_current.z();

    state_pub.publish(state_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_state_processor");
    ros::NodeHandle nh;

    // 参数：帧率
    double framerate = 300.0;
    nh.param("framerate", framerate, 200.0);

    // 帧计数器
    int frame_counter = 0;

    // 发布器和订阅器
    ros::Publisher state_pub = nh.advertise<test_controller::UAVState>("/uav/state", 10);
    ros::Subscriber feedback_sub = nh.subscribe<test_controller::UAVState>("/uav/feedback", 10,
        boost::bind(feedbackCallback, _1, boost::ref(state_pub), boost::ref(frame_counter), framerate));

    ros::spin();
    return 0;
}