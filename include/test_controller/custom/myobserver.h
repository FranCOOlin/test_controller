#ifndef OBSERVER_MY_OBSERVER_H
#define OBSERVER_MY_OBSERVER_H

#include "test_controller/observer/observer.h"
#include "test_controller/common/integrator.hpp"
#include "test_controller/custom/myparams.h"
#include "test_controller/custom/mystate.h"
#include "test_controller/custom/mymeasurement.h"
#include "test_controller/custom/mycontrol_input.h"
#include <boost/numeric/odeint.hpp>
#include <eigen3/Eigen/Dense>
#include <deque>

namespace observer {

using SystemDynamics = std::function<void(const state_type&, state_type&, double)>;

class MyObserver : public Observer {
public:
    common::MyParams &params;
    common::MyMeasurement &measurement;
    common::MyState &state;
    common::MyControlInput &control_input;
    common::Integrator<boost::numeric::odeint::runge_kutta_fehlberg78<state_type>> integrator;
    Eigen::Vector3d p_old;          // 上次的位置
    Eigen::Matrix3d R_old; // 上次的旋转矩阵
    Eigen::Vector3d vi_old;         // 上次的速度
    Eigen::Vector3d omega_old;      // 上次的角速度
    std::deque<Eigen::Vector3d> position_window;
    std::deque<Eigen::Vector3d> velocity_window;
    bool simu;
    // 使用 Boost ODEint 积分器类型

    // 构造函数：接收 Params、State 和 Measurement 的引用
    MyObserver(common::MyParams &_params, common::MyState &_state, common::MyMeasurement &_measurement, common::MyControlInput &_control_input, bool _simu)
        : params(_params), state(_state), measurement(_measurement),control_input(_control_input), integrator(std::bind(&MyObserver::f, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::ref(params), std::ref(control_input)), 0.01),simu(_simu), p_old(Eigen::Vector3d::Zero()), R_old(Eigen::Matrix3d::Identity()), vi_old(Eigen::Vector3d::Zero()), omega_old(Eigen::Vector3d::Zero()){}

    // 中值计算函数（适用于 Vector3d）
    Eigen::Vector3d calculateMedian(std::deque<Eigen::Vector3d>& window) {
        // if (window.size() < 5) return window.back();  // 如果窗口未满，返回最新值
        if (window.size() < 5) return Eigen::Vector3d::Zero();  // 如果窗口未满，返回零向量

        Eigen::Vector3d median;
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
    Eigen::Matrix3d q2R(const Eigen::Quaterniond& quat) {
        return quat.toRotationMatrix();
    }

    // 计算当前状态
    void calculateState(Eigen::Vector3d& p, Eigen::Vector3d& vi, Eigen::Vector3d& vb, Eigen::Matrix3d& R, Eigen::Vector3d& omega,
        Eigen::Vector3d& p_current, Eigen::Quaterniond& q_current, Eigen::Vector3d& p_old, Eigen::Matrix3d R_old, Eigen::Vector3d vi_old, Eigen::Vector3d omega_old, double last_time, bool zDown = false) {
        // 固定旋转矩阵 R1
        Eigen::Matrix3d R1;
        R1 << 1, 0,  0,
            0, -1, 0,
            0,  0, -1;

        // 当前旋转矩阵
        if (zDown||simu)
        {
            R = q2R(q_current);
            p = p_current;
        }
        else{
            R = R1 * q2R(q_current) * R1.transpose();
            p = R1 * p_current;
            p_current = p;
            Eigen::Quaterniond q(R);
            q_current = q;
        }
        

        // 时间增量
        double delta_t = ros::Time::now().toSec() - last_time;

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
        Eigen::Matrix3d skew_omega = R.transpose() * (R - R_old) / delta_t;

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
    }


    void f(const state_type &x, state_type &dxdt, double t, common::MyParams &params,const common::MyControlInput &control_input) {
        // 通过状态方程计算 dx/dt
        dxdt = x;
        // your code here
    }
    // 实现具体的 update() 函数：估计系统状态并更新 state
    void update() override {

        //从state转换为std::vector<double>
        // Eigen::VectorXd eigen_vec(state.p.size()+state.q.size());
        // eigen_vec << state.p, state.q;
        // std::vector<double> int_vec(eigen_vec.data(), eigen_vec.data() + eigen_vec.size());

        Eigen::Vector3d p, vi, vb, omega;
        Eigen::Matrix3d R;
        calculateState(p, vi, vb, R, omega, measurement.p,measurement.attitude, p_old, R_old, vi_old, omega_old, last_update_time);
            // 更新滑动窗口
        updateWindow(position_window, p);
        updateWindow(velocity_window, vi);
        // 计算中值
        Eigen::Vector3d position_median = calculateMedian(position_window);
        Eigen::Vector3d velocity_median = calculateMedian(velocity_window);
    // 发布新状态
        // 通过积分器对状态进行计算
        double t = ros::Time::now().toSec();
        // integrator.integrate(int_vec, 0.0, t-last_update_time);
        state.updated = true;
        last_update_time = t;

        state.p = position_median;
        state.vi = velocity_median;
        state.vb = vb;
        state.q = Eigen::Vector4d(measurement.attitude.w(), measurement.attitude.x(), measurement.attitude.y(), measurement.attitude.z());
        state.R = R;
        state.omega = omega;
        state.euler = R.eulerAngles(2, 1, 0);
        // ROS_INFO("Observer updated: vel = [%f, %f, %f]", state.vi(0), state.vi(1), state.vi(2));
        ROS_INFO("Observer updated: pos = [%f, %f, %f]", state.p(0), state.p(1), state.p(2));
        // 从std::vector<double>转换为state
        // state.p = Eigen::VectorXd::Map(int_vec.data(), 3);
        // state.q = Eigen::VectorXd::Map(int_vec.data()+3, 4);
    }

    // 可选的初始化函数
    void initialize() override {
        // 初始化过程（如果需要）
        
        ROS_INFO("Observer initialized with integrator.");
    }

};

} // namespace observer

#endif // OBSERVER_MY_OBSERVER_H
