#ifndef KALMAN_QSLS_OBSERVER_H
#define KALMAN_QSLS_OBSERVER_H

#include "test_controller/observer/observer.h"
#include "test_controller/common/integrator.hpp"
#include "test_controller/custom/system_params.h"
#include "test_controller/custom/qsls_state.h"
#include "test_controller/custom/nokov_with_force.h"
#include "test_controller/custom/quadrotor_control_input.h"
#include <boost/numeric/odeint.hpp>
#include <eigen3/Eigen/Dense>
#include <deque>

namespace observer
{

    using SystemDynamics = std::function<void(const state_type &, state_type &, double)>;

    class KalmanQSLSObserver : public Observer
    {
    public:
        common::SystemParams &params;
        common::NokovWithForce &measurement;
        common::QSLSState &state;
        common::QuadrotorControlInput &control_input;
        common::Integrator<boost::numeric::odeint::runge_kutta_fehlberg78<state_type>> integrator; // 使用 Boost ODEint 积分器类型

        // 构造函数：接收 Params、State 和 Measurement 的引用
        KalmanQSLSObserver(common::SystemParams &_params, common::QSLSState &_state, common::NokovWithForce &_measurement, common::QuadrotorControlInput &_control_input, bool _simu)
            : params(_params), state(_state), measurement(_measurement), control_input(_control_input), integrator(std::bind(&KalmanQSLSObserver::f, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::ref(params), std::ref(control_input)), 0.01) {}

        void f(const state_type &x, state_type &dxdt, double t, common::SystemParams &params, const common::QuadrotorControlInput &control_input)
        {
            // 通过状态方程计算 dx/dt
            dxdt = x;
        }
        // 实现具体的 update() 函数：估计系统状态并更新 state
        void update() override
        {
            // 从state转换为std::vector<double>
            //  Eigen::VectorXd eigen_vec(state.pQ.size()+state.vQ.size()+state.pL.size()+state.vL.size()+state.bQ.size()+state.bL.size());
            Eigen::VectorXd eigen_vec(18);
            eigen_vec << state.pQ, state.vQ, state.pL, state.vL, state.bQ, state.bL;
            std::vector<double> int_vec(eigen_vec.data(), eigen_vec.data() + eigen_vec.size());

            double t = measurement.time;
            integrator.integrate(int_vec, 0.0, t - last_update_time);
            state.updated = true;
            last_update_time = t;
        }

        // 可选的初始化函数
        void initialize() override
        {
            // 初始化过程（如果需要）

            ROS_INFO("Observer initialized with integrator.");
        }
    };

} // namespace observer

#endif // OBSERVER_MY_OBSERVER_H
