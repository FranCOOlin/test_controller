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
        KalmanQSLSObserver(common::SystemParams &_params, common::QSLSState &_state, common::NokovWithForce &_measurement, common::QuadrotorControlInput &_control_input)
            : params(_params), state(_state), measurement(_measurement), control_input(_control_input), integrator(std::bind(&KalmanQSLSObserver::f, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::ref(params), std::ref(control_input), std::ref(measurement)), 0.01) {last_update_time = -1.0;}

        void f(const state_type &x, state_type &dxdt, double t, common::SystemParams &params, const common::QuadrotorControlInput &control_input, const common::NokovWithForce &measurement)
        {   
            Eigen::VectorXd hat_eta(18);
            Eigen::VectorXd dhat_eta(18);
            hat_eta = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
            Eigen::Matrix3d R = measurement.attitude.toRotationMatrix();
            Eigen::Vector3d F = -control_input.thrust * R * Eigen::Vector3d(0, 0, 1);
            Eigen::Vector3d G = params.g * Eigen::Vector3d(0, 0, 1);
            Eigen::VectorXd u = Eigen::VectorXd(measurement.fc.size() + F.size() + G.size());
            u << measurement.fc, F, G;
            Eigen::Vector3d pL_pQ = params.QSLS_l * (-measurement.fc/measurement.fc.norm());
            Eigen::VectorXd y = Eigen::VectorXd(measurement.p.size() + measurement.fc.size());
            y << measurement.p, pL_pQ;
            dhat_eta = params.obs_A * hat_eta + params.obs_B * u + params.obs_K * (y - params.obs_C * hat_eta);
            for(size_t i = 0; i < x.size(); ++i)
            {
                dxdt[i] = dhat_eta(i);
            }
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
            if(last_update_time < 0)
            {
                if(measurement.updated){
                    last_update_time = t;
                }
                return;
            }
            
            if(t - last_update_time < 1e-6)
            {   
                return; // no data updated
            }
            // ROS_INFO("Observer update: t = %f", last_update_time);
            integrator.step(int_vec, 0.0, t - last_update_time);
            state.pQ = Eigen::Vector3d(int_vec.data());
            state.vQ = Eigen::Vector3d(int_vec.data() + 3);
            state.pL = Eigen::Vector3d(int_vec.data() + 6);
            state.vL = Eigen::Vector3d(int_vec.data() + 9);
            state.bQ = Eigen::Vector3d(int_vec.data() + 12);
            state.bL = Eigen::Vector3d(int_vec.data() + 15);
            state.q = (state.pL-state.pQ).normalized();
            state.w = (state.pL-state.pQ).cross(state.vL-state.vQ)/(state.pL-state.pQ).squaredNorm();
            state.quat = measurement.attitude;
            state.R = state.quat.toRotationMatrix();
            state.updated = true;
            measurement.updated = false;
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
