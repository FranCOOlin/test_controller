#ifndef CUSTOM_SYSTEM_PARAMS_H
#define CUSTOM_SYSTEM_PARAMS_H

#include "test_controller/common/integrator.hpp"
#include "test_controller/common/params.h"
#include <fstream>
#include <iostream>
#include <utility>
#include <functional>
#include <eigen3/Eigen/Dense>

namespace common
{
    class SystemParams : public Params
    {
    public:
        // Global param
        double g,max_thrust;
        // For quadrotor controller of ganyu
        double quadrotor_mq, quadrotor_kp, quadrotor_kv, quadrotor_kr, quadrotor_hr;

        // For QSLS saturated backstepping controller
        double QSLS_bar_mQ;
        double QSLS_bar_mL;
        double QSLS_l;
        double QSLS_k1;
        double QSLS_beta;
        double QSLS_ks1;
        double QSLS_k2;
        double QSLS_ks2;
        double QSLS_hq;
        double QSLS_kq;
        double QSLS_hw;
        double QSLS_kw;
        double QSLS_hr;
        double QSLS_kr;

        // For observer
        Eigen::Matrix3d obs_covQ_pQ;
        Eigen::Matrix3d obs_covQ_vQ;
        Eigen::Matrix3d obs_covQ_pL;
        Eigen::Matrix3d obs_covQ_vL;
        Eigen::Matrix3d obs_covQ_bQ;
        Eigen::Matrix3d obs_covQ_bL;
        Eigen::Matrix3d obs_covR_pQ;
        Eigen::Matrix3d obs_covR_q;
        Eigen::MatrixXd obs_covQ;
        Eigen::MatrixXd obs_covR;
        Eigen::MatrixXd obs_invcovR;
        Eigen::MatrixXd obs_P;
        Eigen::MatrixXd obs_A;
        Eigen::MatrixXd obs_C;
        Eigen::MatrixXd obs_B;
        Eigen::MatrixXd obs_K;

        // For signal generation
        bool use_polyval;
        Eigen::Vector3d p1, p2, p3, p4;

        virtual ~SystemParams() = default;

        // 从 ROS 参数服务器加载参数的实现
        virtual bool loadFromRos(ros::NodeHandle &nh) override
        {
            std::string uav_id;
            ros::param::get("~uav_id", uav_id); // 读取当前命名空间下的 uav_id
            if (uav_id.empty())
            {
                ROS_ERROR("uav_id not set");
            }
            if (!nh.getParam(uav_id + "/controller/quadrotor/kp", quadrotor_kp))
            {
                ROS_WARN("Failed to get parameter: /controller/quadrotor/kp");
            }
            if (!nh.getParam(uav_id + "/controller/quadrotor/kv", quadrotor_kv))
            {
                ROS_WARN("Failed to get parameter: /controller/quadrotor/kv");
            }
            if (!nh.getParam(uav_id + "/controller/quadrotor/kr", quadrotor_kr))
            {
                ROS_WARN("Failed to get parameter: /controller/quadrotor/kr");
            }
            if (!nh.getParam(uav_id + "/controller/quadrotor/hr", quadrotor_hr))
            {
                ROS_WARN("Failed to get parameter: /controller/quadrotor/hr");
            }
            if (!nh.getParam(uav_id + "/controller/quadrotor/mq", quadrotor_mq))
            {
                ROS_WARN("Failed to get parameter: /controller/quadrotor/mq");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/bar_mQ", QSLS_bar_mQ))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/bar_mQ");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/bar_mL", QSLS_bar_mL))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/bar_mL");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/l", QSLS_l))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/l");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/k1", QSLS_k1))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/k1");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/beta", QSLS_beta))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/beta");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/ks1", QSLS_ks1))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/ks1");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/k2", QSLS_k2))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/k2");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/ks2", QSLS_ks2))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/ks2");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/hq", QSLS_hq))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/hq");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/kq", QSLS_kq))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/kq");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/hw", QSLS_hw))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/hw");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/kw", QSLS_kw))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/kw");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/hr", QSLS_hr))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/hr");
            }
            if (!nh.getParam(uav_id + "/controller/QSLS/kr", QSLS_kr))
            {
                ROS_WARN("Failed to get parameter: /controller/QSLS/kr");
            }
            if (!nh.getParam(uav_id + "/controller/g", g))
            {
                ROS_WARN("Failed to get parameter: g");
            }
            if (!nh.getParam(uav_id + "/controller/use_polyval", use_polyval))
            {
                ROS_WARN("Failed to get parameter: controller/use_polyval");
            }
            std::vector<double> temp;
            if (!nh.getParam(uav_id + "/controller/p1", temp))
            {
                ROS_WARN("Failed to get parameter: controller/p1");
            }
            else
            {
                p1 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
            }
            if (!nh.getParam(uav_id + "controller/p2", temp))
            {
                ROS_WARN("Failed to get parameter: controller/p2");
            }
            else
            {
                p2 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
            }
            if (!nh.getParam(uav_id + "controller/p3", temp))
            {
                ROS_WARN("Failed to get parameter: controller/p3");
            }
            else
            {
                p3 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
            }
            if (!nh.getParam(uav_id + "controller/p4", temp))
            {
                ROS_WARN("Failed to get parameter: controller/p4");
            }
            else
            {
                p4 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
            }

            ROS_INFO("Loaded parameters from ROS parameter server.");
            return true;
        }

        // 从 JSON 文件加载参数的实现
        virtual bool loadFromFile(const std::string &filename) override
        {
            std::ifstream file(filename);
            if (!file.is_open())
            {
                ROS_ERROR("Failed to open parameter file: %s", filename.c_str());
                return false;
            }
            try
            {
                file >> paramDict;
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("Exception while reading JSON file: %s", e.what());
                return false;
            }
            // 从 JSON 中读取各个参数
            try
            {
                quadrotor_mq = paramDict["controller"]["quadrotor"]["mq"].get<double>();
                quadrotor_kp = paramDict["controller"]["quadrotor"]["kp"].get<double>();
                quadrotor_kv = paramDict["controller"]["quadrotor"]["kv"].get<double>();
                quadrotor_kr = paramDict["controller"]["quadrotor"]["kr"].get<double>();
                quadrotor_hr = paramDict["controller"]["quadrotor"]["hr"].get<double>();
                QSLS_bar_mQ = paramDict["controller"]["QSLS"]["bar_mQ"].get<double>();
                QSLS_bar_mL = paramDict["controller"]["QSLS"]["bar_mL"].get<double>();
                QSLS_l = paramDict["controller"]["QSLS"]["l"].get<double>();
                QSLS_k1 = paramDict["controller"]["QSLS"]["k1"].get<double>();
                QSLS_beta = paramDict["controller"]["QSLS"]["beta"].get<double>();
                QSLS_ks1 = paramDict["controller"]["QSLS"]["ks1"].get<double>();
                QSLS_k2 = paramDict["controller"]["QSLS"]["k2"].get<double>();
                QSLS_ks2 = paramDict["controller"]["QSLS"]["ks2"].get<double>();
                QSLS_hq = paramDict["controller"]["QSLS"]["hq"].get<double>();
                QSLS_kq = paramDict["controller"]["QSLS"]["kq"].get<double>();
                QSLS_hw = paramDict["controller"]["QSLS"]["hw"].get<double>();
                QSLS_kw = paramDict["controller"]["QSLS"]["kw"].get<double>();
                QSLS_hr = paramDict["controller"]["QSLS"]["hr"].get<double>();
                QSLS_kr = paramDict["controller"]["QSLS"]["kr"].get<double>();
                g = paramDict["controller"]["g"].get<double>();
                max_thrust = paramDict["controller"]["max_thrust"].get<double>();
                use_polyval = paramDict["controller"]["use_polyval"].get<bool>();
                p1 = Eigen::Map<const Eigen::VectorXd>(paramDict["controller"]["p1"].get<std::vector<double>>().data(), paramDict["controller"]["p1"].size());
                p2 = Eigen::Map<const Eigen::VectorXd>(paramDict["controller"]["p2"].get<std::vector<double>>().data(), paramDict["controller"]["p2"].size());
                p3 = Eigen::Map<const Eigen::VectorXd>(paramDict["controller"]["p3"].get<std::vector<double>>().data(), paramDict["controller"]["p3"].size());
                p4 = Eigen::Map<const Eigen::VectorXd>(paramDict["controller"]["p4"].get<std::vector<double>>().data(), paramDict["controller"]["p4"].size());
                obs_covQ_pQ = diag(Eigen::Map<const Eigen::Vector3d>(paramDict["observer"]["covQ_pQ_diag"].get<std::vector<double>>().data(), paramDict["observer"]["covQ_pQ_diag"].size()));
                obs_covQ_vQ = diag(Eigen::Map<const Eigen::Vector3d>(paramDict["observer"]["covQ_vQ_diag"].get<std::vector<double>>().data(), paramDict["observer"]["covQ_vQ_diag"].size()));
                obs_covQ_pL = diag(Eigen::Map<const Eigen::Vector3d>(paramDict["observer"]["covQ_pL_diag"].get<std::vector<double>>().data(), paramDict["observer"]["covQ_pL_diag"].size()));
                obs_covQ_vL = diag(Eigen::Map<const Eigen::Vector3d>(paramDict["observer"]["covQ_vL_diag"].get<std::vector<double>>().data(), paramDict["observer"]["covQ_vL_diag"].size()));
                obs_covQ_bQ = diag(Eigen::Map<const Eigen::Vector3d>(paramDict["observer"]["covQ_bQ_diag"].get<std::vector<double>>().data(), paramDict["observer"]["covQ_bQ_diag"].size()));
                obs_covQ_bL = diag(Eigen::Map<const Eigen::Vector3d>(paramDict["observer"]["covQ_bL_diag"].get<std::vector<double>>().data(), paramDict["observer"]["covQ_bL_diag"].size()));
                obs_covR_pQ = diag(Eigen::Map<const Eigen::Vector3d>(paramDict["observer"]["covR_pQ_diag"].get<std::vector<double>>().data(), paramDict["observer"]["covR_pQ_diag"].size()));
                obs_covR_q = diag(Eigen::Map<const Eigen::Vector3d>(paramDict["observer"]["covR_q_diag"].get<std::vector<double>>().data(), paramDict["observer"]["covR_q_diag"].size()));
                obs_covQ = blkdiag({obs_covQ_pQ, obs_covQ_vQ, obs_covQ_pL, obs_covQ_vL, obs_covQ_bQ, obs_covQ_bL});
                obs_covR = blkdiag({obs_covR_pQ, obs_covR_q});
                obs_invcovR = obs_covR.inverse();
                // solve Riccati equation begin
                // initialize A, B and C
                obs_A = Eigen::MatrixXd::Zero(18, 18);
                obs_B = Eigen::MatrixXd::Zero(18, 9);
                obs_C = Eigen::MatrixXd::Zero(6, 18);
                Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
                obs_A.block(0, 3, 3, 3) = I;
                obs_A.block(3, 12, 3, 3) = -I;
                obs_A.block(6, 9, 3, 3) = I;
                obs_A.block(9, 15, 3, 3) = I;
                obs_B.block(3, 0, 3, 3) = -1.0/QSLS_bar_mQ*I;
                obs_B.block(3, 3, 3, 3) = 1.0/QSLS_bar_mQ*I;
                obs_B.block(3, 6, 3, 3) = I;
                obs_B.block(9, 0, 3, 3) = 1.0/QSLS_bar_mL*I;
                obs_B.block(9, 6, 3, 3) = I;
                obs_C.block(0, 0, 3, 3) = I;
                obs_C.block(3, 0, 3, 3) = -I;
                obs_C.block(3, 6, 3, 3) = I;
                std::cout << obs_A << std::endl;
                std::cout << obs_C << std::endl;
                common::Integrator<boost::numeric::odeint::runge_kutta_fehlberg78<state_type>> integrator(std::bind(&SystemParams::Riccati, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::ref(obs_A), std::ref(obs_C), std::ref(obs_covQ), std::ref(obs_invcovR)), 0.01);
                // initialize P
                Eigen::MatrixXd P = Eigen::MatrixXd::Zero(18, 18);
                // initialize a vector for integrator
                Eigen::Map<Eigen::VectorXd> P_vector(P.data(), P.size());
                state_type x(P.size());
                for (size_t i = 0; i < x.size(); ++i) {
                    x[i] = P_vector[i];  // 将计算的dx/dt传回
                }
                integrator.integrate(x, 0, 10);
                P_vector = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
                P = Eigen::Map<const Eigen::MatrixXd>(P_vector.data(), P.rows(), P.cols());
                std::cout << "P:" << std::endl;
                std::cout << P << std::endl;
                obs_P = P;
                obs_K = obs_P*obs_C.transpose()*obs_invcovR;
                // solve Riccati equation end
                
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("Exception while parsing parameters from JSON: %s", e.what());
                return false;
            }
            ROS_INFO("Loaded parameters from file: %s", filename.c_str());
            return true;
        }
        // 生成对角阵
        Eigen::Matrix3d diag(const Eigen::Vector3d &v)
        {
            Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
            D(0, 0) = v(0);
            D(1, 1) = v(1);
            D(2, 2) = v(2);
            return D;
        }
        // 分块对角矩阵
        // 利用输入的多个方阵（此处示例为对角阵），在主对角线上拼接成一个更大的分块对角矩阵
        Eigen::MatrixXd blkdiag(const std::vector<Eigen::MatrixXd> &matrices)
        {
            // 1. 先计算拼接后矩阵的总行列数（假设每个矩阵都是方阵）
            int totalSize = 0;
            for (const auto &mat : matrices)
            {
                // 如果确实都是对角阵/方阵，这里 mat.rows() == mat.cols()
                totalSize += mat.rows();
            }

            // 2. 构造一个大的零矩阵，为分块对角矩阵准备空间
            Eigen::MatrixXd result = Eigen::MatrixXd::Zero(totalSize, totalSize);

            // 3. 逐个将小矩阵拷贝到大矩阵的对应对角块上
            int currentPos = 0;
            for (const auto &mat : matrices)
            {
                int blockSize = mat.rows(); // 等于 mat.cols()，因为 mat 是方阵
                // 将 mat 放到 (currentPos, currentPos) 这个对角位置上
                result.block(currentPos, currentPos, blockSize, blockSize) = mat;
                currentPos += blockSize;
            }

            return result;
        }
        void Riccati(const state_type &x, state_type &dxdt, double t, const Eigen::MatrixXd &A, const Eigen::MatrixXd &C, const Eigen::MatrixXd &covQ, const Eigen::MatrixXd &invcovR)
        {
            // use Eigen
            Eigen::Map<const Eigen::MatrixXd> P(x.data(), 18,18);
            Eigen::MatrixXd P_dot = -P*C.transpose()*invcovR*C*P + A*P + P*A.transpose() +covQ;
            Eigen::Map<Eigen::VectorXd> P_dot_vector(P_dot.data(), P_dot.size());
            for (size_t i = 0; i < x.size(); ++i) {
                dxdt[i] = P_dot_vector[i];  // 将计算的dx/dt传回
            }
        }
    };

} // namespace common

#endif // CUSTOM_SYSTEM_PARAMS_H
