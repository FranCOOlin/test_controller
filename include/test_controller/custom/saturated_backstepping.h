#ifndef CONTROLLER_SATURATED_BACKSTEPPING_H
#define CONTROLLER_SATURATED_BACKSTEPPING_H

#include "test_controller/controller/controller.h"
#include "test_controller/custom/system_params.h"
#include "test_controller/custom/qsls_state.h"
#include "test_controller/custom/mytrajectory.h"
#include "test_controller/custom/quadrotor_control_input.h"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <cmath>
#include <iostream>

namespace controller
{

  class SaturatedBackstepping : public Controller
  {
  public:
    common::SystemParams &params;
    common::QSLSState &state;
    common::MyTrajectory &trajectory;
    common::QuadrotorControlInput &control_input;
    SaturatedBackstepping(common::SystemParams &_params, common::QSLSState &_state, common::MyTrajectory &_trajectory, common::QuadrotorControlInput &_control_input)
        : params(_params), state(_state), trajectory(_trajectory), control_input(_control_input) {}

    double clamp(double x, double min, double max)
    {
      return x < min ? min : (x > max ? max : x);
    }

    // 多项式计算
    double polyval(double x, const Eigen::VectorXd &p)
    {

      // 多项式计算 y = polyval(p1, x)
      double y = 0.0;
      int degree = p.size() - 1;
      for (int i = 0; i < p.size(); ++i)
      {
        y += p[i] * std::pow(x, degree - i);
      }

      return y;
    }

    // 符号函数
    template <typename T>
    int sign(T value)
    {
      if (value > 0)
        return 1;
      if (value < 0)
        return -1;
      return 0;
    }

    // 交叉乘积矩阵
    inline Eigen::Matrix3d S(const Eigen::Vector3d &vec)
    {
      Eigen::Matrix3d mat;
      mat << 0, -vec(2), vec(1),
          vec(2), 0, -vec(0),
          -vec(1), vec(0), 0;
      return mat;
    }

    // 投影矩阵
    inline Eigen::Matrix3d PI(const Eigen::Vector3d &vec)
    {
      return Eigen::Matrix3d::Identity() - vec * vec.transpose();
    }

    virtual ~SaturatedBackstepping() {}

    inline double sigma_scalar(double x)
    {
      return std::tanh(x);
    }

    inline Eigen::Vector3d sigma(const Eigen::Vector3d &v)
    {
      return Eigen::Vector3d(
          sigma_scalar(v(0)),
          sigma_scalar(v(1)),
          sigma_scalar(v(2)));
    }

    inline double dsigma_scalar(double x)
    {
      double sech_x = 1.0 / std::cosh(x); // sech(x)
      return sech_x * sech_x;             // sech^2(x)
    }

    inline Eigen::Matrix3d dsigma(const Eigen::Vector3d &v)
    {
      double val0 = dsigma_scalar(v(0));
      double val1 = dsigma_scalar(v(1));
      double val2 = dsigma_scalar(v(2));
      return diag(Eigen::Vector3d(val0, val1, val2));
    }

    inline double d2sigma_scalar(double x)
    {
      // sech(x) = 1 / cosh(x)
      double sech_x = 1.0 / std::cosh(x);
      double sech2 = sech_x * sech_x; // sech^2(x)
      double tanh_x = std::tanh(x);

      // -2 * sech^2(x) * tanh(x)
      return -2.0 * sech2 * tanh_x;
    }

    inline Eigen::Matrix3d d2sigma(const Eigen::Vector3d &v)
    {
      double val0 = d2sigma_scalar(v(0));
      double val1 = d2sigma_scalar(v(1));
      double val2 = d2sigma_scalar(v(2));
      return diag(Eigen::Vector3d(val0, val1, val2));
    }

    inline double d3sigma_scalar(double x)
    {
      // sech(x) = 1 / cosh(x)
      double sech_x = 1.0 / std::cosh(x);
      double sech2 = sech_x * sech_x; // sech^2(x)
      double tanh_x = std::tanh(x);
      double tanh2 = tanh_x * tanh_x; // tanh^2(x)

      // -2 * ( -2 * sech^2(x)*tanh^2(x) + sech^4(x) )
      return -2.0 * (-2.0 * sech2 * tanh2 + sech2 * sech2);
    }

    /**
     * @brief d3sigma: 对向量逐元素计算 d3sigma_scalar.
     */
    inline Eigen::Matrix3d d3sigma(const Eigen::Vector3d &v)
    {
      double val0 = d3sigma_scalar(v(0));
      double val1 = d3sigma_scalar(v(1));
      double val2 = d3sigma_scalar(v(2));
      return diag(Eigen::Vector3d(val0, val1, val2));
    }

    /**
     * @brief diag: 将 3x1 向量作为对角线元素，生成 3x3 对角矩阵
     * @param v 3x1
     * @return 3x3 对角矩阵
     */
    inline Eigen::Matrix3d diag(const Eigen::Vector3d &v)
    {
      Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
      D(0, 0) = v(0);
      D(1, 1) = v(1);
      D(2, 2) = v(2);
      return D;
    }

    // 实现 update()，计算控制信号（此处仅为示例：返回目标航点与当前位置的差值组成的 1 维向量）
    virtual void update() override
    {
      // ------------------------
      // 第一部分: 提取输入与初始量
      // ------------------------

      // Extract vectors from input
      if (!state.updated)
      {
        return;
      }
      Eigen::Vector3d hat_pQ = state.pQ;
      Eigen::Vector3d hat_vQ = state.vQ;
      Eigen::Vector3d hat_pL = state.pL;
      Eigen::Vector3d hat_vL = state.vL;
      Eigen::Vector3d hat_q = state.q;
      Eigen::Vector3d hat_w = state.w;
      Eigen::Vector3d hat_bQ = state.bQ;
      Eigen::Vector3d hat_bL = state.bL;
      Eigen::Matrix3d R = state.R;

      Eigen::Vector3d pd = trajectory.pd;
      Eigen::Vector3d dpd = trajectory.dpd;
      Eigen::Vector3d d2pd = trajectory.d2pd;
      Eigen::Vector3d d3pd = trajectory.d3pd;
      Eigen::Vector3d d4pd = trajectory.d4pd;
      Eigen::Vector3d d5pd = trajectory.d5pd;

      double bar_mQ = params.QSLS_bar_mQ;
      double bar_mL = params.QSLS_bar_mL;
      double l = params.QSLS_l;
      double g = params.g;
      double k1 = params.QSLS_k1;
      double beta = params.QSLS_beta;
      double ks1 = params.QSLS_ks1;
      double k2 = params.QSLS_k2;
      double ks2 = params.QSLS_ks2;
      double hq = params.QSLS_hq;
      double kq = params.QSLS_kq;
      double hw = params.QSLS_hw;
      double kw = params.QSLS_kw;
      double hr = params.QSLS_hr;
      double kr = params.QSLS_kr;

      Eigen::Vector3d e3(0, 0, 1);
      double T;
      bool use_polyval = params.use_polyval;

      Eigen::Vector3d p1 = params.p1;
      Eigen::Vector3d p2 = params.p2;
      Eigen::Vector3d p3 = params.p3;
      Eigen::Vector3d p4 = params.p4;

      // ------------- 定义中间矩阵/向量 -------------
      Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
      Eigen::RowVector3d hat_q_T = hat_q.transpose(); // speedup
      Eigen::Matrix3d S_hat_w = S(hat_w); // speedup
      Eigen::Matrix3d P_hat_q = hat_q * hat_q_T;
      Eigen::Matrix3d PI_hat_q = I - P_hat_q;

      double hat_l = (hat_pL - hat_pQ).norm();  // hat_l = norm(hat_pL-hat_pQ);
      Eigen::Vector3d dhpL_0 = hat_vL;          // dhpL_0 = hat_vL;
      Eigen::Vector3d dhpQ_0 = hat_vQ;          // dhpQ_0 = hat_vQ;
      Eigen::Vector3d dhq_0 = S_hat_w * hat_q; // checked

      // ------------------------
      // 第二部分: Control input calculation
      // ------------------------

      // zp, zv, e and their derivatives
      Eigen::Vector3d zp = hat_pL - pd; // checked
      Eigen::Vector3d zv = hat_vL - dpd;
      Eigen::Vector3d e = k1 * (zp + beta * zv);
      Eigen::Vector3d dzp_0 = zv;
      Eigen::Vector3d gamma = ks1 * beta * sigma(e / ks1) + zv;
      double bar_mt = (bar_mQ + bar_mL); // speedup

      Eigen::Vector3d n = bar_mt * (hat_bL + g * e3 - d2pd + ks1 * sigma(e / ks1) + k2 * ks2 * sigma(gamma / ks2));
      double hat_w_squared_norm = hat_w.squaredNorm(); // speedup
      Eigen::Vector3d PqFd = -P_hat_q * n + bar_mQ * l * hat_w_squared_norm * hat_q + bar_mQ * P_hat_q * (hat_bQ + hat_bL);
      Eigen::Vector3d dzv_n = PqFd / bar_mt - (bar_mQ * l / bar_mt) * hat_w_squared_norm * hat_q - (bar_mQ / bar_mt) * P_hat_q * (hat_bQ + hat_bL) + hat_bL + g * e3 - d2pd;
      Eigen::Matrix3d P_zv_P_hvl = I;
      Eigen::Vector3d de_n = k1 * (dzp_0 + beta * dzv_n);
      Eigen::Matrix3d P_e_P_hvl = k1 * beta * P_zv_P_hvl;
      Eigen::Matrix3d dsigma_e_ks1 = dsigma(e / ks1); // speedup
      Eigen::Matrix3d dsigma_gamma_ks2 = dsigma(gamma / ks2); // speedup
      Eigen::Vector3d dgamma_n = beta * dsigma_e_ks1 * de_n + dzv_n;
      Eigen::Matrix3d P_gamma_P_hvl = beta * dsigma_e_ks1 * P_e_P_hvl + P_zv_P_hvl;
      Eigen::Vector3d dn_n = bar_mt * (-d3pd + dsigma_e_ks1 * de_n + k2 * dsigma_gamma_ks2 * dgamma_n);
      Eigen::Matrix3d P_n_P_hvl = bar_mt * (dsigma_e_ks1 * P_e_P_hvl + k2 * dsigma_gamma_ks2 * P_gamma_P_hvl);


      double n_norm = n.norm(); // speedup
      // Define qd
      Eigen::Vector3d qd = n / n_norm; 
      // Define zq
      Eigen::Vector3d zq = hat_q - qd;

      // Cable direction control
      // 计算 dqd_n = PIqd/n_norm*dn_n;
      
      Eigen::RowVector3d qd_T = qd.transpose(); // speedup
      Eigen::Matrix3d PIqd = I - qd * qd_T;
      Eigen::Vector3d dqd_n = (PIqd / n_norm) * dn_n;
      Eigen::Matrix3d P_qd_P_hvl = PIqd / n_norm * P_n_P_hvl; // checked

      Eigen::Matrix3d S_hat_q = S(hat_q); // speedup
      Eigen::Matrix3d S_qd = S(qd); // speedup
      // Define wd
      Eigen::Vector3d wd = S_qd * dqd_n + (2.0 * kq / hq) / (1.0 + hat_q.dot(qd)) * S_hat_q * qd + n_norm / (hq * bar_mt) * S_hat_q * gamma;

      // zw = S_hat_q*(hat_w - wd);


      Eigen::Vector3d zw = S_hat_q * (hat_w - wd); // checked
      Eigen::RowVector3d P_V2_P_zv = gamma.transpose() - hq * hat_q_T * PIqd * P_qd_P_hvl;
      
      Eigen::RowVector3d dhq_0_T = dhq_0.transpose(); // speedup
      // calculate d_dzv_n
      Eigen::Vector3d d_dzv_n_n = -1.0 / bar_mt * (dhq_0 * hat_q_T * n + hat_q * dhq_0_T * n + hat_q * hat_q_T * dn_n) - d3pd;
      Eigen::Matrix3d P_dzv_n_P_hvl = -1.0 / bar_mt * P_hat_q * P_n_P_hvl;
      // calculate d_de_n
      Eigen::Vector3d d_de_n_n = k1 * (dzv_n + beta * d_dzv_n_n);
      Eigen::Matrix3d P_de_n_P_hvl = k1 * (P_zv_P_hvl + beta * P_dzv_n_P_hvl);

      Eigen::Matrix3d d2sigma_e_ks1 = d2sigma(e / ks1); // speedup
      Eigen::Matrix3d d2sigma_gamma_ks2 = d2sigma(gamma / ks2); // speedup
      Eigen::Matrix3d diag_de_n = diag(de_n); // speedup
      Eigen::Matrix3d diag_dgamma_n = diag(dgamma_n); // speedup
      // calculate d_dgamma_n
      Eigen::Vector3d d_dgamma_n_n = beta / ks1 * d2sigma_e_ks1 * diag_de_n * de_n + beta * dsigma_e_ks1 * d_de_n_n + d_dzv_n_n; // checked
      Eigen::Matrix3d P_dgamma_n_P_hvl = beta / ks1 * d2sigma_e_ks1 * diag_de_n * P_e_P_hvl + beta * dsigma_e_ks1 * P_de_n_P_hvl + P_dzv_n_P_hvl;
      // calculate d_dn_n_n
      Eigen::Vector3d d_dn_n_n = bar_mt * (-d4pd + 1.0 / ks1 * d2sigma_e_ks1 * diag_de_n * de_n + dsigma_e_ks1 * d_de_n_n + k2 / ks2 * d2sigma_gamma_ks2 * diag_dgamma_n * dgamma_n + k2 * dsigma_gamma_ks2 * d_dgamma_n_n); // checked
      Eigen::Matrix3d P_dn_n_P_hvl = bar_mt * (1.0 / ks1 * d2sigma_e_ks1 * diag_de_n * P_e_P_hvl + dsigma_e_ks1 * P_de_n_P_hvl + k2 / ks2 * d2sigma_gamma_ks2 * diag_dgamma_n * P_gamma_P_hvl + k2 * dsigma_gamma_ks2 * P_dgamma_n_P_hvl);

      // calculate d_dqd_n_n
      Eigen::RowVector3d dn_n_T = dn_n.transpose(); // speedup
      Eigen::RowVector3d n_T = n.transpose(); // speedup
      double n_norm_3 = pow(n_norm, 3); // speedup
      Eigen::Vector3d d_dqd_n_n = PIqd / n_norm * d_dn_n_n - 1.0 / n_norm * (qd_T * dn_n * I + qd * dn_n_T) * dqd_n - PIqd / n_norm_3 * dn_n * n_T * dn_n;
      Eigen::Matrix3d P_dqd_n_P_hvl = PIqd / n_norm * P_dn_n_P_hvl - 1.0 / n_norm * (qd_T * dn_n * I + qd * dn_n_T) * P_qd_P_hvl - PIqd / n_norm_3 * dn_n * n_T * P_n_P_hvl; // checked

      // calculate dwd_n

      Eigen::Vector3d dwd_n = S_qd * d_dqd_n_n + 2.0 * kq / (hq * (1.0 + hat_q_T * qd)) * (S_hat_q * dqd_n - S_qd * S_hat_w * hat_q) - 2.0 * kq / (hq * pow((1.0 + hat_q_T * qd), 2)) * (qd.dot(S_hat_w * hat_q) + hat_q.dot(dqd_n)) * S_hat_q * qd + 1.0 / hq / bar_mt * (n_T / n_norm * dn_n * S_hat_q * gamma - n_norm * S(gamma) * S_hat_w * hat_q + beta * n_norm * S_hat_q * dsigma_e_ks1 * de_n + n_norm * S_hat_q * dzv_n); // checked
      Eigen::Matrix3d P_wd_P_hvl = S_qd * P_dqd_n_P_hvl - S(dqd_n) * P_qd_P_hvl + 2.0 * kq / hq / (1.0 + hat_q_T * qd) * (S_hat_q * P_qd_P_hvl - S_hat_q * qd * hat_q_T * P_qd_P_hvl / (1.0 + hat_q_T * qd)) + 1.0 / hq / bar_mt * (1.0 / n_norm * S_hat_q * gamma * n_T * P_n_P_hvl + beta * n_norm * S_hat_q * dsigma_e_ks1 * P_e_P_hvl + n_norm * S_hat_q * P_zv_P_hvl);

      // Now we can design PIq_Fd

      Eigen::Matrix3d S_hat_w_wd = S(hat_w - wd);

      Eigen::Vector3d PIq_Fd = bar_mQ * hat_l * -PI_hat_q * (-(hat_bQ + hat_bL) / hat_l - S_hat_q * dwd_n - S_hat_w_wd * dhq_0 + kw / hw * zw + hq / hw * qd);

      // Combine PqFd and PIq_Fd to get Fd

      Eigen::Vector3d Fd = PqFd + PIq_Fd;

      // Then we can design F

      double Td = Fd.norm();
      Eigen::Vector3d r3 = R * e3;
      Eigen::Matrix3d PI_r3 = I - r3 * r3.transpose();
      Eigen::Vector3d r3d = -Fd / Td;
      T = Td * r3d.transpose() * r3;
      Eigen::Vector3d F = Fd + Td * PI_r3 * r3d;

      Eigen::RowVector3d P_V3_P_zv = P_V2_P_zv - hw * zw.transpose() * S_hat_q * P_wd_P_hvl;
      // Now we can get the equation of F
      // Then we can get dx_0 = dx_n + 1/(bar_mQ+bar_mL)*P_x_P_hvl*PqF_PqFd and dx = dx_0 + delta_dx

      Eigen::Vector3d PqF_PqFd = P_hat_q * F - PqFd;

      Eigen::Vector3d dhw_0 = -S_hat_q * F / bar_mQ / hat_l + S_hat_q * (hat_bQ + hat_bL) / hat_l + 2.0 * S(hat_vL - hat_vQ) * (hat_pL - hat_pQ) * (hat_pL - hat_pQ).transpose() * (hat_vL - hat_vQ) / pow(hat_l, 4);

      Eigen::Vector3d dzv_0 = dzv_n + 1.0 / bar_mt * P_zv_P_hvl * PqF_PqFd;

      Eigen::Vector3d de_0 = k1 * (dzp_0 + beta * dzv_0);

      Eigen::Vector3d dgamma_0 = dgamma_n + 1.0 / bar_mt * P_gamma_P_hvl * PqF_PqFd;

      Eigen::Vector3d dn_0 = dn_n + 1.0 / bar_mt * P_n_P_hvl * PqF_PqFd;

      Eigen::Vector3d dqd_0 = dqd_n + 1.0 / bar_mt * P_qd_P_hvl * PqF_PqFd;

      Eigen::Vector3d d_dzv_n_0 = d_dzv_n_n + 1.0 / bar_mt * P_dzv_n_P_hvl * PqF_PqFd;

      Eigen::Vector3d d_de_n_0 = d_de_n_n + 1.0 / bar_mt * P_de_n_P_hvl * PqF_PqFd;

      Eigen::Vector3d d_dgamma_n_0 = d_dgamma_n_n + 1.0 / bar_mt * P_dgamma_n_P_hvl * PqF_PqFd;

      Eigen::Vector3d d_dn_n_0 = d_dn_n_n + 1.0 / bar_mt * P_dn_n_P_hvl * PqF_PqFd;

      Eigen::Vector3d d_dqd_n_0 = d_dqd_n_n + 1.0 / bar_mt * P_dqd_n_P_hvl * PqF_PqFd;

      Eigen::Vector3d dwd_0 = dwd_n + 1.0 / bar_mt * P_wd_P_hvl * PqF_PqFd;

      Eigen::Vector3d dzw_n = -S_hat_w_wd * dhq_0 + PI_hat_q * F / (bar_mQ * hat_l) - PI_hat_q * (hat_bQ + hat_bL) / hat_l + 2.0 * hat_q_T * (hat_vL - hat_vQ) * PI_hat_q * (hat_vL - hat_vQ) / pow(hat_l, 2) - S_hat_q * dwd_n;
      Eigen::Matrix3d P_zw_P_hvl = -S_hat_q * P_wd_P_hvl;
      Eigen::Vector3d dzw_0 = dzw_n + 1.0 / bar_mt * P_zw_P_hvl * PqF_PqFd;

      // calculate d_dhq_0_0

      Eigen::Vector3d d_dhq_0_0 = S_hat_w * dhq_0 - S_hat_q * dhw_0;

      // calculate d_d_dzv_n_n_0
      Eigen::Vector3d d_d_dzv_n_n_0 = -1.0 / bar_mt * (d_dhq_0_0 * hat_q_T * n + dhq_0 * dhq_0_T * n + dhq_0 * hat_q_T * dn_0 + dhq_0 * dhq_0_T * n + hat_q * d_dhq_0_0.transpose() * n + hat_q * dhq_0_T * dn_0 + dhq_0 * hat_q_T * dn_n + hat_q * dhq_0_T * dn_n + hat_q * hat_q_T * d_dn_n_0) - d4pd;

      // calculate d_d_de_n_n_0
      Eigen::Vector3d d_d_de_n_n_0 = k1 * (d_dzv_n_0 + beta * d_d_dzv_n_n_0);

      Eigen::Matrix3d d3sigma_e_ks1 = d3sigma(e / ks1); // speedup
      // calculate d_d_dgamma_n_n_0
      Eigen::Vector3d d_d_dgamma_n_n_0 = beta / pow(ks1, 2) * d3sigma_e_ks1 * diag_de_n * diag_de_n * de_0 + 2.0 * beta / ks1 * d2sigma_e_ks1 * diag_de_n * d_de_n_0 + beta / ks1 * d2sigma_e_ks1 * diag(d_de_n_n) * de_0 + beta * dsigma_e_ks1 * d_d_de_n_n_0 + d_d_dzv_n_n_0;

      // calculate d_d_dn_n_n_0
      Eigen::Vector3d d_d_dn_n_n_0 = bar_mt * (-d5pd + 1.0 / pow(ks1, 2) * d3sigma_e_ks1 * diag_de_n * diag_de_n * de_0 + 2.0 / ks1 * d2sigma_e_ks1 * diag_de_n * d_de_n_0 + 1.0 / ks1 * d2sigma_e_ks1 * diag(d_de_n_n) * de_0 + dsigma_e_ks1 * d_d_de_n_n_0 + k2 / pow(ks2, 2) * d3sigma(gamma / ks2) * diag_dgamma_n * diag_dgamma_n * dgamma_0 + 2.0 * k2 / ks2 * d2sigma_gamma_ks2 * diag_dgamma_n * d_dgamma_n_0 + k2 / ks2 * d2sigma_gamma_ks2 * diag(d_dgamma_n_n) * dgamma_0 + k2 * dsigma_gamma_ks2 * d_d_dgamma_n_n_0);
      Eigen::RowVector3d dqd_0_T = dqd_0.transpose(); // speedup
      // calculate d_d_dqd_n_n_0

      Eigen::Vector3d d_d_dqd_n_n_0 = -PIqd / n_norm_3 * (n_T * dn_0) * d_dn_n_n - 1.0 / n_norm * (dqd_0 * qd_T + qd * dqd_0_T) * d_dn_n_n + PIqd / n_norm * d_d_dn_n_n_0 + n.dot(dn_0) / n_norm_3 * (qd.dot(dn_n) * dqd_n + qd * (dn_n.dot(dqd_n))) - 1.0 / n_norm * (dqd_0_T * dn_n * I + qd_T * d_dn_n_0 * I + dqd_0 * dn_n_T + qd * d_dn_n_0.transpose()) * dqd_n - 1.0 / n_norm * (qd_T * dn_n * I + qd * dn_n_T) * d_dqd_n_0 + 3.0 * PIqd / pow(n_norm, 5) * (n_T * dn_0 * n_T * dn_n) * dn_n + 1.0 / n_norm_3 * (dqd_0 * qd_T + qd * dqd_0_T) * (n_T * dn_n) * dn_n - PIqd / n_norm_3 * d_dn_n_0 * n_T * dn_n - PIqd / n_norm_3 * dn_n * dn_0.transpose() * dn_n - PIqd / n_norm_3 * dn_n * n_T * d_dn_n_0;


      // ros::Time t_start = ros::Time::now();
      double d_dwd_n_0_temp_1 = kq / hq / pow((1.0 + hat_q.dot(qd)), 2);
      // calculate d_dwd_n_0
      Eigen::Vector3d d_dwd_n_0 = S(dqd_0) * d_dqd_n_n + S_qd * d_d_dqd_n_n_0 - 2.0 * d_dwd_n_0_temp_1 * (hat_q.dot(dqd_0) + qd.dot(dhq_0)) * (S_hat_q * dqd_n - S_qd * S_hat_w * hat_q) + 2.0 * kq / hq / (1.0 + hat_q_T * qd) * (S(dhq_0) * dqd_n + S_hat_q * d_dqd_n_0 - S(dqd_0) * S_hat_w * hat_q - S_qd * S(dhw_0) * hat_q - S_qd * S_hat_w * dhq_0) + 4.0 * kq / hq / pow((1.0 + hat_q_T * qd), 3) * (hat_q_T * dqd_0 + qd_T * dhq_0) * (qd_T * S_hat_w * hat_q + hat_q_T * dqd_n) * S_hat_q * qd - 2.0 * d_dwd_n_0_temp_1 * (dqd_0.dot(S_hat_w * hat_q) + qd.dot(S(dhw_0) * hat_q) + qd.dot(S_hat_w * dhq_0) + dhq_0.dot(dqd_n) + hat_q.dot(d_dqd_n_0)) * S_hat_q * qd - 2.0 * d_dwd_n_0_temp_1 * (qd.dot(S_hat_w * hat_q) + hat_q.dot(dqd_n)) * (S(dhq_0) * qd + S_hat_q * dqd_0) + 1.0 / hq / bar_mt * (dn_n_T * (I / n_norm - n * n_T / n_norm_3) * dn_0 * S_hat_q * gamma + n_T / n_norm * d_dn_n_0 * S_hat_q * gamma + n_T / n_norm * dn_n * S(dhq_0) * gamma + n_T / n_norm * dn_n * S_hat_q * dgamma_0 - n.dot(dn_0) / n_norm * S(gamma) * S_hat_w * hat_q - n_norm * S(dgamma_0) * S_hat_w * hat_q - n_norm * S(gamma) * S(dhw_0) * hat_q - n_norm * S(gamma) * S_hat_w * dhq_0 + beta * n.dot(dn_0) / n_norm * S_hat_q * dsigma_e_ks1 * de_n + beta * n_norm * S(dhq_0) * dsigma_e_ks1 * de_n + beta / ks1 * n_norm * S_hat_q * d2sigma_e_ks1 * diag_de_n * de_0 + beta * n_norm * S_hat_q * dsigma_e_ks1 * d_de_n_0 + n.dot(dn_0) / n_norm * S_hat_q * dzv_n + n_norm * S(dhq_0) * dzv_n + n_norm * S_hat_q * d_dzv_n_0);

      // ros::Time t_end = ros::Time::now();
      // ROS_INFO("Time elapsed: %f", (t_end - t_start).toSec());
      // calculate d_PqFd_0

      Eigen::Vector3d d_PqFd_0 = -(hat_q_T * n * dhq_0 + hat_q * n_T * dhq_0) - hat_q * hat_q_T * dn_0 + 2 * bar_mQ * l * hat_q * hat_w.transpose() * dhw_0 + bar_mQ * l * hat_w_squared_norm * dhq_0 + bar_mQ * hat_q_T * (hat_bQ + hat_bL) * dhq_0 + bar_mQ * hat_q * (hat_bQ + hat_bL).transpose() * dhq_0;

      // calculate d_hat_l

      double d_hat_l_0 = hat_q_T * (dhpL_0 - dhpQ_0);

      // calculate d_PIq_Fd_0

      Eigen::Vector3d d_PIq_Fd_0 = bar_mQ * (dhq_0 * hat_q_T + hat_q * dhq_0_T) * (-(hat_bQ + hat_bL) - hat_l * S_hat_q * dwd_n + hat_l * S(dhq_0) * (hat_w - wd) + hq / hw * hat_l * qd + kw / hw * hat_l * zw) + bar_mQ * (hat_q * hat_q_T - I) * (-hat_l * S(dhq_0) * dwd_n - hat_l * S_hat_q * d_dwd_n_0 + hat_l * S(d_dhq_0_0) * (hat_w - wd) + hat_l * S(dhq_0) * (dhw_0 - dwd_0) + hq / hw * hat_l * dqd_0 + kw / hw * hat_l * dzw_0) + bar_mQ * (hat_q * hat_q_T - I) * d_hat_l_0 * (-S_hat_q * dwd_n + S(dhq_0) * (hat_w - wd) + hq / hw * qd + kw / hw * zw);

      // calculate d_r3d
      Eigen::Vector3d dr3d_0 = (r3d * r3d.transpose() - I) / Td * (d_PqFd_0 + d_PIq_Fd_0);

      // Now we can design Omega

      Eigen::Vector3d Omega = -S(e3) * S(e3) * (hw * Td / (hr * bar_mQ * hat_l) * S(e3) * R.transpose() * zw + Td / (hr * bar_mt) * S(e3) * R.transpose() * hat_q * hat_q_T * P_V3_P_zv.transpose() + R.transpose() * S(r3d) * dr3d_0 + 2 * kr / (hr * (1.0 + r3.transpose() * r3d)) * S(e3) * R.transpose() * r3d);
      T = clamp(T, 0, params.max_thrust);
      control_input.thrust = T;
      control_input.omega = Omega;
      if (use_polyval)
      {
        T = clamp(polyval(T * 1000 / 9.8, p1), 0, 1.0);
        Omega(0) = clamp(sign(Omega(0)) * polyval(abs(Omega(0)), p3), -3.14, 3.14);
        Omega(1) = clamp(sign(Omega(1)) * polyval(abs(Omega(1)), p2), -3.14, 3.14);
        Omega(2) = clamp(0 * sign(Omega(2)) * polyval(abs(Omega(2)), p4), -3.14, 3.14);
      }
      control_input.mavlink_thrust = T;
      control_input.mavlink_omega = Omega;
      state.updated = false; // 等待下一次状态更新
      // control_input.omega = dwd_n;
    }
  };

} // namespace controller

#endif // CONTROLLER_SATURATED_BACKSTEPPING_H
