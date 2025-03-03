#ifndef SATURATED_BACKSTEPPING_H
#define SATURATED_BACKSTEPPING_H

#include "test_controller/controller/controller.h"
#include "test_controller/custom/myparams.h"
#include "test_controller/custom/mystate.h"
#include "test_controller/custom/mytrajectory.h"
#include "test_controller/custom/mycontrol_input.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>

namespace controller
{

  class saturatedBackstepping : public Controller
  {
  public:
    common::MyParams &params;
    common::MyState &state;
    common::MyTrajectory &trajectory;
    common::MyControlInput &control_input;
    saturatedBackstepping(common::MyParams &_params, common::MyState &_state, common::MyTrajectory &_trajectory, common::MyControlInput &_control_input)
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
    Eigen::Matrix3d S(const Eigen::Vector3d &vec)
    {
      Eigen::Matrix3d mat;
      mat << 0, -vec(2), vec(1),
          vec(2), 0, -vec(0),
          -vec(1), vec(0), 0;
      return mat;
    }

    // 投影矩阵
    Eigen::Matrix3d PI(const Eigen::Vector3d &vec)
    {
      return Eigen::Matrix3d::Identity() - vec * vec.transpose();
    }

    virtual ~saturatedBackstepping() {}

    //----------------------------------------------------
    // 需要的占位辅助函数（请提供或自行实现）：
    //----------------------------------------------------

    /**
     * @brief 返回输入向量 v 的反对称矩阵 (skew-symmetric matrix), 即 S(v).
     *        用于向量叉乘运算:  S(a)*b = a x b
     *
     * @param v 3x1向量
     * @return 3x3矩阵
     */
    inline Eigen::Matrix3d S(const Eigen::Vector3d &v)
    {
      Eigen::Matrix3d M;
      M << 0, -v.z(), v.y(),
          v.z(), 0, -v.x(),
          -v.y(), v.x(), 0;
      return M;
    }

    /**
     * @brief sigma_scalar(x): 标量的 sigma 函数（示例）。
     *        若需要对向量逐元素应用, 可对每个元素调用此函数.
     *        请根据您的实际需求进行实现.
     */
    inline double sigma_scalar(double x)
    {
      // 这里是占位实现, 您可自行替换为真实的非线性函数
      // 例如: return std::tanh(x);
      return x;
    }

    /**
     * @brief sigma 对 3x1 向量逐元素调用 sigma_scalar.
     * @param v 3x1向量
     * @return 3x1向量
     */
    inline Eigen::Vector3d sigma(const Eigen::Vector3d &v)
    {
      return Eigen::Vector3d(
          sigma_scalar(v(0)),
          sigma_scalar(v(1)),
          sigma_scalar(v(2)));
    }

    /**
     * @brief dsigma_scalar(x): sigma 的一阶导数（标量版本）.
     *        若 sigma_scalar(x) = tanh(x)，则 dsigma_scalar(x) = sech,2)(x)
     *        以下仅占位.
     */
    inline double dsigma_scalar(double x)
    {
      // 请根据实际需求实现.
      return 1.0;
    }

    /**
     * @brief dsigma: 对向量逐元素计算 dsigma_scalar.
     * @param v 3x1
     * @return 3x1
     */
    inline Eigen::Vector3d dsigma(const Eigen::Vector3d &v)
    {
      return Eigen::Vector3d(
          dsigma_scalar(v(0)),
          dsigma_scalar(v(1)),
          dsigma_scalar(v(2)));
    }

    /**
     * @brief d2sigma_scalar(x): sigma 的二阶导数.
     *        以下仅占位.
     */
    inline double d2sigma_scalar(double x)
    {
      return 0.0; // 请根据实际需求实现.
    }

    /**
     * @brief d2sigma: 对向量逐元素计算 d2sigma_scalar.
     */
    inline Eigen::Vector3d d2sigma(const Eigen::Vector3d &v)
    {
      return Eigen::Vector3d(
          d2sigma_scalar(v(0)),
          d2sigma_scalar(v(1)),
          d2sigma_scalar(v(2)));
    }

    /**
     * @brief d3sigma_scalar(x): sigma 的三阶导数.
     *        以下仅占位.
     */
    inline double d3sigma_scalar(double x)
    {
      return 0.0; // 请根据实际需求实现.
    }

    /**
     * @brief d3sigma: 对向量逐元素计算 d3sigma_scalar.
     */
    inline Eigen::Vector3d d3sigma(const Eigen::Vector3d &v)
    {
      return Eigen::Vector3d(
          d3sigma_scalar(v(0)),
          d3sigma_scalar(v(1)),
          d3sigma_scalar(v(2)));
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

    //----------------------------------------------------
    // 主函数: Controller
    //----------------------------------------------------

    /**
     * @brief 将原 MATLAB 版的 Controller 函数转换为 C++ 版本(完整展开)，使用Eigen进行向量和矩阵运算.
     *
     * 这是对您提供的完整 MATLAB 版函数的翻译，所有中间量和推导都保留. 其中一些函数(例如 sigma,
     * dsigma, d2sigma, d3sigma, S, diag等) 需要您结合实际的非线性函数实现.
     *
     * @param t         时间标量
     * @param pd        期望位置(3x1)
     * @param dpd       期望速度(3x1)
     * @param d2pd      期望加速度(3x1)
     * @param d3pd      期望三阶导(3x1)
     * @param d4pd      期望四阶导(3x1)
     * @param d5pd      期望五阶导(3x1)
     * @param hat_eta   [hat_pQ(3), hat_vQ(3)]共6维
     * @param hat_x     [hat_pL(3), hat_vL(3), hat_q(3), hat_w(3), hat_bQ(3), hat_bL(3)] 共18维
     * @param R         旋转矩阵(3x3)
     * @param bar_mQ    标量
     * @param bar_mL    标量
     * @param l         标量
     * @param g         重力加速度
     * @param e3        (0, 0, 1) 方向向量
     * @param k1        标量增益
     * @param beta      标量增益
     * @param ks1       标量增益
     * @param k2        标量增益
     * @param ks2       标量增益
     * @param hq        标量增益
     * @param kq        标量增益
     * @param hw        标量增益
     * @param kw        标量增益
     * @param hr        标量增益
     * @param kr        标量增益
     * @param[out] zp   3x1
     * @param[out] zv   3x1
     * @param[out] zq   3x1
     * @param[out] zw   3x1
     * @param[out] zr   3x1
     * @param[out] T    标量
     * @param[out] Omega 3x1
     * @param[out] V    标量
     * @param[out] dV   标量
     */

    inline void Controller(
        double t,
        const Eigen::Vector3d &pd,
        const Eigen::Vector3d &dpd,
        const Eigen::Vector3d &d2pd,
        const Eigen::Vector3d &d3pd,
        const Eigen::Vector3d &d4pd,
        const Eigen::Vector3d &d5pd,
        const Eigen::VectorXd &hat_eta,
        const Eigen::VectorXd &hat_x,
        const Eigen::Matrix3d &R,
        double bar_mQ,
        double bar_mL,
        double l,
        double g,
        const Eigen::Vector3d &e3,
        double k1,
        double beta,
        double ks1,
        double k2,
        double ks2,
        double hq,
        double kq,
        double hw,
        double kw,
        double hr,
        double kr,
        // 输出
        Eigen::Vector3d &zr,
        double &T,
        Eigen::Vector3d &Omega,
        double &V,
        double &dV)
    {
      // ------------------------
      // 第一部分: 提取输入与初始量
      // ------------------------

      // Extract vectors from input
      Eigen::Vector3d hat_pQ = hat_eta.segment<3>(0); // hat_eta(1:3)
      Eigen::Vector3d hat_vQ = hat_eta.segment<3>(3); // hat_eta(4:6)

      Eigen::Vector3d hat_pL = hat_x.segment<3>(0);  // hat_x(1:3)
      Eigen::Vector3d hat_vL = hat_x.segment<3>(3);  // hat_x(4:6)
      Eigen::Vector3d hat_q = hat_x.segment<3>(6);   // hat_x(7:9)
      Eigen::Vector3d hat_w = hat_x.segment<3>(9);   // hat_x(10:12)
      Eigen::Vector3d hat_bQ = hat_x.segment<3>(12); // hat_x(13:15)
      Eigen::Vector3d hat_bL = hat_x.segment<3>(15); // hat_x(16:18)

      // ------------- 定义中间矩阵/向量 -------------
      Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d P_hat_q = hat_q * hat_q.transpose();
      Eigen::Matrix3d PI_hat_q = I - P_hat_q;

      double hat_l = (hat_pL - hat_pQ).norm();  // hat_l = norm(hat_pL-hat_pQ);
      Eigen::Vector3d dhpL_0 = hat_vL;          // dhpL_0 = hat_vL;
      Eigen::Vector3d dhpQ_0 = hat_vQ;          // dhpQ_0 = hat_vQ;
      Eigen::Vector3d dhq_0 = S(hat_w) * hat_q; // dhq_0 = S(hat_w)*hat_q;

      // ------------------------
      // 第二部分: Control input calculation
      // ------------------------

      // zp, zv, e and their derivatives
      Eigen::Vector3d zp = hat_pL - pd;
      Eigen::Vector3d zv = hat_vL - dpd;
      Eigen::Vector3d e = k1 * (zp + beta * zv);
      Eigen::Vector3d dzp_0 = zv;
      Eigen::Vector3d gamma = ks1 * beta * sigma(e / ks1) + zv;
      Eigen::Vector3d n = (bar_mQ + bar_mL) * (hat_bL + g * e3 - d2pd + ks1 * sigma(e / ks1) + k2 * ks2 * sigma(gamma / ks2));
      Eigen::Vector3d PqFd = -P_hat_q * n + bar_mQ * l * hat_w.squaredNorm() * hat_q + bar_mQ * P_hat_q * (hat_bQ + hat_bL);
      Eigen::Vector3d dzv_n = PqFd / (bar_mQ + bar_mL) - (bar_mQ * l / (bar_mQ + bar_mL)) * hat_w.squaredNorm() * hat_q - (bar_mQ / (bar_mQ + bar_mL)) * P_hat_q * (hat_bQ + hat_bL) + hat_bL + g * e3 - d2pd;
      Eigen::Matrix3d P_zv_P_hvl = I;
      Eigen::Vector3d de_n = k1 * (dzp_0 + beta * dzv_n);
      Eigen::Matrix3d P_e_P_hvl = k1 * beta * P_zv_P_hvl;
      Eigen::Vector3d dgamma_n = beta * dsigma(e / ks1).cwiseProduct(de_n) + dzv_n;
      Eigen::Matrix3d P_gamma_P_hvl = beta * dsigma(e / ks1) * P_e_P_hvl + P_zv_P_hvl;
      Eigen::Vector3d dn_n = (bar_mQ + bar_mL) * (-d3pd + dsigma(e / ks1).cwiseProduct(de_n) + k2 * dsigma(gamma / ks2).cwiseProduct(dgamma_n));
      Eigen::Matrix3d P_n_P_hvl = (bar_mQ + bar_mL) * (dsigma(e / ks1) * P_e_P_hvl + k2 * dsigma(gamma / ks2) * P_gamma_P_hvl);

      // Define qd
      Eigen::Vector3d qd = n / n.norm();
      // Define zq
      Eigen::Vector3d zq = hat_q - qd;

      // Cable direction control
      // 计算 dqd_n = PIqd/n.norm()*dn_n;
      Eigen::Matrix3d PIqd = I - qd * qd.transpose();
      Eigen::Vector3d dqd_n = (PIqd / n.norm()) * dn_n;
      Eigen::Matrix3d P_qd_P_hvl = PIqd / n.norm() * P_n_P_hvl;

      // Define wd
      Eigen::Vector3d wd = S(qd) * dqd_n + (2.0 * kq / hq) / (1.0 + hat_q.dot(qd)) * S(hat_q) * qd + n.norm() / (hq * (bar_mQ + bar_mL)) * S(hat_q) * gamma;

      // zw = S(hat_q)*(hat_w - wd);
      Eigen::Vector3d zw = S(hat_q) * (hat_w - wd);
      Eigen::Matrix3d P_V2_P_zv = gamma.transpose() - hq * hat_q.transpose() * PIqd * P_qd_P_hvl;
      // calculate d_dzv_n
      Eigen::Vector3d d_dzv_n_n = -1.0 / (bar_mQ + bar_mL) * (dhq_0 * hat_q.transpose() * n + hat_q * dhq_0.transpose() * n + hat_q * hat_q.transpose() * dn_n) - d3pd;
      Eigen::Vector3d P_dzv_n_P_hvl = -1.0 / (bar_mQ + bar_mL) * P_hat_q * P_n_P_hvl;
      // calculate d_de_n
      Eigen::Vector3d d_de_n_n = k1 * (dzv_n + beta * d_dzv_n_n);
      Eigen::Vector3d P_de_n_P_hvl = k1 * (P_zv_P_hvl + beta * P_dzv_n_P_hvl);
      // calculate d_dgamma_n
      Eigen::Vector3d d_dgamma_n_n = beta / ks1 * d2sigma(e / ks1) * diag(de_n) * de_n + beta * dsigma(e / ks1) * d_de_n_n + d_dzv_n_n;
      Eigen::Matrix3d P_dgamma_n_P_hvl = beta / ks1 * d2sigma(e / ks1) * diag(de_n) * P_e_P_hvl + beta * dsigma(e / ks1) * P_de_n_P_hvl + P_dzv_n_P_hvl;
      // calculate d_dn_n_n
      Eigen::Vector3d d_dn_n_n = (bar_mQ + bar_mL) * (-d4pd + 1 / ks1 * d2sigma(e / ks1) * diag(de_n) * de_n + dsigma(e / ks1) * d_de_n_n + k2 / ks2 * d2sigma(gamma / ks2) * diag(dgamma_n) * dgamma_n + k2 * dsigma(gamma / ks2) * d_dgamma_n_n);               // checked
      Eigen::Matrix3d P_dn_n_P_hvl = (bar_mQ + bar_mL) * (1 / ks1 * d2sigma(e / ks1) * diag(de_n) * P_e_P_hvl + dsigma(e / ks1) * P_de_n_P_hvl + k2 / ks2 * d2sigma(gamma / ks2) * diag(dgamma_n) * P_gamma_P_hvl + k2 * dsigma(gamma / ks2) * P_dgamma_n_P_hvl); // checked

      // calculate d_dqd_n_n

      Eigen::Vector3d d_dqd_n_n = PIqd/n.norm()*d_dn_n_n-1/n.norm()*(qd.transpose()*dn_n*I+qd*dn_n.transpose())*dqd_n-PIqd/pow(n.norm(),3)*dn_n*n.transpose()*dn_n; // checked
      Eigen::Matrix3d P_dqd_n_P_hvl = PIqd/n.norm()*P_dn_n_P_hvl - 1/n.norm()*(qd.transpose()*dn_n*I+qd*dn_n.transpose())*P_qd_P_hvl - PIqd/pow(n.norm(),3)*dn_n*n.transpose()*P_n_P_hvl; // checked


      // calculate dwd_n

      Eigen::Vector3d dwd_n = S(qd)*d_dqd_n_n + 2.0*kq/(hq*(1.0+hat_q.transpose()*qd))*(S(hat_q)*dqd_n-S(qd)*S(hat_w)*hat_q)-2.0*kq/(hq*pow((1+hat_q.transpose()*qd),2))*(qd.transpose()*S(hat_w)*hat_q+hat_q.transpose()*dqd_n)*S(hat_q)*qd+1.0/hq/(bar_mQ+bar_mL)*(n.transpose()/n.norm()*dn_n*S(hat_q)*gamma-n.norm()*S(gamma)*S(hat_w)*hat_q+beta*n.norm()*S(hat_q)*dsigma(e/ks1)*de_n+n.norm()*S(hat_q)*dzv_n); // checked
      Eigen::Matrix3d P_wd_P_hvl = S(qd)*P_dqd_n_P_hvl - S(dqd_n)*P_qd_P_hvl + 2.0*kq/hq/(1.0+hat_q.transpose()*qd)*(S(hat_q)*P_qd_P_hvl-S(hat_q)*qd*hat_q.transpose()*P_qd_P_hvl/(1.0+hat_q.transpose()*qd)) + 1.0/hq/(bar_mQ+bar_mL)*(1.0/n.norm()*S(hat_q)*gamma*n.transpose()*P_n_P_hvl + beta*n.norm()*S(hat_q)*dsigma(e/ks1)*P_e_P_hvl + n.norm()*S(hat_q)*P_zv_P_hvl); // checked

      // Now we can design PIq_Fd

      Eigen::Vector3d PIq_Fd = bar_mQ*hat_l*-PI_hat_q*(-(hat_bQ+hat_bL)/hat_l - S(hat_q)*dwd_n - S(hat_w-wd)*dhq_0 + kw/hw*zw + hq/hw*qd); 

      // Combine PqFd and PIq_Fd to get Fd

      Eigen::Vector3d Fd = PqFd + PIq_Fd;

      // Then we can design F

      double Td = Fd.norm();
      Eigen::Vector3d r3 = R*e3;
      Eigen::Vector3d PI_r3 = I - r3*r3.transpose();
      Eigen::Vector3d r3d = -Fd/Td;
      T = Td*r3d.transpose()*r3;
      Eigen::Vector3d F = Fd+Td*PI_r3*r3d;

      Eigen::Matrix3d P_V3_P_zv = P_V2_P_zv - hw*zw.transpose()*S(hat_q)*P_wd_P_hvl; // checked
      // Now we can get the equation of F
      // Then we can get dx_0 = dx_n + 1/(bar_mQ+bar_mL)*P_x_P_hvl*PqF_PqFd and dx = dx_0 + delta_dx

      Eigen::Matrix3d PqF_PqFd = P_hat_q*F-PqFd;


      Eigen::Vector3d dhw_0 = -S(hat_q)*F/bar_mQ/hat_l+S(hat_q)*(hat_bQ+hat_bL)/hat_l+2*S(hat_vL-hat_vQ)*(hat_pL-hat_pQ)*(hat_pL-hat_pQ).transpose()*(hat_vL-hat_vQ)/pow(hat_l,4); // checked


      Eigen::Vector3d dzv_0 = dzv_n+1.0/(bar_mQ+bar_mL)*P_zv_P_hvl*PqF_PqFd; // checked


      Eigen::Vector3d de_0 = k1*(dzp_0 + beta*dzv_0); // checked


      Eigen::Vector3d dgamma_0 = dgamma_n + 1.0/(bar_mQ+bar_mL)*P_gamma_P_hvl*PqF_PqFd; // checked


      Eigen::Vector3d dn_0 = dn_n + 1.0/(bar_mQ+bar_mL)*P_n_P_hvl*PqF_PqFd; // checked


      Eigen::Vector3d dqd_0 = dqd_n + 1.0/(bar_mQ+bar_mL)*P_qd_P_hvl*PqF_PqFd; // checked


      Eigen::Vector3d d_dzv_n_0 = d_dzv_n_n + 1.0/(bar_mQ+bar_mL)*P_dzv_n_P_hvl*PqF_PqFd; // checked


      Eigen::Vector3d d_de_n_0 = d_de_n_n + 1.0/(bar_mQ+bar_mL)*P_de_n_P_hvl*PqF_PqFd; // checked


      Eigen::Vector3d d_dgamma_n_0 = d_dgamma_n_n + 1.0/(bar_mQ+bar_mL)*P_dgamma_n_P_hvl*PqF_PqFd; // checked


      Eigen::Vector3d d_dn_n_0 = d_dn_n_n + 1.0/(bar_mQ+bar_mL)*P_dn_n_P_hvl*PqF_PqFd; // checked


      Eigen::Vector3d d_dqd_n_0 = d_dqd_n_n + 1.0/(bar_mQ+bar_mL)*P_dqd_n_P_hvl*PqF_PqFd; // checked


      Eigen::Vector3d dwd_0 = dwd_n + 1.0/(bar_mQ+bar_mL)*P_wd_P_hvl*PqF_PqFd; // checked


      Eigen::Vector3d dzw_n = -S(hat_w - wd)*dhq_0+ PI_hat_q*F/(bar_mQ*hat_l) - PI_hat_q*(hat_bQ+hat_bL)/hat_l+2.0*hat_q.transpose()*(hat_vL-hat_vQ)*PI_hat_q*(hat_vL-hat_vQ)/pow(hat_l,2) - S(hat_q)*dwd_n;
      Eigen::Matrix3d P_zw_P_hvl = -S(hat_q)*P_wd_P_hvl; // checked
      Eigen::Vector3d dzw_0 = dzw_n + 1.0/(bar_mQ+bar_mL)*P_zw_P_hvl*PqF_PqFd; // checked


      // calculate d_dhq_0_0

      Eigen::Vector3d d_dhq_0_0 = S(hat_w)*dhq_0 - S(hat_q)*dhw_0; // checked

      // calculate d_d_dzv_n_n_0
      Eigen::Vector3d d_d_dzv_n_n_0 = -1.0/(bar_mQ+bar_mL)*(d_dhq_0_0*hat_q.transpose()*n+dhq_0*dhq_0.transpose()*n+dhq_0*hat_q.transpose()*dn_0+dhq_0*dhq_0.transpose()*n+hat_q*d_dhq_0_0.transpose()*n+hat_q*dhq_0.transpose()*dn_0+dhq_0*hat_q.transpose()*dn_n+hat_q*dhq_0.transpose()*dn_n+hat_q*hat_q.transpose()*d_dn_n_0)-d4pd; // checked

      // calculate d_d_de_n_n_0
      Eigen::Vector3d d_d_de_n_n_0 = k1*(d_dzv_n_0 + beta*d_d_dzv_n_n_0); // checked

      // calculate d_d_dgamma_n_n_0
      Eigen::Vector3d d_d_dgamma_n_n_0 = beta/pow(ks1,2)*d3sigma(e/ks1)*diag(de_n)*diag(de_n)*de_0 + 2.0*beta/ks1*d2sigma(e/ks1)*diag(de_n)*d_de_n_0 + beta/ks1*d2sigma(e/ks1)*diag(d_de_n_n)*de_0 + beta*dsigma(e/ks1)*d_d_de_n_n_0 + d_d_dzv_n_n_0; // checked

      // calculate d_d_dn_n_n_0
      Eigen::Vector3d d_d_dn_n_n_0 = (bar_mQ+bar_mL)*(-d5pd+1.0/pow(ks1,2)*d3sigma(e/ks1)*diag(de_n)*diag(de_n)*de_0 + 2.0/ks1*d2sigma(e/ks1)*diag(de_n)*d_de_n_0 + 1/ks1*d2sigma(e/ks1)*diag(d_de_n_n)*de_0 + dsigma(e/ks1)*d_d_de_n_n_0 + k2/pow(ks2,2)*d3sigma(gamma/ks2)*diag(dgamma_n)*diag(dgamma_n)*dgamma_0 + 2*k2/ks2*d2sigma(gamma/ks2)*diag(dgamma_n)*d_dgamma_n_0 + k2/ks2*d2sigma(gamma/ks2)*diag(d_dgamma_n_n)*dgamma_0 + k2*dsigma(gamma/ks2)*d_d_dgamma_n_n_0); // checked


      // calculate d_d_dqd_n_n_0

      Eigen::Vector3d d_d_dqd_n_n_0 = -PIqd/pow(n.norm(),3)*(n.transpose()*dn_0)*d_dn_n_n-1/n.norm()*(dqd_0*qd.transpose()+qd*dqd_0.transpose())*d_dn_n_n+PIqd/n.norm()*d_d_dn_n_n_0+n.transpose()*dn_0/pow(n.norm(),3)*(qd.transpose()*dn_n*I+qd*dn_n.transpose())*dqd_n-1/n.norm()*(dqd_0.transpose()*dn_n*I+qd.transpose()*d_dn_n_0*I+dqd_0*dn_n.transpose()+qd*d_dn_n_0.transpose())*dqd_n-1/n.norm()*(qd.transpose()*dn_n*I+qd*dn_n.transpose())*d_dqd_n_0+3*PIqd/pow(n.norm(),5)*(n.transpose()*dn_0*n.transpose()*dn_n)*dn_n+1/pow(n.norm(),3)*(dqd_0*qd.transpose()+qd*dqd_0.transpose())*(n.transpose()*dn_n)*dn_n-PIqd/pow(n.norm(),3)*d_dn_n_0*n.transpose()*dn_n-PIqd/pow(n.norm(),3)*dn_n*dn_0.transpose()*dn_n-PIqd/pow(n.norm(),3)*dn_n*n.transpose()*d_dn_n_0; // checked

      // calculate d_dwd_n_0
      Eigen::Vector3d d_dwd_n_0 = S(dqd_0)*d_dqd_n_n + S(qd)*d_d_dqd_n_n_0 - 2*kq/hq/pow((1+hat_q.transpose()*qd),2)*(hat_q.transpose()*dqd_0+qd.transpose()*dhq_0)*(S(hat_q)*dqd_n-S(qd)*S(hat_w)*hat_q) \
          +2*kq/hq/(1+hat_q.transpose()*qd)*(S(dhq_0)*dqd_n+S(hat_q)*d_dqd_n_0-S(dqd_0)*S(hat_w)*hat_q-S(qd)*S(dhw_0)*hat_q-S(qd)*S(hat_w)*dhq_0) \
          +4*kq/hq/pow((1+hat_q.transpose()*qd),3)*(hat_q.transpose()*dqd_0+qd.transpose()*dhq_0)*(qd.transpose()*S(hat_w)*hat_q+hat_q.transpose()*dqd_n)*S(hat_q)*qd \
          -2*kq/hq/pow((1+hat_q.transpose()*qd),2)*(dqd_0.transpose()*S(hat_w)*hat_q+qd.transpose()*S(dhw_0)*hat_q+qd.transpose()*S(hat_w)*dhq_0+dhq_0.transpose()*dqd_n+hat_q.transpose()*d_dqd_n_0)*S(hat_q)*qd \
          -2*kq/hq/pow((1+hat_q.transpose()*qd),2)*(qd.transpose()*S(hat_w)*hat_q+hat_q.transpose()*dqd_n)*(S(dhq_0)*qd+S(hat_q)*dqd_0) \
          +1/hq/(bar_mQ+bar_mL)*(dn_n.transpose()*(I/n.norm()-n*n.transpose()/pow(n.norm(),3))*dn_0*S(hat_q)*gamma+n.transpose()/n.norm()*d_dn_n_0*S(hat_q)*gamma+n.transpose()/n.norm()*dn_n*S(dhq_0)*gamma+n.transpose()/n.norm()*dn_n*S(hat_q)*dgamma_0-n.transpose()*dn_0/n.norm()*S(gamma)*S(hat_w)*hat_q-n.norm()*S(dgamma_0)*S(hat_w)*hat_q-n.norm()*S(gamma)*S(dhw_0)*hat_q-n.norm()*S(gamma)*S(hat_w)*dhq_0+beta*n.transpose()*dn_0/n.norm()*S(hat_q)*dsigma(e/ks1)*de_n+beta*n.norm()*S(dhq_0)*dsigma(e/ks1)*de_n+beta/ks1*n.norm()*S(hat_q)*d2sigma(e/ks1)*diag(de_n)*de_0+beta*n.norm()*S(hat_q)*dsigma(e/ks1)*d_de_n_0+n.transpose()*dn_0/n.norm()*S(hat_q)*dzv_n+n.norm()*S(dhq_0)*dzv_n+n.norm()*S(hat_q)*d_dzv_n_0); // checked

      // calculate d_PqFd_0

      Eigen::Vector3d d_PqFd_0 = -(hat_q.transpose()*n*I+hat_q*n.transpose())*dhq_0-hat_q*hat_q.transpose()*dn_0 + 2*bar_mQ*l*hat_q*hat_w.transpose()*dhw_0+bar_mQ*l*hat_w.squaredNorm()*dhq_0+bar_mQ*hat_q.transpose()*(hat_bQ+hat_bL)*dhq_0+bar_mQ*hat_q*(hat_bQ+hat_bL).transpose()*dhq_0; // checked

      // calculate d_hat_l

      Eigen::Vector3d d_hat_l_0 = hat_q.transpose()*(dhpL_0 - dhpQ_0); // checked

      // calculate d_PIq_Fd_0

      Eigen::Vector3d d_PIq_Fd_0 = bar_mQ*(dhq_0*hat_q.transpose()+hat_q*dhq_0.transpose())*(-(hat_bQ+hat_bL)-hat_l*S(hat_q)*dwd_n+hat_l*S(dhq_0)*(hat_w-wd)+hq/hw*hat_l*qd+kw/hw*hat_l*zw)\
      +bar_mQ*(hat_q*hat_q.transpose()-I)*(-hat_l*S(dhq_0)*dwd_n-hat_l*S(hat_q)*d_dwd_n_0+hat_l*S(d_dhq_0_0)*(hat_w-wd)+hat_l*S(dhq_0)*(dhw_0-dwd_0)+hq/hw*hat_l*dqd_0+kw/hw*hat_l*dzw_0)\
      +bar_mQ*(hat_q*hat_q.transpose()-I)*d_hat_l_0*(-S(hat_q)*dwd_n+S(dhq_0)*(hat_w-wd)+hq/hw*qd+kw/hw*zw); // checked 

      // calculate d_r3d
      Eigen::Vector3d dr3d_0 = (r3d*r3d.transpose()-I)/Td*(d_PqFd_0+d_PIq_Fd_0); // checked

      // Now we can design Omega

      Eigen::Vector3d Omega = -S(e3)*S(e3)*(hw*Td/(hr*bar_mQ*hat_l)*S(e3)*R.transpose()*zw+Td/(hr*(bar_mQ+bar_mL))*S(e3)*R.transpose()*hat_q*hat_q.transpose()*P_V3_P_zv.transpose()+R.transpose()*S(r3d)*dr3d_0+2*kr/(hr*(1+r3.transpose()*r3d))*S(e3)*R.transpose()*r3d);
    }

    // 实现 update()，计算控制信号（此处仅为示例：返回目标航点与当前位置的差值组成的 1 维向量）
    virtual void update() override
    {
    }
  };

} // namespace controller

#endif // CONTROLLER_MYCONTROLLER_H
