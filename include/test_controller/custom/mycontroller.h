#ifndef CONTROLLER_MYCONTROLLER_H
#define CONTROLLER_MYCONTROLLER_H

#include "test_controller/controller/controller.h"
#include "test_controller/custom/myparams.h"
#include "test_controller/custom/mystate.h"
#include "test_controller/custom/mytrajectory.h"
#include <eigen3/Eigen/Dense>

namespace controller {

// 示例派生类 MyController，实现了 update() 函数
class MyController : public Controller {
public:
  common::MyParams &my_params;
  common::MyState &my_state;
  common::MyTrajectory &my_trajectory;
  MyController(common::MyParams &_params, common::MyState &_state, common::MyTrajectory &_trajectory)
  : Controller(_params, _state, _trajectory), my_params(_params), my_state(_state), my_trajectory(_trajectory) {}


  double clamp(double x, double min, double max) {
    return x < min ? min : (x > max ? max : x);
  }

  // 多项式计算
  double polyval(double x, const Eigen::VectorXd& p) {

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
  Eigen::Matrix3d S(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;
      mat <<  0,       -vec(2),  vec(1),
              vec(2),  0,       -vec(0),
            -vec(1),  vec(0),   0;
      return mat;
  }

  // 投影矩阵
  Eigen::Matrix3d PI(const Eigen::Vector3d& vec) {
      return Eigen::Matrix3d::Identity() - vec * vec.transpose();
  }

  virtual ~MyController() { }

  // 实现 update()，计算控制信号（此处仅为示例：返回目标航点与当前位置的差值组成的 1 维向量）
  virtual Eigen::VectorXd update() override {
    Eigen::VectorXd output(4);

    double kp = my_params.kp;
    double kv = my_params.kv;
    double kr = my_params.kr;
    double hr = my_params.hr;
    double mq = my_params.mq;
    double g = my_params.g;
    double use_polyval = my_params.use_polyval;
    Eigen::Vector3d p1 = my_params.p1;
    Eigen::Vector3d p2 = my_params.p2;
    Eigen::Vector3d p3 = my_params.p3;
    Eigen::Vector3d p4 = my_params.p4;
    Eigen::Vector3d e3(0, 0, 1);
    // 计算位置和速度误差
    double b =0.5;
    Eigen::Vector3d zp = my_state.p - my_trajectory.pd;
    double nzp = zp.norm();
    Eigen::Vector3d zv = my_state.vi - my_trajectory.dpd;
    double nzv = zv.norm();
    Eigen::Vector3d u = -kp * zp - kv * zv;
    Eigen::Vector3d Fd = mq * (u - g * e3 + my_trajectory.d2pd);
    // ROS_INFO("Fd: %f %f %f", Fd(0), Fd(1), Fd(2));
    double Td = Fd.norm();
    Eigen::Vector3d r3d = -Fd / Td;
    Eigen::Vector3d r3 = my_state.R*e3;
    double T = Td*r3d.dot(r3);
    // ROS_INFO("r3d: %f %f %f", r3d(0), r3d(1), r3d(2));
    Eigen::Vector3d F = -T*my_state.R*e3;
    Eigen::Vector3d dzv = F/mq + g*e3 - my_trajectory.d2pd;
    Eigen::Vector3d dFd = mq*(-kp*zv - kv*dzv + my_trajectory.d3pd);
    Eigen::Vector3d dr3d = S(r3d)*S(r3d)*dFd/Fd.norm();
    Eigen::Vector3d zr = r3 - r3d;
    Eigen::Vector3d omega;
    double yaw = my_state.euler(0);
    omega = - S(e3)*S(e3)*(my_state.R.transpose()*S(r3d)*dr3d + kr/hr*S(e3)*my_state.R.transpose()*r3d + Td/(mq*hr)*S(e3)*my_state.R.transpose()*(b*zp+zv)) - 0.1*(yaw-1.57)*e3;
    if(use_polyval) {
      output(0) = clamp(polyval(T*1000/9.8,p1),0,1);
      output(1) = clamp(sign(omega(0))*polyval(abs(omega(0)), p2),-3.14,3.14);
      output(2) = clamp(sign(omega(1))*polyval(abs(omega(1)), p3),-3.14,3.14);
      output(3) = clamp(0*sign(omega(2))*polyval(abs(omega(2)), p4),-3.14,3.14);
    }
    else {
      output(0) = T;
      output(1) = omega(0);
      output(2) = omega(1);
      output(3) = omega(2);
    }
    
    return output;
  }
};

} // namespace controller

#endif // CONTROLLER_MYCONTROLLER_H
