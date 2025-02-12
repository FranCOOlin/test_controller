#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include <functional>
#include <memory>

typedef std::vector<double> state_type;

namespace common {

// 定义一个动力学方程的类型（dx/dt = f(x, t)）
using SystemDynamics = std::function<void(const state_type&, state_type&, double)>;

template <typename StepperType>
class Integrator {
public:
    // 构造函数，接收动力学方程和步长
    Integrator(const SystemDynamics& dynamics, double step_size)
        : dynamics_(dynamics), step_size_(step_size), stepper_() {}

    // 设置积分器的步长
    void set_step_size(double step_size) {
        step_size_ = step_size;
    }

    // 执行一步积分
    void step(state_type& state, double t_start, double step_size) {
        stepper_.do_step(dynamics_, state, t_start, step_size);  // 直接使用成员变量 stepper_
    }

    // 执行多步积分
    void integrate(state_type& state, double t_start, double t_end) {
        size_t steps = static_cast<size_t>((t_end - t_start) / step_size_);
        for (size_t i = 0; i < steps; ++i) {
            step(state, t_start + i * step_size_, step_size_);
        }
    }

private:
    SystemDynamics dynamics_;   // 动力学方程
    double step_size_;         // 积分步长
    StepperType stepper_;      // 步骤器，作为成员变量
};

}  // namespace common

#endif
