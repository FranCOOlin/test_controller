#ifndef CUSTOM_MY_MEASUREMENT_H
#define CUSTOM_MY_MEASUREMENT_H

#include "test_controller/common/measurement.h"
#include <eigen3/Eigen/Dense>

namespace common {

// 派生类 MyMeasurement，继承自 Measurement 类
class MyMeasurement : public Measurement {
public:
    Eigen::Vector3d p;       // 位置测量值（3D 向量）
    Eigen::Quaterniond attitude; // 姿态测量值（四元数）
    
    // 构造函数：初始化位置、姿态和更新标志
    MyMeasurement() : p(Eigen::Vector3d::Zero()), attitude(Eigen::Quaterniond::Identity()) {}

};

} // namespace common

#endif // COMMON_MY_MEASUREMENT_H
