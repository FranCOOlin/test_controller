#ifndef COMMON_CONVERT_H
#define COMMON_CONVERT_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace common {

// 将多个 Eigen::VectorXd 向量转换为 std_msgs::Float64MultiArray
inline std_msgs::Float64MultiArray vectorXdToFloat64MultiArray(const std::vector<Eigen::VectorXd>& vecs) {
    std_msgs::Float64MultiArray msg;
    for (const auto& vec : vecs) {
        msg.data.insert(msg.data.end(), vec.data(), vec.data() + vec.size());
    }
    return msg;
}

// 将多个 std_msgs::Float64MultiArray 转换为 Eigen::VectorXd
inline Eigen::VectorXd float64MultiArrayToVectorXd(const std::vector<std_msgs::Float64MultiArray>& msgs) {
    size_t total_size = 0;
    for (const auto& msg : msgs) {
        total_size += msg.data.size();
    }

    Eigen::VectorXd vec(total_size);
    size_t idx = 0;
    for (const auto& msg : msgs) {
        for (const auto& val : msg.data) {
            vec[idx++] = val;
        }
    }
    return vec;
}

// 将多个 std::vector<double> 转换为 Eigen::VectorXd
inline Eigen::VectorXd vectorToVectorXd(const std::vector<std::vector<double>>& vecs) {
    // 计算总大小
    size_t total_size = 0;
    for (const auto& vec : vecs) {
        total_size += vec.size();
    }

    Eigen::VectorXd eig_vec(total_size);
    size_t idx = 0;
    for (const auto& vec : vecs) {
        for (size_t i = 0; i < vec.size(); ++i) {
            eig_vec[idx++] = vec[i];
        }
    }
    return eig_vec;
}

// 将多个 Eigen::VectorXd 转换为 std::vector<double>
inline std::vector<double> vectorXdToVector(const std::vector<Eigen::VectorXd>& vecs) {
    std::vector<double> std_vec;
    for (const auto& vec : vecs) {
        std_vec.insert(std_vec.end(), vec.data(), vec.data() + vec.size());
    }
    return std_vec;
}

} // namespace common

#endif // COMMON_CONVERT_H
