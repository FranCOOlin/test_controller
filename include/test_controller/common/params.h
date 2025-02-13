#ifndef COMMON_PARAMS_H
#define COMMON_PARAMS_H

#include <ros/ros.h>
#include <string>
#include "nlohmann/json.hpp"  

namespace common {

class Params {
public:
    // 内部存储参数的 JSON 字典
    nlohmann::json paramDict;

    // 从 JSON 中提取向量的函数
    Eigen::VectorXd getVectorFromJson(const nlohmann::json &j, const std::string &key) {
        if (j.contains(key)) {
            std::vector<double> vector_data = j[key];
            Eigen::VectorXd vec(vector_data.size());
            for (size_t i = 0; i < vector_data.size(); ++i) {
                vec(i) = vector_data[i];
            }
            return vec;
        } else {
            std::cerr << "Key '" << key << "' not found in JSON!" << std::endl;
            return Eigen::VectorXd();  // 返回一个空向量
        }
    }

    // 从 JSON 中提取矩阵的函数
    Eigen::MatrixXd getMatrixFromJson(const nlohmann::json &j, const std::string &key) {
        if (j.contains(key)) {
            std::vector<std::vector<double>> matrix_data = j[key];
            size_t rows = matrix_data.size();
            size_t cols = matrix_data[0].size();
            Eigen::MatrixXd mat(rows, cols);
            for (size_t i = 0; i < rows; ++i) {
                for (size_t j = 0; j < cols; ++j) {
                    mat(i, j) = matrix_data[i][j];
                }
            }
            return mat;
        } else {
            std::cerr << "Key '" << key << "' not found in JSON!" << std::endl;
            return Eigen::MatrixXd();  // 返回一个空矩阵
        }
    }

    Params() = default;
    virtual ~Params() = default;

    // 从 ROS 参数服务器加载参数（纯虚函数）
    virtual bool loadFromRos(ros::NodeHandle &nh) = 0;

    // 从 JSON 文件加载参数（纯虚函数）
    virtual bool loadFromFile(const std::string &filename) = 0;
};

} // namespace common

#endif // COMMON_PARAMS_H
