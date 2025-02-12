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

    Params() = default;
    virtual ~Params() = default;

    // 从 ROS 参数服务器加载参数（纯虚函数）
    virtual bool loadFromRos(ros::NodeHandle &nh) = 0;

    // 从 JSON 文件加载参数（纯虚函数）
    virtual bool loadFromFile(const std::string &filename) = 0;
};

} // namespace common

#endif // COMMON_PARAMS_H
