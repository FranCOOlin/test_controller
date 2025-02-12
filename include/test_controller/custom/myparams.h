#ifndef CUSTOM_MY_PARAMS_H
#define CUSTOM_MY_PARAMS_H

#include "test_controller/common/params.h"
#include <fstream>
#include <iostream>

namespace common {

class MyParams : public Params {
public:
    // 定义多个 double 类型的参数
    double controller_kp;
    double controller_ki;
    double controller_kd;
    double observer_noise_std;
    double traj_gen_speed;
    double extra_param;  // 示例：可根据需要添加更多参数

    MyParams() 
      : controller_kp(0.0), controller_ki(0.0), controller_kd(0.0),
        observer_noise_std(0.0), traj_gen_speed(0.0), extra_param(0.0)
    { }

    virtual ~MyParams() = default;

    // 从 ROS 参数服务器加载参数的实现
    virtual bool loadFromRos(ros::NodeHandle &nh) override {
        if (!nh.getParam("controller/kp", controller_kp)) {
            ROS_WARN("Failed to get parameter: controller/kp");
            return false;
        }
        if (!nh.getParam("controller/ki", controller_ki)) {
            ROS_WARN("Failed to get parameter: controller/ki");
            return false;
        }
        if (!nh.getParam("controller/kd", controller_kd)) {
            ROS_WARN("Failed to get parameter: controller/kd");
            return false;
        }
        if (!nh.getParam("observer/noise_std", observer_noise_std)) {
            ROS_WARN("Failed to get parameter: observer/noise_std");
            return false;
        }
        if (!nh.getParam("traj_gen/speed", traj_gen_speed)) {
            ROS_WARN("Failed to get parameter: traj_gen/speed");
            return false;
        }
        if (!nh.getParam("extra_param", extra_param)) {
            ROS_WARN("Failed to get parameter: extra_param");
            return false;
        }
        // 存储到 JSON 字典中
        paramDict["controller"]["kp"] = controller_kp;
        paramDict["controller"]["ki"] = controller_ki;
        paramDict["controller"]["kd"] = controller_kd;
        paramDict["observer"]["noise_std"] = observer_noise_std;
        paramDict["traj_gen"]["speed"] = traj_gen_speed;
        paramDict["extra_param"] = extra_param;

        ROS_INFO("Loaded parameters from ROS parameter server.");
        return true;
    }

    // 从 JSON 文件加载参数的实现
    virtual bool loadFromFile(const std::string &filename) override {
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open parameter file: %s", filename.c_str());
            return false;
        }
        try {
            file >> paramDict;
        } catch (const std::exception &e) {
            ROS_ERROR("Exception while reading JSON file: %s", e.what());
            return false;
        }
        // 从 JSON 中读取各个参数
        try {
            controller_kp = paramDict["controller"]["kp"].get<double>();
            controller_ki = paramDict["controller"]["ki"].get<double>();
            controller_kd = paramDict["controller"]["kd"].get<double>();
            observer_noise_std = paramDict["observer"]["noise_std"].get<double>();
            traj_gen_speed = paramDict["traj_gen"]["speed"].get<double>();
            extra_param = paramDict["extra_param"].get<double>();
        } catch (const std::exception &e) {
            ROS_ERROR("Exception while parsing parameters from JSON: %s", e.what());
            return false;
        }
        ROS_INFO("Loaded parameters from file: %s", filename.c_str());
        return true;
    }
};

} // namespace common

#endif // CUSTOM_MY_PARAMS_H
