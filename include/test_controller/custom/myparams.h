#ifndef CUSTOM_MY_PARAMS_H
#define CUSTOM_MY_PARAMS_H

#include "test_controller/common/params.h"
#include <fstream>
#include <iostream>

namespace common {

class MyParams : public Params {
public:
    // 定义多个 double 类型的参数
    double mq, g, kp, kv, kr, hr;
    bool use_polyval ;
    Eigen::Vector3d p1, p2, p3, p4;
    // MyParams() //构造函数
    //   : controller_kp(0.0), controller_ki(0.0), controller_kd(0.0),
    //     observer_noise_std(0.0), traj_gen_speed(0.0), extra_param(0.0)
    // { }

    virtual ~MyParams() = default;

    // 从 ROS 参数服务器加载参数的实现
    virtual bool loadFromRos(ros::NodeHandle &nh) override {
        std::string uav_id;
        ros::param::get("~uav_id", uav_id);// 读取当前命名空间下的 uav_id
        if (uav_id.empty()) {
            ROS_ERROR("uav_id not set");
            return false;
        }
        if (!nh.getParam(uav_id+"controller/kp", kp)) {
            ROS_WARN("Failed to get parameter: controller/kp");
            return false;
        }
        if (!nh.getParam(uav_id+"controller/kv", kv)) {
            ROS_WARN("Failed to get parameter: controller/kv");
            return false;
        }
        if (!nh.getParam(uav_id+"controller/kr", kr)) {
            ROS_WARN("Failed to get parameter: controller/kr");
            return false;
        }
        if (!nh.getParam(uav_id+"controller/hr", hr)) {
            ROS_WARN("Failed to get parameter: controller/hr");
            return false;
        }
        if (!nh.getParam(uav_id+"controller/mq", mq)) {
            ROS_WARN("Failed to get parameter: controller/mq");
            return false;
        }
        if (!nh.getParam(uav_id+"controller/g", g)) {
            ROS_WARN("Failed to get parameter: controller/g");
            return false;
        }
        if (!nh.getParam(uav_id+"controller/use_polyval", use_polyval)) {
            ROS_WARN("Failed to get parameter: controller/use_polyval");
            return false;
        }
        std::vector<double> temp;
        if (!nh.getParam(uav_id+"controller/p1", temp)) {
            ROS_WARN("Failed to get parameter: controller/p1");
            return false;
        }
        else{
            p1 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
        }
        if (!nh.getParam(uav_id+"controller/p2", temp)) {
            ROS_WARN("Failed to get parameter: controller/p2");
            return false;
        }
        else{
            p2 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
        }
        if (!nh.getParam(uav_id+"controller/p3", temp)) {
            ROS_WARN("Failed to get parameter: controller/p3");
            return false;
        }
        else{
            p3 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
        }
        if (!nh.getParam(uav_id+"controller/p4", temp)) {
            ROS_WARN("Failed to get parameter: controller/p4");
            return false;
        }
        else{
            p4 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
        }

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
            mq = paramDict["controller"]["mq"].get<double>();
            g = paramDict["controller"]["g"].get<double>();
            kp = paramDict["controller"]["kp"].get<double>();
            kv = paramDict["controller"]["kv"].get<double>();
            kr = paramDict["controller"]["kr"].get<double>();
            hr = paramDict["controller"]["hr"].get<double>();
            use_polyval = paramDict["controller"]["use_polyval"].get<bool>();
            p1 = Eigen::Map<const Eigen::VectorXd>(paramDict["controller"]["p1"].get<std::vector<double>>().data(), paramDict["controller"]["p1"].size());
            p2 = Eigen::Map<const Eigen::VectorXd>(paramDict["controller"]["p2"].get<std::vector<double>>().data(), paramDict["controller"]["p2"].size());
            p3 = Eigen::Map<const Eigen::VectorXd>(paramDict["controller"]["p3"].get<std::vector<double>>().data(), paramDict["controller"]["p3"].size());
            p4 = Eigen::Map<const Eigen::VectorXd>(paramDict["controller"]["p4"].get<std::vector<double>>().data(), paramDict["controller"]["p4"].size());
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
