#ifndef CUSTOM_SYSTEM_PARAMS_H
#define CUSTOM_SYSTEM_PARAMS_H

#include "test_controller/common/params.h"
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace common {

class SystemParams : public Params {
public:
    // Global param
    double g;
    // For quadrotor controller of ganyu
    double quadrotor_mq, quadrotor_kp, quadrotor_kv, quadrotor_kr, quadrotor_hr;

    // For QSLS saturated backstepping controller
    double QSLS_bar_mQ;
    double QSLS_bar_mL;
    double QSLS_l;
    double QSLS_k1;
    double QSLS_beta;
    double QSLS_ks1;
    double QSLS_k2;
    double QSLS_ks2;
    double QSLS_hq;
    double QSLS_kq;
    double QSLS_hw;
    double QSLS_kw;
    double QSLS_hr;
    double QSLS_kr;

    // For signal generation
    bool use_polyval;
    Eigen::Vector3d p1, p2, p3, p4;

    virtual ~SystemParams() = default;

    // 从 ROS 参数服务器加载参数的实现
    virtual bool loadFromRos(ros::NodeHandle &nh) override {
        std::string uav_id;
        ros::param::get("~uav_id", uav_id);// 读取当前命名空间下的 uav_id
        if (uav_id.empty()) {
            ROS_ERROR("uav_id not set");
        }
        if (!nh.getParam(uav_id+"/controller/quadrotor/kp", quadrotor_kp)) {
            ROS_WARN("Failed to get parameter: /controller/quadrotor/kp");
        }
        if (!nh.getParam(uav_id+"/controller/quadrotor/kv", quadrotor_kv)) {
            ROS_WARN("Failed to get parameter: /controller/quadrotor/kv");
        }
        if (!nh.getParam(uav_id+"/controller/quadrotor/kr", quadrotor_kr)) {
            ROS_WARN("Failed to get parameter: /controller/quadrotor/kr");
        }
        if (!nh.getParam(uav_id+"/controller/quadrotor/hr", quadrotor_hr)) {
            ROS_WARN("Failed to get parameter: /controller/quadrotor/hr");
        }
        if (!nh.getParam(uav_id+"/controller/quadrotor/mq", quadrotor_mq)) {
            ROS_WARN("Failed to get parameter: /controller/quadrotor/mq");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/bar_mQ", QSLS_bar_mQ)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/bar_mQ");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/bar_mL", QSLS_bar_mL)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/bar_mL");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/l", QSLS_l)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/l");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/k1", QSLS_k1)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/k1");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/beta", QSLS_beta)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/beta");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/ks1", QSLS_ks1)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/ks1");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/k2", QSLS_k2)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/k2");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/ks2", QSLS_ks2)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/ks2");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/hq", QSLS_hq)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/hq");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/kq", QSLS_kq)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/kq");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/hw", QSLS_hw)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/hw");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/kw", QSLS_kw)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/kw");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/hr", QSLS_hr)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/hr");
        }
        if (!nh.getParam(uav_id+"/controller/QSLS/kr", QSLS_kr)) {
            ROS_WARN("Failed to get parameter: /controller/QSLS/kr");
        }
        if (!nh.getParam(uav_id+"/controller/g", g)) {
            ROS_WARN("Failed to get parameter: g");
        }
        if (!nh.getParam(uav_id+"/controller/use_polyval", use_polyval)) {
            ROS_WARN("Failed to get parameter: controller/use_polyval");
        }
        std::vector<double> temp;
        if (!nh.getParam(uav_id+"/controller/p1", temp)) {
            ROS_WARN("Failed to get parameter: controller/p1");
        }
        else{
            p1 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
        }
        if (!nh.getParam(uav_id+"controller/p2", temp)) {
            ROS_WARN("Failed to get parameter: controller/p2");
        }
        else{
            p2 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
        }
        if (!nh.getParam(uav_id+"controller/p3", temp)) {
            ROS_WARN("Failed to get parameter: controller/p3");
        }
        else{
            p3 = Eigen::Map<const Eigen::VectorXd>(temp.data(), temp.size());
        }
        if (!nh.getParam(uav_id+"controller/p4", temp)) {
            ROS_WARN("Failed to get parameter: controller/p4");
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
            quadrotor_mq = paramDict["controller"]["quadrotor"]["mq"].get<double>();
            quadrotor_kp = paramDict["controller"]["quadrotor"]["kp"].get<double>();
            quadrotor_kv = paramDict["controller"]["quadrotor"]["kv"].get<double>();
            quadrotor_kr = paramDict["controller"]["quadrotor"]["kr"].get<double>();
            quadrotor_hr = paramDict["controller"]["quadrotor"]["hr"].get<double>();
            QSLS_bar_mQ = paramDict["controller"]["QSLS"]["bar_mQ"].get<double>();
            QSLS_bar_mL = paramDict["controller"]["QSLS"]["bar_mL"].get<double>();
            QSLS_l = paramDict["controller"]["QSLS"]["l"].get<double>();
            QSLS_k1 = paramDict["controller"]["QSLS"]["k1"].get<double>();
            QSLS_beta = paramDict["controller"]["QSLS"]["beta"].get<double>();
            QSLS_ks1 = paramDict["controller"]["QSLS"]["ks1"].get<double>();
            QSLS_k2 = paramDict["controller"]["QSLS"]["k2"].get<double>();
            QSLS_ks2 = paramDict["controller"]["QSLS"]["ks2"].get<double>();
            QSLS_hq = paramDict["controller"]["QSLS"]["hq"].get<double>();
            QSLS_kq = paramDict["controller"]["QSLS"]["kq"].get<double>();
            QSLS_hw = paramDict["controller"]["QSLS"]["hw"].get<double>();
            QSLS_kw = paramDict["controller"]["QSLS"]["kw"].get<double>();
            QSLS_hr = paramDict["controller"]["QSLS"]["hr"].get<double>();
            QSLS_kr = paramDict["controller"]["QSLS"]["kr"].get<double>();
            g = paramDict["controller"]["g"].get<double>();
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

#endif // CUSTOM_SYSTEM_PARAMS_H
