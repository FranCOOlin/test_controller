#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <deque>

// CRC16 Modbus 校验
uint16_t crc16_modbus(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// 大端转小端函数（32-bit）
uint32_t swap_endian(uint32_t x) {
    return ((x >> 24) & 0x000000FF) | ((x >> 8) & 0x0000FF00) |
           ((x << 8) & 0x00FF0000) | ((x << 24) & 0xFF000000);
}

// 从大端字节流中提取浮动数 (32 位 float, 低 32 位)
float bytes_to_float(uint8_t *bytes) {
    uint32_t temp = 0;
    memcpy(&temp, bytes, sizeof(temp));
    
    // 如果数据是大端格式，进行字节顺序转换
    // temp = swap_endian(temp);  // 转换为小端格式
    
    float value;
    memcpy(&value, &temp, sizeof(value));
    return value;
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "adc_reader_node");
    ros::NodeHandle nh;

    // 创建 ROS 发布者
    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3Stamped>("triple_adc_value", 10);

    // 打开串口
    int fd;
    if ((fd = serialOpen("/dev/ttyAS3", 115200)) < 0) {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        return 1;
    }

    wiringPiSetup(); // 初始化 wiringPi
    pinMode(19, OUTPUT); // 设置 pin19 为输出模式
    digitalWrite(19, HIGH); // 初始化 pin19 为高电平
    std::deque<uint8_t> data_queue;


    digitalWrite(19, LOW); // 重启单片机
    delay(100); // 等待 100 毫秒
    digitalWrite(19, HIGH); // 重新设置 pin19 为高电平

    while (ros::ok()) {
        // 检查串口是否有数据可用
        if (serialDataAvail(fd)) {
            int c = serialGetchar(fd);
            if (c >= 0) {
                data_queue.push_back((uint8_t)c);
            }
        }

        // 滑动窗口解析 14 字节数据包
        while (data_queue.size() >= 14) {
            // 拷贝 14 字节窗口数据
            uint8_t buf[14];
            for (int i = 0; i < 14; ++i) {
                buf[i] = data_queue[i];
            }

            // CRC16 校验
            uint16_t crc = crc16_modbus(buf, 12);
            uint16_t crc_recv = buf[12] | (buf[13] << 8);

            if (crc == crc_recv) {
                // CRC 校验通过，解析数据（假设数据为 double 类型）
                // 由于 STM32 端是 64 位 double，我们只取低 32 位作为 float
                float v1 = bytes_to_float(&buf[0]);
                float v2 = bytes_to_float(&buf[4]);
                float v3 = bytes_to_float(&buf[8]);

                // 创建并发布消息
                geometry_msgs::Vector3Stamped msg;
                msg.header.stamp = ros::Time::now(); // 添加时间戳
                msg.vector.x = v1;
                msg.vector.y = v2;
                msg.vector.z = v3;
                pub.publish(msg);

                // ROS_INFO("Parsed: ADC1=%.6f, ADC2=%.6f, ADC3=%.6f", v1, v2, v3);

                // 弹出已解析的 14 字节
                for (int i = 0; i < 14; ++i) {
                    data_queue.pop_front();
                }
            } else {
                // CRC 校验失败，丢弃首字节
                data_queue.pop_front();
                ROS_WARN("CRC check failed, discarding first byte");
            }

            // 可选：限制队列长度防止内存增长
            if (data_queue.size() > 512) {
                data_queue.clear();
                ROS_WARN("Queue cleared due to overflow");
            }
        }

        // 调用 ros::spinOnce() 来处理回调函数
        ros::spinOnce();
    }

    return 0;
}
