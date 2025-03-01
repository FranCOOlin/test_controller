#include "test_controller/triple_adc/dev_gpio.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
// #include <ros/ros.h>

int DEV_GPIO_INIT(int pin, int direction, int init_val)
{
    // wiringPi只需初始化一次
    static bool initialized = false;
    if (!initialized) {
        if (wiringPiSetup() == -1) {
            ROS_ERROR("wiringPiSetup Fail");
            return -1;
        }
        ROS_INFO("wiringPiSetup Success");
        initialized = true;
    }

    // 根据传入的方向设置引脚模式
    if (direction == DEV_GPIO_OUTPUT) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, init_val ? HIGH : LOW);
        // ROS_INFO("引脚 %d 被初始化为输出模式，初始电平为 %d", pin, init_val);
    } else if (direction == DEV_GPIO_INPUT) {
        pinMode(pin, INPUT);
        // ROS_INFO("引脚 %d 被初始化为输入模式", pin);
    } else {
        // ROS_ERROR("无效的方向参数（%d）用于引脚 %d", direction, pin);
        return -1;
    }
    return 0;
}

int DEV_GPIO_Write(int pin, int value)
{
    digitalWrite(pin, value ? HIGH : LOW);
    // ROS_INFO("Pin %d is set to %d", pin, value);
    return 0;
}

int DEV_GPIO_Read(int pin)
{
    int val = digitalRead(pin);
    // ROS_INFO("Pin %d is %d", pin, val);
    return val;
}
