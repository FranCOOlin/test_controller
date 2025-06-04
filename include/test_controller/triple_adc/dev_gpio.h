#ifndef __DEV_GPIO_
#define __DEV_GPIO_


#include <stdio.h>
#include <ros/ros.h>
#define DEV_GPIO_INPUT   0
#define DEV_GPIO_OUTPUT 1

#define DEV_GPIO_LOW  0
#define DEV_GPIO_HIGH 1

#define DEV_GPIO_Debug(__info,...)  

// #ifdef RPI
// wiringPi GPIO
#define DEV_GPIO7 7 
#define DEV_GPIO0 0
#define DEV_GPIO2 2 
#define DEV_GPIO3 3
#define DEV_GPIO21 21
#define DEV_GPIO22 22
#define DEV_GPIO23 23 
#define DEV_GPIO24 24
#define DEV_GPIO25 25
#define DEV_GPIO1 1
#define DEV_GPIO4 4
#define DEV_GPIO5 5
#define DEV_GPIO6 6
#define DEV_GPIO26 26
#define DEV_GPIO27 27
#define DEV_GPIO28 28
#define DEV_GPIO29 29
#define DEV_SPI0_MOSI 12 
#define DEV_SPI0_MISO 13 
#define DEV_SPI0_SCK 14 
#define DEV_SPI0_CS0 10
#define DEV_SPI0_CS1 11
#define DEV_SDA0 30
#define DEV_SCL0 31
#define DEV_SDA1 8
#define DEV_SCL1 9
// #endif

// #ifdef OPI
// #define DEV_GPIO7 7 
// #define DEV_GPIO0 0
// #define DEV_GPIO2 2 
// #define DEV_GPIO3 3
// #define DEV_GPIO21 21
// #define DEV_GPIO22 22
// #define DEV_GPIO23 23 
// #define DEV_GPIO24 24
// #define DEV_GPIO25 25
// #define DEV_GPIO1 1
// #define DEV_GPIO4 4
// #define DEV_GPIO5 5
// #define DEV_GPIO6 6
// #define DEV_GPIO26 26
// #define DEV_GPIO27 27
// #define DEV_GPIO28 28
// #define DEV_GPIO29 29
// #define DEV_SPI0_MOSI 12 
// #define DEV_SPI0_MISO 13 
// #define DEV_SPI0_SCK 14 
// #define DEV_SPI0_CS0 10
// #define DEV_SPI0_CS1 11
// #define DEV_SDA0 30
// #define DEV_SCL0 31
// #define DEV_SDA1 8
// #define DEV_SCL1 9
// #endif



int DEV_GPIO_INIT(int pin, int direction, int init_val);
int DEV_GPIO_Read(int pin);
int DEV_GPIO_Write(int pin, int value);

#endif

