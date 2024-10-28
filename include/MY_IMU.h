#ifndef MY_IMU_H_
#define MY_IMU_H_

#include "./Serial.h"
#include "./crc_table.h"
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <unistd.h>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


namespace FDILink
{
#define FRAME_HEAD 0xfc   // 用于指示数据帧的开始
#define FRAME_END 0xfd    // 用于指示数据帧的结束
#define TYPE_IMU 0x40     // 表示惯性测量单元 (IMU) 的数据类型
#define TYPE_AHRS 0x41    // 表示姿态和航向参考系统 (AHRS) 的数据类型
#define TYPE_INSGPS 0x42  // 表示惯导和全球定位系统 (INS/GPS) 的数据类型
#define TYPE_GEODETIC_POS 0x5c  // 可能表示大地测量位置 (Geodetic Position) 的数据类型
#define TYPE_GROUND 0xf0        // 可能表示地面数据的数据类型

#define IMU_LEN 0x38          // 0x38表示的值是56
#define AHRS_LEN 0x30         // 姿态和航向参考系统 (AHRS) 数据的长度为48
#define INSGPS_LEN 0x48       // 惯导和全球定位系统 (INS/GPS) 数据的长度为80
#define GEODETIC_POS_LEN 0x20 // 可能表示大地测量位置 (Geodetic Position) 数据的长度为32
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295 // 用于将角度值转换为弧度值的数学计算中



    struct ImuMeasureData
    {
        float gyroscope_x;          // unit: rad/s
        float gyroscope_y;          // unit: rad/s
        float gyroscope_z;          // unit: rad/s
        float accelerometer_x;      // m/s^2
        float accelerometer_y;      // m/s^2
        float accelerometer_z;      // m/s^2
        float magnetometer_x;       // mG
        float magnetometer_y;       // mG
        float magnetometer_z;       // mG
        float imu_temperature;      // C
        float Pressure;             // Pa
        float pressure_temperature; // C
        long long Timestamp;        // us
    };

     // 标准化的 加速度acceleration 和 磁场强度 magnetic field intensity
     // 无量纲


    // 姿态
    struct ImuAttitude
    {
        Eigen::Quaterniond qbn; // b系相对n系的四元数
        Eigen::Matrix3d cbn;    // b系相对n系的旋转矩阵
        Eigen::Vector3d euler;  // b系相对n系的欧拉角
        Eigen::Vector4d Vqbn;   // 将四元数转成向量
    };

    // // 位置/速度/姿态
    // struct IMU_PVA
    // {
    //     Eigen::Vector3d pos; // 维度/经度/高度
    //     Eigen::Vector3d vel; // 北向速度 东向速度  垂向速度
    //     IMU_Attitude att;
    // };

    // IMU结构体,IMU用于计算的数据
    struct IMUCalData
    {
        // bool isInit = true;
        double dt = 0.01; // 时间间隔
        Eigen::Vector3d dtheta; // 角度增量
        Eigen::Vector3d dvel;   // 速度增量
        // double odovel; // ？？？s
    };

    class MY_IMU
    {
    public:
        MY_IMU();
        MY_IMU(Serial *serial);

        ~MY_IMU();

        void readData(void);
        void printOriginData(void);
        void generateData(void);   // 输出角度增量，速度增量，dt-时间间隔
        void printAcc(void);


        ImuMeasureData imu_origin_data;   // IMU原始数据
        Eigen::Vector3d norm_acc,norm_mag;           // 归一化的三轴加速
        Eigen::Matrix<double, 9, 1> measure_9; //acc,gyr,mag

        // ImuAttitude preAtt;               // 上一时刻的四元数/旋转矩阵/欧拉角
        Eigen::Vector3d preBias;          // 上一时刻的陀螺仪偏置
        long long time[2] = {0, 0};       // 维护2个时刻的时间 是时刻
        IMUCalData IMUDataArray[2];       //imu 输出角度增量，速度增量，dt-时间间隔


    private:
        Serial *serial;
        bool isDebug = false;


    };

};
#endif