/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Liqiang Wang
 *    Contact : wlq@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef KF_GINS_TYPES_H
#define KF_GINS_TYPES_H

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

#include "angle.h"

// 姿态
typedef struct Ex_Attitude {
    Eigen::Quaterniond qbn; // b系相对n系的四元数
    Eigen::Matrix3d cbn;    // b系相对n系的旋转矩阵
    Eigen::Vector3d euler;  // b系相对n系的欧拉角
} Ex_Attitude;

// 位置/速度/姿态
typedef struct PVA {
    Eigen::Vector3d pos; // 维度/经度/高度
    Eigen::Vector3d vel; // 北向速度 东向速度  垂向速度
    Ex_Attitude att;
} PVA;

typedef struct ImuError {
    Eigen::Vector3d gyrbias;  // 陀螺零偏
    Eigen::Vector3d accbias;  // 加速度计零偏
    Eigen::Vector3d gyrscale; // 陀螺比例因子
    Eigen::Vector3d accscale; // 加速度计比例因子
} ImuError;

typedef struct NavState {
    Eigen::Vector3d pos;   // 三维位置向量
    Eigen::Vector3d vel;   // 三维速度向量
    Eigen::Vector3d euler; // 三维位姿（欧拉角）向量

    ImuError imuerror; // IMU误差
} NavState;

typedef struct ImuNoise {
    Eigen::Vector3d gyr_arw;      // ARW（角度随机游走）
    Eigen::Vector3d acc_vrw;      // VRW（速度随机游走）
    Eigen::Vector3d gyrbias_std;  // ？陀螺零偏标准差
    Eigen::Vector3d accbias_std;  // ？加速度计零偏标准差
    Eigen::Vector3d gyrscale_std; // 陀螺比例因子标准差
    Eigen::Vector3d accscale_std; // ？加速度计比例因子标准差
    double corr_time;             // 相关时间？
} ImuNoise;

typedef struct GINSOptions {

    // 初始状态和状态标准差
    // initial state and state standard deviation
    NavState initstate;
    NavState initstate_std;

    // IMU噪声参数
    // imu noise parameters
    ImuNoise imunoise;

    // 安装参数
    // install parameters
    Eigen::Vector3d antlever = {0, 0, 0}; // 天线杆臂（相对加速度计中心的）坐标

    /**
     * @brief
     *
     */
    void print_options() {
        std::cout << "---------------KF-GINS Options:---------------" << std::endl;

        // 打印初始状态
        // print initial state
        std::cout << " - Initial State: " << std::endl;
        std::cout << '\t' << "- initial position: ";
        std::cout << std::setprecision(12) << initstate.pos[0] * R2D << "  ";
        std::cout << std::setprecision(12) << initstate.pos[1] * R2D << "  ";
        std::cout << std::setprecision(6) << initstate.pos[2] << " [deg, deg, m] " << std::endl;
        std::cout << '\t' << "- initial velocity: " << initstate.vel.transpose() << " [m/s] " << std::endl;
        std::cout << '\t' << "- initial attitude: " << initstate.euler.transpose() * R2D << " [deg] " << std::endl;
        std::cout << '\t' << "- initial gyrbias : " << initstate.imuerror.gyrbias.transpose() * R2D * 3600
                  << " [deg/h] " << std::endl;
        std::cout << '\t' << "- initial accbias : " << initstate.imuerror.accbias.transpose() * 1e5 << " [mGal] "
                  << std::endl;
        std::cout << '\t' << "- initial gyrscale: " << initstate.imuerror.gyrscale.transpose() * 1e6
                  << " [百万分之ppm] " << std::endl;
        std::cout << '\t' << "- initial accscale: " << initstate.imuerror.accscale.transpose() * 1e6
                  << " [百万分之ppm] " << std::endl;

        // 打印初始状态标准差
        // print initial state STD
        std::cout << " - Initial State STD: " << std::endl;
        std::cout << '\t' << "- initial position std: " << initstate_std.pos.transpose() << " [m] " << std::endl;
        std::cout << '\t' << "- initial velocity std: " << initstate_std.vel.transpose() << " [m/s] " << std::endl;
        std::cout << '\t' << "- initial attitude std: " << initstate_std.euler.transpose() * R2D << " [deg] "
                  << std::endl;
        std::cout << '\t' << "- initial gyrbias std: " << initstate_std.imuerror.gyrbias.transpose() * R2D * 3600
                  << " [deg/h] " << std::endl;
        std::cout << '\t' << "- initial accbias std: " << initstate_std.imuerror.accbias.transpose() * 1e5 << " [mGal] "
                  << std::endl;
        std::cout << '\t' << "- initial gyrscale std: " << initstate_std.imuerror.gyrscale.transpose() * 1e6
                  << " [ppm] " << std::endl;
        std::cout << '\t' << "- initial accscale std: " << initstate_std.imuerror.accscale.transpose() * 1e6
                  << " [ppm] " << std::endl;

        // 打印IMU噪声参数
        // print IMU noise parameters
        std::cout << " - IMU noise: " << std::endl;
        std::cout << '\t' << "- arw: " << imunoise.gyr_arw.transpose() * R2D * 60 << " [deg/s/sqrt(h)] " << std::endl;
        std::cout << '\t' << "- vrw: " << imunoise.acc_vrw.transpose() * 60 << " [m/s/sqrt(h)] " << std::endl;
        std::cout << '\t' << "- gyrbias  std: " << imunoise.gyrbias_std.transpose() * R2D * 3600 << " [deg/h] "
                  << std::endl;
        std::cout << '\t' << "- accbias  std: " << imunoise.accbias_std.transpose() * 1e5 << " [mGal] " << std::endl;
        std::cout << '\t' << "- gyrscale std: " << imunoise.gyrscale_std.transpose() * 1e6 << " [ppm] " << std::endl;
        std::cout << '\t' << "- accscale std: " << imunoise.accscale_std.transpose() * 1e6 << " [ppm] " << std::endl;
        std::cout << '\t' << "- correlation time: " << imunoise.corr_time / 3600.0 << " [h] " << std::endl;

        // 打印GNSS天线杆臂
        // print GNSS antenna leverarm
        std::cout << " - Antenna leverarm: " << antlever.transpose() << " [m] " << std::endl << std::endl;
    }

} GINSOptions;

#endif // KF_GINS_TYPES_H
