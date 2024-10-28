/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
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

#ifndef PRINT_H
#define PRINT_H

// #include "types.h"

#include "../Eigen/Geometry"
#include "../Eigen/Dense"
#include <iostream>
#include "rotation.h"

using namespace Eigen;
using namespace std;

class Print
{

public:
    /* 传进来四元数，打印四元数 */
    static void print_q(const Quaterniond &q)
    {
        // 传进来四元数
        std::cout << q.w() << ","
                  << q.x() << ","
                  << q.y() << ","
                  << q.z() << std::endl;
    }


    // 传进欧拉角，打印欧拉角
    static void print_euler(const Vector3d &euler)
    {
        // 旋转顺序是什么？ 
        std::cout << "欧拉角为：" 
                  << euler[0] << ","
                  << euler[1] << ","
                  << euler[2] << std::endl;
    }


    // 传进3维向量，打印向量
    static void print_vector(const Vector3d &V,string &S)
    {
        // 旋转顺序是什么？ 
        std::cout << S << ": " << std::endl;
        std::cout << V[0] << "," << V[1] << "," << V[2] << std::endl;
    }


};

#endif // PRINT_H
