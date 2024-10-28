/*
 * 双矢量定姿态，此处是指重力矢量g^n和w_ie^n
 * 用到的参数，维度/经度/高度
 * 采用北东地坐标系
 */

#include <math.h>
#include <iostream>
#include "earth.h"
#include "../Eigen/Dense"
#include <iomanip>
#include <iostream>
#include <string>
#include "earth.h"
#include "angle.h"
#include "imu_types.h"
#include "rotation.h"
#include "IMU.h"
#include "SSEKF.h"


// 双矢量定姿 dvAtt （d-double，v-vector Att-Attitude）
// 注意，返回值是n系投降到b系，还是b系投影到n系

using namespace Eigen;
using namespace std;


class initAtt
{

public:


    // 输入原始imu的加速度计数据和原始imu的陀螺仪数据
    // 此处只读取了一次imu数据，应该多读几次，然后取平均值
    static Eigen::Quaterniond get_iniAtt(Eigen::Vector3d &imu_a, Eigen::Vector3d &imu_w)
    {


        double lat_rad = Angle::deg2rad(lat);
        double lon_rad = Angle::deg2rad(lon);
        Eigen::Vector3d blh;
        blh << lat_rad, lon_rad, alt;
        double g;
        g = Earth::gravity(blh);
        // std::cout << "wie_n" << wie_n << std::endl;
        Eigen::Vector3d g_n;
        g_n << 0, 0, g; // 北东地坐标系下，重力的坐标值

        Eigen::Vector3d wie_n;
        wie_n = Earth::iewn(lat_rad); // 地球自转角速度投影到n系

        // 当地水平坐标系相对于地心地固坐标系的牵连角速度，由运动产生，
        Eigen::Vector3d wen_n;
        wen_n << 0, 0, 0;

        Eigen::Vector3d v_g;
        v_g = g_n * (1.0 / g_n.norm());

        Eigen::Vector3d v_w = g_n.cross(wie_n); // g_n 叉乘 w_ie_n

        Eigen::Vector3d v_g_w;
        v_g_w = v_w.cross(g_n);

        v_w = v_w * (1.0 / v_w.norm());
        v_g_w = v_g_w * (1.0 / v_g_w.norm());

        Eigen::Vector3d w_g = imu_a * (1.0 / imu_a.norm());
        Eigen::Vector3d w_w = imu_a.cross(imu_w);
        Eigen::Vector3d w_g_w = w_w.cross(imu_a);

        w_w = w_w * (1.0 / w_w.norm());
        w_g_w = w_g_w * (1.0 / w_g_w.norm());

        Eigen::Matrix3d matrix_n;
        Eigen::Matrix3d matrix_b;

        matrix_n << v_g, v_w, v_g_w;
        matrix_b << w_g, w_w, w_g_w;

        Eigen::Matrix3d cbn;
        cbn = matrix_b * matrix_n.inverse();
        cbn.transpose(); // 单位正交矩阵

        Eigen::Quaterniond q;
        q = Rotation::matrix2quaternion(cbn);

        return q;
    }


    // 使用加速度计和磁力计 估算初始位姿
    // 加速度计进行归一化，磁力计使用原始数据 
    // 此处只读取了一次imu数据，应该多读几次，然后取平均值
    
    static Eigen::Vector4d cal_iniAtt(Eigen::Vector3d &norm_a, Eigen::Vector3d &imu_m,Eigen::Vector4d pre_P)
    {

        Eigen::Vector3d temp1, temp2,norm_m;

        norm_m = SSEKF::get_mag(imu_m,pre_P);
        temp1 = norm_a.cross(norm_m);
        temp2 = temp1.cross(norm_a);

        Eigen::Matrix3d initM;   // 初始姿态矩阵 init_att_matrix
        initM << temp2, temp1, norm_a;

        Eigen::Vector4d initq; // 初始姿态四元数
        initq[0] = sqrt(initM(0, 0) + initM(1, 1) + initM(2, 2) + 1) * 0.5;

        double x = initM(2, 1) - initM(1, 2);
        int a;
        a = (x < 0) ? -1 : (x > 0);
        initq[1] = a * 0.5 * sqrt(initM(0, 0) - initM(1, 1) - initM(2, 2) + 1);

        double y = initM(0, 2) - initM(2, 0);
        a = (y < 0) ? -1 : (y > 0);
        initq[2] = a * 0.5 * sqrt(initM(1, 1) - initM(2, 2) - initM(0, 0) + 1);

        double z = initM(1, 0) - initM(0, 1);
        a = (z < 0) ? -1 : (z > 0);
        initq[3] = a * 0.5 * sqrt(initM(2, 2) - initM(1, 1) - initM(0, 0) + 1);

        return initq;
        
    }


    

    // 四元数姿态更新算法，由当前四元数估计下一时刻的四元数
    // 参考武汉大学 惯导位姿解算，part2 15页
    // 此处的输入数据是积分后的角度增量，积分后的速度增量
    // 传入：，dt，pre_dtheta，cur_dtheta，pre_q，
    // dt 为imu经历时间间隔
    static Quaterniond updateAtt(Quaterniond pre_q,Vector3d wie_n,Vector3d wen_n,double dt,Vector3d pre_dtheta,Vector3d cur_dtheta)
    {

        Quaterniond cur_q, qne_mid, qnn, qbb;
        Vector3d temp1; // midpos, midvel;

        // 重新计算中间时刻地理参数
        // recompute rmrn, wie_n, wen_n at k-1/2

        // 计算n系的旋转四元数 k-1时刻到k时刻变换
        // n-frame rotation vector (n(k-1) with respect to n(k)-frame)
        // 计算 wie_n，wen_n
    
        

        temp1 = -(wie_n + wen_n) * dt; // 此处wen_n为0
        qnn = Rotation::rotvec2quaternion(temp1);

        // 计算b系旋转四元数 补偿二阶圆锥误差
        // b-frame rotation vector (b(k) with respect to b(k-1)-frame)
        // compensate the second-order coning correction term.

        temp1 = cur_dtheta + pre_dtheta.cross(cur_dtheta) / 12;
        qbb = Rotation::rotvec2quaternion(temp1);

        // 姿态更新完成
        // attitude update finish
        cur_q = qnn * pre_q * qbb;

        return cur_q;

    };


};
