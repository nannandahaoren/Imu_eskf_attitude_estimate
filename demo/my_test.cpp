#include <iostream>
#include <vector>
#include "time.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../include/Convert.h"
#include "../include/ESKF_Attitude.h"

#include "../include/MY_IMU.h"
#include "../include/Serial.h"
#include "../include/Socket.h"
#include <Eigen/Dense>
#include "../include/tic_toc.h"
#include <fstream>

using namespace std;
using namespace FDILink;
using namespace Eigen;


int main(int argc, char **argv)
{
    Socket socket;
    socket.open();

    Serial serial1;
    serial1.open();
    serial1.setConfig();

    MY_IMU imu1(&serial1);

    bool iniFlag = true;
    int frame_num = 0;


	IMU::Vector_9 measurements;
	IMU::Vector_3 Euler;


    Eigen::Matrix<double, 12, 1> ESKF_InitVec;
    ESKF_InitVec << 1e-5*Eigen::Vector3d::Ones(), 1e-6*Eigen::Vector3d::Ones(),
                    1e-6*Eigen::Vector3d::Ones(), 1e-6*Eigen::Vector3d::Ones();
    IMU::ESKF_Attitude ESKF_AHRS(ESKF_InitVec, 0.02);

	unsigned int i = 0;

	IMU::TicToc tc;
	while (1)
	{
		imu1.readData();
        measurements =imu1.measure_9;
        // cout << measurements << endl;


		Eigen::Quaterniond quaternion;
        double YAW;

        quaternion = ESKF_AHRS.Run(measurements);
        YAW = ESKF_AHRS.get_Yaw();
        // cout << "quaternion:" <<  quaternion << endl;

        Euler = IMU::Quaternion_to_Euler(quaternion).transpose();

        std::string sx = std::to_string(Euler[0]);
        std::string sy = std::to_string(Euler[1]);
        std::string sz = std::to_string(0);

        

        cout << "横滚角为：" <<  Euler[0] << endl;
        cout << "俯仰角为：" <<  Euler[1] << endl;
        cout << "偏航角为：" <<  (Euler[2] + YAW)/2.0<< endl;

        // cout << "measurements:" <<  measurements << endl;
        // char str[] = "&10&20&30&";
        std::string str = "&" + sx + "&" + sy + "&" + sz + "&";
        socket.writeData(str.data(), str.length());



		// i++;
	};

	// cout << tc.toc() << "ms" << endl;
	// writeTofile(Euler, "Euler.bin");

	return 0;
}

