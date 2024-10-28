#include <iostream>
#include <vector>
#include "time.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Convert.h"
#include "EKF_Attitude.h"
#include "Mahony_Attitude.h"
#include "ESKF_Attitude.h"
#include <iostream>
#include "./IMU.h"
#include "./Serial.h"
#include "./Attitude.hpp"
#include "Socket.h"
#include "../Eigen/Dense"
#include <fstream>




using namespace std;
using namespace IMU;


int main(int argc, char **argv)
{
	Eigen::MatrixXd data, measurements, groundtruth;
    data =IMU::readFromfile("./datasets/NAV2_data.bin");
    if(data.isZero())
        return 0;

    const int Rows = data.rows() - 1;
    measurements = data.block(0, 0, Rows, 9);
    groundtruth = data.block(0, 9, Rows, 3)*180/M_PI;


    Eigen::Matrix<double, 12, 1> ESKF_InitVec;
    ESKF_InitVec << 1e-5*Eigen::Vector3d::Ones(), 1e-9*Eigen::Vector3d::Ones(),
                    1e-3*Eigen::Vector3d::Ones(), 1e-4*Eigen::Vector3d::Ones();
    IMU::ESKF_Attitude ESKF_AHRS(ESKF_InitVec, 0.02);

	unsigned int i = 0;
	Eigen::MatrixXd Euler(measurements.rows(), 3), Euler1(measurements.rows(), 3), Euler2(measurements.rows(), 3);

    std::vector<double> Index, Roll, Pitch, Yaw, Roll_gt, Pitch_gt, Yaw_gt;
    std::vector<double> Roll1, Pitch1, Yaw1, Roll_gt1, Pitch_gt1, Yaw_gt1;
    std::vector<double> Roll2, Pitch2, Yaw2, Roll_gt2, Pitch_gt2, Yaw_gt2;
	TicToc tc;
	do
	{
		Eigen::MatrixXd measure;
		Eigen::Quaterniond quaternion;
		
		Vector_3 Euler_single;


        quaternion = ESKF_AHRS.Run(measurements.row(i).transpose());
        Euler2.row(i) = Quaternion_to_Euler(quaternion).transpose();


        Index.push_back(i*1.0);
        Roll.push_back(Euler.row(i)[0]);
        Pitch.push_back(Euler.row(i)[1]);
        Yaw.push_back(Euler.row(i)[2]);


        Roll1.push_back(Euler1.row(i)[0]);
        Pitch1.push_back(Euler1.row(i)[1]);
        Yaw1.push_back(Euler1.row(i)[2]);

        Roll2.push_back(Euler2.row(i)[0]);
        Pitch2.push_back(Euler2.row(i)[1]);
        Yaw2.push_back(Euler2.row(i)[2]);

        Roll_gt.push_back(groundtruth.row(i)[0]);
        Pitch_gt.push_back(groundtruth.row(i)[1]);
        Yaw_gt.push_back(groundtruth.row(i)[2]);


		i++;
	} while (i<measurements.rows());

	cout << tc.toc() << "ms" << endl;
	writeTofile(Euler, "Euler.bin");


	return 0;
}

