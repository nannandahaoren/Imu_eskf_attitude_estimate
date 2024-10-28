#include <iostream>
#include "./IMU.h"
#include "./Serial.h"
#include "./Attitude.hpp"
#include "Socket.h"
#include "../Eigen/Dense"
#include <fstream>

using namespace std;
using namespace FDILink;
using namespace Eigen;


std::string CSV_PATH = "./data.csv";
ofstream csv;

//n ode ./socket_websocket.js 

int main(int argc, char const *argv[])
{

    Socket socket;
    socket.open();

    csv.open(CSV_PATH);

    Serial serial1;
    serial1.open();
    serial1.setConfig();

    IMU imu1(&serial1);

    bool iniFlag = true;
    int frame_num = 0;


    while (1)
    {
        
        Attitude<IMU> attitude(&imu1);
        imu1.readData();
        // imu1.printOriginData();

        imu1.generateData();
        // // imu1.printAcc();
        // // imu1.printProcessdata();

        attitude.format_data();

        // if (iniFlag)
        // {
        //     attitude.initAtt();
        //     iniFlag = false;

        // }
        // else
        // {

        //     attitude.predictAndUpdate();
        // }

        // Eigen::Vector3d imu_m;
        // imu_m = attitude.get_imu_m();

        // if (csv.is_open())
        // {
        //     frame_num++;
        //     if (frame_num == 1)
        //     {
        //         csv << "magX"
        //             << ","
        //             << "magY"
        //             << ","
        //             << "magZ"
        //             << "\n";
        //     }
        //     else
        //     {
        //         csv << imu_m[0] << "," << imu_m[1] << "," << imu_m[2] << "\n";
        //     }
        // }

        // if (frame_num % 10 == 0)
        // {
        //     std::cout << frame_num << ": " << imu_m[0]<< "," << imu_m[1]<< "," << imu_m[2]<< std::endl; 
        // }

        // if (frame_num > 10000) {
        //     csv.close();
        // }
        // attitude.write2imu();
        

        // Eigen::Vector3d euler;
        // euler = attitude.get_result();


        // // attitude.attUpdate();
        // // Eigen::Vector3d euler = attitude.getResult();
        
        // std::string sx = std::to_string(euler[0]);
        // std::string sy = std::to_string(euler[1]);
        // std::string sz = std::to_string(euler[2]);
        // // // std::string time1 = std::to_string(time);


        // // // // 向客户端发送数据
        // // // char str[] = "&10&20&30&";
        // std::string str = "&" + sx + "&" + sy + "&" + sz + "&";
        // socket.writeData(str.data(), str.length());

        // // attitude.show_result();


        // attitude.get_accelerate();// 
        // attitude.print_w_ie_n();
        // attitude.get_w_ie_n();
        // attitude.cal_Attitude();
        // attitude.print_eular_angle();

    }
    return 0;
};