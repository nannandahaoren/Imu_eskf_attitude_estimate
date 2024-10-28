#include <iostream>
#include "../Eigen/Dense"
#include "SSEKF.h"

// using namespace Eigen;
using namespace std;


int main(int argc, char const *argv[])
{
    // Vector3d norm_a;
    // norm_a << 1,2,3;

    // Vector3d norm_b;
    // norm_b << 4,5,6;

    // VectorXd aaa = EKF::get_zt(norm_a, norm_b);
    // cout<<aaa<<endl;

    // Quaterniond q;
    // q.x() = 0;
    // q.y() = 0;
    // q.z() = 0;
    // q.w() = 0;

    // Matrix<double, 6, 4> a = EKF::get_H(q);
    // while(1);
    // Matrix<double, 4, 4> P;
    // P.fill(3);
    // aaa(P);
    // cout<<P<<endl;

    Eigen::Vector4d qht;
    qht << 1,2,3,4;
 
    Eigen::Vector3d imu_m;
    imu_m << 1, 1, 1;



    Eigen::Matrix<double,4,3> m;

    // // m1 = SSEKF::get_zt(v1, v2);
    // m2.block(0,0,3,1) = v1;
    // m2.block(3,0,3,1) = v2;
    // m2 << v1,v2;
    m = SSEKF::get_Sq(qht);

    // cout << v1 << endl;

    // cout << m1 << endl;

    cout << m << endl;

}