#include "../include/MY_IMU.h"

using namespace FDILink;
using namespace std::chrono;

float getPram32(int offset, uint8_t *data_buf)
{
    uint8_t d0 = data_buf[offset + 0];
    uint8_t d1 = data_buf[offset + 1];
    uint8_t d2 = data_buf[offset + 2];
    uint8_t d3 = data_buf[offset + 3];

    // float result = d3 << 24 + d2 << 16 + d1 << 8 + d0;
    // return result;

    long long transition_32;
    float tmp = 0;
    float last_tmp = 0;
    int sign = 0;
    int exponent = 0;
    float mantissa = 0;
    transition_32 = 0;
    transition_32 |= d3 << 24; // 得到数据的底 8 位
    transition_32 |= d2 << 16;
    transition_32 |= d1 << 8;
    transition_32 |= d3 << 0;                     // 得到数据的高 8 位
    sign = (transition_32 & 0x80000000) ? -1 : 1; // 符号位
    // 先右移操作，再按位与计算，出来结果是 30 到 23 位对应的 e
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    // 将 22~0 转化为 10 进制，得到对应的 x 系数
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    tmp = sign * mantissa * pow(2, exponent);
    last_tmp = tmp;
    return tmp;
}
MY_IMU::MY_IMU(){};

MY_IMU::MY_IMU(Serial *_serial)
{
    serial = _serial;
};

MY_IMU::~MY_IMU(){

};

void MY_IMU::readData(void)
{

    serial->flush();
    while (1)
    {

        // check head start  检查起始 数据帧头
        uint8_t check_head[1] = {0xff};

        // 传入端口号 和 check_head 以及 要读取的字节数
        // 会将读取结果放在 check_head 里面
        size_t head_s = serial->read(check_head, 1);
        if (head_s != 1)
        {
            printf("head read fail...\r\n");
            continue;
        }

        if (check_head[0] != FRAME_HEAD)
        {
            continue;
        }

        uint8_t head_type[1] = {0xff};
        size_t type_s = serial->read(head_type, 1);
        if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS && head_type[0] != TYPE_GEODETIC_POS && head_type[0] != 0x50 && head_type[0] != TYPE_GROUND && head_type[0] != 0xff)
        {
            continue;
        }
        uint8_t check_len[1] = {0xff};
        size_t len_s = serial->read(check_len, 1);

        // read head sn  检查sn 流水序号
        uint8_t check_sn[1] = {0xff};
        size_t sn_s = serial->read(check_sn, 1);

        uint8_t head_crc8[1] = {0xff};
        size_t crc8_s = serial->read(head_crc8, 1);

        uint8_t head_crc16_H[1] = {0xff};
        uint8_t head_crc16_L[1] = {0xff};
        size_t crc16_H_s = serial->read(head_crc16_H, 1);
        size_t crc16_L_s = serial->read(head_crc16_L, 1);
        //

        if (head_type[0] == TYPE_IMU)
        {

            // std::cout << "head_type: " << std::hex << (int)head_type[0] << std::dec << std::endl;

            uint8_t frame_header[4] = {
                check_head[0],
                head_type[0],
                check_len[0],
                check_sn[0],
            };

            uint8_t CRC8 = CRC8_Table(frame_header, 4);

            if (CRC8 != head_crc8[0])
            {
                // printf("header_crc8 error ;\r\n");
                continue;
            }

            uint16_t head_crc16_l = head_crc16_L[0];
            uint16_t head_crc16_h = head_crc16_H[0];
            uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);

            uint8_t data_buf[IMU_LEN + 1] = {0xff};
            size_t data_s = serial->read(data_buf, IMU_LEN + 1);

            if (data_s == -1)
            {
                printf("read data fail...\r\n");
                continue;
            }
            uint16_t CRC16 = CRC16_Table(data_buf, IMU_LEN);
            if (head_crc16 != CRC16)
            {
                // printf("check crc16 faild(imu).\r\n");
                continue;
            }
            if (isDebug)
            {
                std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
                std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
            }

            milliseconds ms;
            ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

            imu_origin_data.gyroscope_x = getPram32(0, data_buf);           // unit: rad/s
            imu_origin_data.gyroscope_y = getPram32(4, data_buf);           // unit: rad/s
            imu_origin_data.gyroscope_z = getPram32(8, data_buf);           // unit: rad/s
            imu_origin_data.accelerometer_x = getPram32(12, data_buf);      // unit: m/s2
            imu_origin_data.accelerometer_y = getPram32(16, data_buf);      // unit: m/s2
            imu_origin_data.accelerometer_z = getPram32(20, data_buf);      // unit: m/s2
            imu_origin_data.magnetometer_x = getPram32(24, data_buf);       // unit: mG 磁感应强度
            imu_origin_data.magnetometer_y = getPram32(28, data_buf);       // unit: mG 磁感应强度
            imu_origin_data.magnetometer_z = getPram32(32, data_buf);       // unit: mG 磁感应强度
            imu_origin_data.imu_temperature = getPram32(36, data_buf);      // unit: 平均温度
            imu_origin_data.Pressure = getPram32(40, data_buf);             // unit: 气压
            imu_origin_data.pressure_temperature = getPram32(44, data_buf); // unit: 气压计的温度值
            imu_origin_data.Timestamp = (long long)ms.count();

            Eigen::Vector3d acc;
            acc << imu_origin_data.accelerometer_x,
                imu_origin_data.accelerometer_y,
                imu_origin_data.accelerometer_z;

            Eigen::Vector3d gyr;

            gyr << imu_origin_data.gyroscope_x,
                    imu_origin_data.gyroscope_y,
                    imu_origin_data.gyroscope_z;

            Eigen::Vector3d mag;
            mag << imu_origin_data.magnetometer_x,
                    imu_origin_data.magnetometer_y,
                    imu_origin_data.magnetometer_z;
            

            // std::cout << "norm_a: " << norm_a.norm() << std::endl;

            MY_IMU::time[0] = MY_IMU::time[1];
            MY_IMU::time[1] = (long long)ms.count();

            if (MY_IMU::time[0] == 0)
            {
                MY_IMU::time[0] = MY_IMU::time[1];
            }
            measure_9 << acc,gyr,mag;
            cout << "deta_time = " << time[1] - time[0] << endl;
            break;
        }
    }
}


// void MY_IMU::generate_dtheta(void)
// {
//     IMUCalData temp;
//     temp.dt = ((double)time[1] - (double)time[0]) / 1000;

//     temp.dtheta << imu_origin_data.gyroscope_x * temp.dt,
//         imu_origin_data.gyroscope_y * temp.dt,
//         imu_origin_data.gyroscope_z * temp.dt;

//     temp.dvel << imu_origin_data.accelerometer_x * temp.dt,
//         imu_origin_data.accelerometer_y * temp.dt,
//         imu_origin_data.accelerometer_z * temp.dt;
//     // temp.isInit = true;

//     MY_IMU::IMUDataArray[0] = IMU::IMUDataArray[1];
//     MY_IMU::IMUDataArray[1] = temp;


// };


void MY_IMU::printOriginData(void)
{
    std::cout << " - IMU origin data: " << std::endl;
    // std::cout << "gyroscope_x:" << imu_origin_data.gyroscope_x << std::endl;
    // std::cout << "gyroscope_y:" << imu_origin_data.gyroscope_y << std::endl;
    // std::cout << "gyroscope_z:" << imu_origin_data.gyroscope_z << std::endl;
    std::cout << "accelerometer_x:" << imu_origin_data.accelerometer_x << std::endl;
    std::cout << "accelerometer_y:" << imu_origin_data.accelerometer_y << std::endl;
    std::cout << "accelerometer_z:" << imu_origin_data.accelerometer_z << std::endl;
    std::cout << "deta_t:" << time[1] -  time[0] << std::endl;
}

void MY_IMU::printAcc(void)
{
    std::cout << " - 加速度模长大小: " << std::endl;
    float g_x = imu_origin_data.accelerometer_x;
    float g_y = imu_origin_data.accelerometer_y;
    float g_z = imu_origin_data.accelerometer_z;

    double g_M = sqrt(g_x * g_x + g_y * g_y + g_z * g_z);

    std::cout << "三轴加速度模值:" << g_M << std::endl;

}
