# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( IMU_Estimator )

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )

# 添加一个可执行程序
add_executable( my_test ./demo/my_test.cpp )

add_library( Socket ./src/Socket.cpp )
add_library( CRC ./src/crc_table.cpp )
add_library( MY_IMU ./src/MY_IMU.cpp )
add_library( Serial ./src/Serial.cpp )
add_library( Convert ./src/Convert.cpp )
add_library( ESKF_Attitude ./src/ESKF_Attitude.cpp )


# add_library( Attitude ./src/Attitude.hpp )

# target_link_libraries( IMU Attitude)

# target_link_libraries(${PROJECT_NAME}
#         ${GLOG_LIBRARIES}
#         ${EIGEN3_LIBS}
#         ${PYTHON_LIBRARIES})

target_link_libraries( MY_IMU CRC Serial)
target_link_libraries( ESKF_Attitude Convert)

target_link_libraries( my_test Serial)
target_link_libraries( my_test CRC)
target_link_libraries( my_test MY_IMU)
target_link_libraries( my_test Socket)
target_link_libraries( my_test ESKF_Attitude)
target_link_libraries( my_test Convert)






