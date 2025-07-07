//
// Created by lingwei on 4/3/24.
//

#ifndef MY_MUJOCO_SIMULATOR_HARDWARE_TYPES_H
#define MY_MUJOCO_SIMULATOR_HARDWARE_TYPES_H

#include <cstdint>
#include <vector>
#include <eigen3/Eigen/Dense>

typedef struct Controller_M_Cmd {
    float q_des;
    float qd_des;
    float kp;
    float kd;
    float tau_ff;
} CMCmd_t;

typedef struct Control_Chip_Cmd {
    CMCmd_t motor_cmds[6];
    int32_t chip_flg;
}CCC_t;

typedef struct USBCommand {
    CCC_t chip_cmds[3];
} USB_Command_t;

typedef struct Controller_M_Data {
    float q;
    float qd;
    float tau;
    float uq;
    float ud;
} CMData_t;

typedef struct Controller_Chip_Data {
    CMData_t motor_datas[6];
    int32_t chip_flg;
} CCData_t;

typedef struct USBData {
    CCData_t chip_datas[3];
} USB_Data_t;

typedef struct IMUData {
    float q[4];
    float gyro[3];
    float accel[3];
} USB_Imu_t;

// Rotation Matrix
template<typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 2x1 Vector
template<typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template<typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template<typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

// 6x1 Vector
template<typename T>
using Vec6 = Eigen::Matrix<T, 6, 1>;

// 10x1 Vector
template<typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

// 12x1 Vector
template<typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

// 16x1 Vector
template<typename T>
using Vec16 = Eigen::Matrix<T, 16, 1>;

// 18x1 Vector
template<typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

template<typename T>
using Vec19 = Eigen::Matrix<T, 19, 1>;

template<typename T>
using Vec22 = Eigen::Matrix<T, 22, 1>;

template<typename T>
using Vec23 = Eigen::Matrix<T, 23, 1>;

// 28x1 vector
template<typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

// 3x3 Matrix
template<typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 4x1 Vector
template<typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// Spatial Vector (6x1, all subspaces)
template<typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// Spatial Transform (6x6)
template<typename T>
using SXform = typename Eigen::Matrix<T, 6, 6>;

// 6x6 Matrix
template<typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 12x12 Matrix
template<typename T>
using Mat12 = typename Eigen::Matrix<T, 12, 12>;

// 18x18 Matrix
template<typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

// 28x28 Matrix
template<typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

// 3x4 Matrix
template<typename T>
using Mat34 = Eigen::Matrix<T, 3, 4>;

// 3x4 Matrix
template<typename T>
using Mat23 = Eigen::Matrix<T, 2, 3>;

// 4x4 Matrix
template<typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

// 10x1 Vector
template<typename T>
using MassProperties = typename Eigen::Matrix<T, 10, 1>;

// Dynamically sized vector
template<typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Dynamically sized matrix
template<typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// Dynamically sized matrix with spatial vector columns
template<typename T>
using D6Mat = typename Eigen::Matrix<T, 6, Eigen::Dynamic>;

// Dynamically sized matrix with cartesian vector columns
template<typename T>
using D3Mat = typename Eigen::Matrix<T, 3, Eigen::Dynamic>;

template<typename T>
using D3Mat18 = typename Eigen::Matrix<T, 3, 18>;
// std::vector (a list) of Eigen things
template<typename T>
using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;
#endif //MY_MUJOCO_SIMULATOR_HARDWARE_TYPES_H
