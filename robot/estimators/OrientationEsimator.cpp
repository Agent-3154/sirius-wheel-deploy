//
// Created by lingwei on 4/24/24.
//
/*! @file OrientationEstimator.cpp
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */

#include "OrientationEstimator.h"
#include "../../utilities/inc/utilities_fun.h"

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
namespace Estimators {
    template<typename T>
    void UsbImuOrientationEstimator<T>::run() {
//wxyz
        this->stateEstimateData_.result_->q_ori_[0] = this->stateEstimateData_.usb_imu_data_->q[0];
        this->stateEstimateData_.result_->q_ori_[1] = this->stateEstimateData_.usb_imu_data_->q[1];
        this->stateEstimateData_.result_->q_ori_[2] = this->stateEstimateData_.usb_imu_data_->q[2];
        this->stateEstimateData_.result_->q_ori_[3] = this->stateEstimateData_.usb_imu_data_->q[3];

        //yaw init
        if (first_vist_) {
            Vec3<T> rpy_ini = ori::quatToRPY(this->stateEstimateData_.result_->q_ori_);
            rpy_ini[0] = 0;
            rpy_ini[1] = 0;
            _ori_ini_inv = ori::rpyToQuat(-rpy_ini); //TODO 这里的负号？
            first_vist_ = false;
        }

        this->stateEstimateData_.result_->q_ori_ = ori::quatProduct(_ori_ini_inv, this->stateEstimateData_.result_->q_ori_);

        this->stateEstimateData_.result_->rpy_ = ori::quatToRPY(this->stateEstimateData_.result_->q_ori_);

        this->stateEstimateData_.result_->r_b_ = ori::quaternionToRotationMatrix(this->stateEstimateData_.result_->q_ori_);

        this->stateEstimateData_.result_->omega_b_(0) =
                this->stateEstimateData_.usb_imu_data_->gyro[0];
        this->stateEstimateData_.result_->omega_b_(1) =
                this->stateEstimateData_.usb_imu_data_->gyro[1];
        this->stateEstimateData_.result_->omega_b_(2) =
                this->stateEstimateData_.usb_imu_data_->gyro[2];

        this->stateEstimateData_.result_->omega_w_ =
                this->stateEstimateData_.result_->r_b_.transpose() *
                this->stateEstimateData_.result_->omega_b_;

        this->stateEstimateData_.result_->a_b_(0) =
                this->stateEstimateData_.usb_imu_data_->accel[0];
        this->stateEstimateData_.result_->a_b_(1) =
                this->stateEstimateData_.usb_imu_data_->accel[1];
        this->stateEstimateData_.result_->a_b_(2) =
                this->stateEstimateData_.usb_imu_data_->accel[2];

        this->stateEstimateData_.result_->a_w_ =
                this->stateEstimateData_.result_->r_b_.transpose() *
                this->stateEstimateData_.result_->a_b_;
    }

    template
    class UsbImuOrientationEstimator<double>;
}
