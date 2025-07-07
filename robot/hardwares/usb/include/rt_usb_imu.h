//
// Created by lingwei on 3/25/24.
//

#ifndef SIRIUS_SOFT_RT_IMU_H
#define SIRIUS_SOFT_RT_IMU_H

#include "sys/select.h"
#include "libusb-1.0/libusb.h"
#include <ctime>
#include <iostream>
#include <thread>
#include <mutex>
#include <exception>
#include "lcm/lcm-cpp.hpp"
#include "../../utilities/types/hardware_types.h"
#include "../../lcm-types/cpp/imu_lcmt.hpp"
#include "rt_usb_base.h"

namespace USB_HARDWARE {
    const uint16_t imu_in_length = 44;
    const uint16_t imu_out_length = 4;
    const uint16_t imu_in_check_length = imu_in_length / 4 - 1;
    const uint16_t imu_out_check_length = imu_out_length / 4 - 1;

    struct eigen_imu_data {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vec3<float> gyro_;
        Vec3<float> accel_;
        Vec3<float> temp_gyro_;
        Vec3<float> temp_accel_;
        RotMat<float> comp_Matrix_;
        RotMat<float> comp_raw_Matrix_;
        RotMat<float> origin_mat_;
        RotMat<float> first_mat_;
        Quat<float> quat_;
        Quat<float> temp_quat_;
    };

    typedef union {
        uint8_t buffer[imu_in_length];
        struct {
            float q[4];
            float gyro[3];
            float accel[3];
            uint32_t checksum;
        };
    } usb_imu_rx_data_u;

    class USB_IMU : public USB_Hardware_Base {
    public:
        USB_IMU(uint16_t _vendor_id, uint16_t _product_id, uint8_t _endpoint_in, uint8_t _endpoit_out);

        ~USB_IMU();

        void Deal_Out_Data();

        void Deal_In_Data();

        //void USB_Com_Start_Trans_Asy(void(*cbf_wrapper)(struct libusb_transfer *)) override;
        void start_transfer() override;

        void USB_In_CBF(struct libusb_transfer *transfer);

        void USB_Out_CBF(struct libusb_transfer *transfer);

        void Print_Rx_Data();

        void usb_imu_set_rx_buffer(USB_Imu_t *imu_data_);

        std::mutex imu_mtx;
        lcm::LCM imu_LCM;
        imu_lcmt *imu_lcm_data;

    private:
        USB_Imu_t *imu_data_buffer = nullptr;
        eigen_imu_data compensated_data_;
        usb_imu_rx_data_u *imu_data_;
        const int if_print = 0;
        bool first_run_ = true;
    };
}

void imu_cbf_wrapper(struct libusb_transfer *_transfer);

#endif //SIRIUS_SOFT_RT_IMU_H
