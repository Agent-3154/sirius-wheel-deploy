//
// Created by lingwei on 4/3/24.
//
#include "../include/rt_usb_imu.h"
#include "cstring"
#include "../../utilities/inc/utilities_fun.h"
#include "../../utilities/types/std_cout_colors.h"

namespace USB_HARDWARE {
    USB_IMU::USB_IMU(uint16_t _vendor_id, uint16_t _product_id, uint8_t _endpoint_in, uint8_t _endpoit_out) : USB_Hardware_Base(
            "IMU", _vendor_id, _product_id, _endpoint_in, _endpoit_out), imu_LCM(getLcmUrl(255)) {
        imu_data_ = new usb_imu_rx_data_u();
        imu_lcm_data = new imu_lcmt();
        // z: -90 degree, y: 90 degree.
        // compensated_data_.comp_raw_Matrix_ << 0, 0, 1, 1, 0, 0, 0, 1, 0;

        // compensated_data_.comp_Matrix_ << 0, -1.0, 0, 1.0, 0.0, 0.0, 0, 0, 1.0;
    }

    USB_IMU::~USB_IMU() {
        delete imu_data_;
        delete imu_lcm_data;
    }

    //void N_Communication::USB_COM_IMU::USB_Com_Start_Trans_Asy(void (*cbf_wrapper)(struct libusb_transfer *)) {
    void USB_IMU::start_transfer() {
        libusb_fill_interrupt_transfer(transfer_rx, deviceHandle, epin_, imu_data_->buffer, imu_in_length,
                                       imu_cbf_wrapper, this, 0);
        libusb_submit_transfer(transfer_rx);
        if (transfer_rx->status == 0) {
            std::cout << GREEN << "[IMU GOOD]: " << RESET << "USB IMU Asynchronously Receiving!\n";
        } else {
            std::cout << RED << "[IMU ERROR]: " << RESET << "Can not start transmitting\n";
            std::abort();
        }
    }

    void USB_IMU::Deal_Out_Data() {
    }

    void USB_IMU::Deal_In_Data() {
        uint32_t t = data_checksum((uint32_t *) imu_data_->buffer, imu_in_check_length);
        if (imu_data_->checksum == t) {
            {
                std::lock_guard<std::mutex> lock(this->imu_mtx);

                // memcpy(imu_data_compensated, imu_data_->buffer, imu_in_length - 4);
                // add installation compensation

                // for (int i = 0; i < 3; i++) {
                //     compensated_data_.temp_gyro_(i) = imu_data_->gyro[i];
                //     compensated_data_.temp_accel_(i) = imu_data_->accel[i];
                //     compensated_data_.temp_quat_(i) = imu_data_->q[i];
                // }
                // compensated_data_.temp_quat_(3) = imu_data_->q[3];
                //
                // if (first_run_) {
                //     compensated_data_.first_mat_ = ori::quaternionToRotationMatrix(compensated_data_.temp_quat_);
                //     first_run_ = false;
                // }
                // compensated_data_.origin_mat_ = ori::quaternionToRotationMatrix(compensated_data_.temp_quat_);
                // compensated_data_.quat_ = ori::rotationMatrixToQuaternion(
                //     compensated_data_.comp_Matrix_ * compensated_data_.first_mat_.transpose() * compensated_data_.origin_mat_ *
                //     compensated_data_.comp_Matrix_.transpose());
                //
                // compensated_data_.accel_ = compensated_data_.comp_raw_Matrix_ * compensated_data_.temp_accel_;
                // compensated_data_.gyro_ = compensated_data_.comp_raw_Matrix_ * compensated_data_.temp_gyro_;
                //
                // for (int i = 0; i < 3; i++) {
                //     imu_data_buffer->gyro[i] = compensated_data_.gyro_(i);
                //     imu_data_buffer->accel[i] = compensated_data_.accel_(i);
                //     imu_data_buffer->q[i] = compensated_data_.quat_(i);
                // }
                // imu_data_buffer->q[3] = compensated_data_.quat_(3);
                memcpy(imu_data_buffer, imu_data_->buffer, sizeof(USB_Imu_t));

                memcpy(imu_lcm_data, imu_data_buffer, sizeof(USB_Imu_t));
                // timestamp: microseconds
                imu_lcm_data->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
            }
        } else {
            std::cout << "[ERROR] USB RX CMD CHECKSUM ERROR!\n";
        }
        imu_LCM.publish("IMU_CHANNEL", imu_lcm_data);
    }

    void USB_IMU::USB_In_CBF(struct libusb_transfer *transfer) {
        // std::cout << "Enter Real CBF!\n";
        if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
            std::cout << "[ERROR] Asy Trans Failed! Try again!\n";
            libusb_submit_transfer(transfer);
        } else if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
            this->Deal_In_Data();
            if (if_print)
                this->Print_Rx_Data();
            libusb_submit_transfer(transfer);
        }
    }

    void USB_IMU::USB_Out_CBF(struct libusb_transfer *transfer) {
        (void) transfer;
    }

    void USB_IMU::Print_Rx_Data() {
        std::cout.precision(3);
        std::cout << "[IMU " << product_id_ << "DATA] [" << imu_data_->q[0] << " " << imu_data_->q[1] << " "
                <<
                imu_data_->q[2] << " " << imu_data_->q[3] << "] [" << imu_data_->gyro[0] << " " <<
                imu_data_->gyro[1] << " " << imu_data_->gyro[2] << "] [" << imu_data_->accel[0] << " " <<
                imu_data_->accel[1] << " " << imu_data_->accel[2] << "]" << std::endl;
    }

    void USB_IMU::usb_imu_set_rx_buffer(USB_Imu_t *imu_data) {
        imu_data_buffer = imu_data;
    }
}

void imu_cbf_wrapper(struct libusb_transfer *_transfer) {
    //std::cout << "Enter wrapper function!!!!!!!!\n";
    auto *temp = reinterpret_cast<USB_HARDWARE::USB_IMU *>(_transfer->user_data);
    temp->USB_In_CBF(_transfer);
}
