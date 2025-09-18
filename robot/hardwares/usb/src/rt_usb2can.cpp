//
// Created by lingwei on 4/3/24.
//
#include "../include/rt_usb2can.h"
#include "../../utilities/types/std_cout_colors.h"
#include "../../utilities/inc/utilities_fun.h"
#include "iostream"
#include <cstdlib>
#include <mutex>
#include <thread>
#include <iox/mutex.hpp>

namespace USB_HARDWARE {
    Beast_USB2CAN::Beast_USB2CAN(uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin,
                                 uint8_t _motors_epout) : USB_Hardware_Base("Motor Control Board",
                                                                            vendor_id,
                                                                            product_id,
                                                                            _motors_epin,
                                                                            _motors_epout),
                                                          usb_cmd_LCM(getLcmUrl(255)),
                                                          usb_data_LCM(getLcmUrl(255)) {
        usb_cmd_u = new USB_Cmd_U();
        usb_data_u = new USB_Data_U();
        control_cmd_serial = new USB_Command_t();
        control_data_serial = new USB_Data_t();
        control_data_serial_offset = new USB_Data_t();
        control_cmd_serial_offset = new USB_Command_t();
        p_usbcmd_diff_lcmdata = new usb_command_t();
        p_usbdata_diff_lcmdata = new usb_data_t();
        p_usbcmd_serial_lcmdata = new usb_command_t();
        p_usbdata_serial_lcmdata = new usb_data_t();
    }

    Beast_USB2CAN::~Beast_USB2CAN() {
        delete control_cmd_serial;
        delete control_data_serial;
        delete control_cmd_serial_offset;
        delete control_data_serial_offset;

        delete p_usbcmd_diff_lcmdata;
        delete p_usbdata_diff_lcmdata;
        delete p_usbcmd_serial_lcmdata;
        delete p_usbdata_serial_lcmdata;
    }


    void Beast_USB2CAN::USB2CAN_SetBuffer(USB_Command_t *_control_cmd, USB_Data_t *_controller_data) {
        control_cmd_serial = _control_cmd;
        control_data_serial_offset = _controller_data;
    }

    void Beast_USB2CAN::motor_epin_callback(struct libusb_transfer *_transfer) {
        if (_transfer->status != LIBUSB_TRANSFER_COMPLETED) {
            std::cout << RED << "[USB2CAN ERROR]: " << RESET << "Motor Ep81 IN Error! Transfer again!\n";
        } else if (_transfer->status == LIBUSB_TRANSFER_COMPLETED) {
            this->Deal_Usb_In_Data();
            libusb_submit_transfer(_transfer);
        }
    }

    void Beast_USB2CAN::motor_epout_callback(struct libusb_transfer *_transfer) {
        if (_transfer->status != LIBUSB_TRANSFER_COMPLETED) {
            std::cout << RED << "[USB2CAN ERROR]: " << RESET << "Motor Ep01 OUT Error! Transfer again!\n";
        } else if (_transfer->status == LIBUSB_TRANSFER_COMPLETED) {
            this->Deal_Usb_Out_Cmd();
            libusb_submit_transfer(_transfer);
        }
    }

    /**
     * @brief usb data receiving handler
     */
    void Beast_USB2CAN::Deal_Usb_In_Data() {
        const uint32_t t = data_checksum((uint32_t *) usb_data_u, usb_motors_in_check_length);
        if (usb_data_u->usb_data_.checksum == t) {
            // 1. Make copy of raw data for LCM and processing
            memcpy(p_usbdata_diff_lcmdata, usb_data_u, sizeof(USB_Data_U));
            USB_Data_U raw_data;
            memcpy(&raw_data, usb_data_u, sizeof(USB_Data_U));

            // 2. Perform the specific four motor swaps (like old code)
            // Chip 0 motor 0 ↔ Chip 2 motor 0
            std::swap(raw_data.usb_data_.usb_chip_data_[0].data_pack[0],
                     raw_data.usb_data_.usb_chip_data_[2].data_pack[0]);
            
            // Chip 0 motor 3 ↔ Chip 2 motor 1
            std::swap(raw_data.usb_data_.usb_chip_data_[0].data_pack[3],
                     raw_data.usb_data_.usb_chip_data_[2].data_pack[1]);
            
            // Chip 1 motor 0 ↔ Chip 2 motor 3
            std::swap(raw_data.usb_data_.usb_chip_data_[1].data_pack[0],
                     raw_data.usb_data_.usb_chip_data_[2].data_pack[3]);
            
            // Chip 1 motor 3 ↔ Chip 2 motor 4
            std::swap(raw_data.usb_data_.usb_chip_data_[1].data_pack[3],
                     raw_data.usb_data_.usb_chip_data_[2].data_pack[4]);

            // memcpy(p_usbdata_diff_lcmdata, raw_data, sizeof(USB_Data_U));

            // 3. Convert differential to serial data
            for (int i = 0; i < 6 * (NUMBER_CHIPS - 1); i++) {
                const int chip_id = i / 6;
                const int chip_motor_id = i % 6;
                const int diff_convert = chip_motor_id % 3;
                const USB_CHIP_DATA_T *chip_data = &raw_data.usb_data_.usb_chip_data_[chip_id];

                switch (diff_convert) {
                    case 0:
                        control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].q =
                            (chip_data->data_pack[chip_motor_id].p_data_ - 
                             chip_data->data_pack[chip_motor_id + 1].p_data_) / 2.f;
                        control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].qd =
                            (chip_data->data_pack[chip_motor_id].v_data_ - 
                             chip_data->data_pack[chip_motor_id + 1].v_data_) / 2.f;
                        control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].tau =
                            chip_data->data_pack[chip_motor_id].t_data_ - 
                            chip_data->data_pack[chip_motor_id + 1].t_data_;
                        break;
                    case 1:
                        control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].q =
                            (chip_data->data_pack[chip_motor_id - 1].p_data_ + 
                             chip_data->data_pack[chip_motor_id].p_data_) / 2.f;
                        control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].qd =
                            (chip_data->data_pack[chip_motor_id - 1].v_data_ + 
                             chip_data->data_pack[chip_motor_id].v_data_) / 2.f;
                        control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].tau =
                            chip_data->data_pack[chip_motor_id - 1].t_data_ + 
                            chip_data->data_pack[chip_motor_id].t_data_;
                        break;
                    case 2:
                        control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].q = 
                            chip_data->data_pack[chip_motor_id].p_data_;
                        control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].qd = 
                            chip_data->data_pack[chip_motor_id].v_data_;
                        control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].tau = 
                            chip_data->data_pack[chip_motor_id].t_data_;
                        break;
                    default:
                        break;
                }
            }

           for (int i = 0; i < NUM_WHEEL_MOTORS; i++) {
                int chip_id = 2;
                int motor_id = i % 2 + (i / 2) * 3;
                control_data_serial->chip_datas[chip_id].motor_datas[motor_id].q = 
                        raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[motor_id].p_data_;
                control_data_serial->chip_datas[chip_id].motor_datas[motor_id].qd = 
                        raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[motor_id].v_data_;
                control_data_serial->chip_datas[chip_id].motor_datas[motor_id].tau = 
                        raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[motor_id].t_data_;    
            }


            // 4. Apply offsets and signs
            std::unique_lock<std::shared_mutex> lock(usb_shared_in_mutex);
            for (int i = 0; i < 6 * (NUMBER_CHIPS - 1); i++) {
                const int chip_id = i / 6;
                const int chip_motor_id = i % 6;
                
                control_data_serial_offset->chip_datas[chip_id].motor_datas[chip_motor_id].q =
                    (control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].q - leg_offset[i]) * leg_side_sign[i];
                control_data_serial_offset->chip_datas[chip_id].motor_datas[chip_motor_id].qd =
                    control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].qd * leg_side_sign[i];
                control_data_serial_offset->chip_datas[chip_id].motor_datas[chip_motor_id].tau =
                    control_data_serial->chip_datas[chip_id].motor_datas[chip_motor_id].tau / leg_side_sign[i];
                
                control_data_serial_offset->chip_datas[chip_id].motor_datas[chip_motor_id].uq = 
                    raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[chip_motor_id].uq_;
                control_data_serial_offset->chip_datas[chip_id].motor_datas[chip_motor_id].ud = 
                    raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[chip_motor_id].ud_;
                
            }

            // 4. Handle wheel motor data specifically (from old code)
            for (int i = 0; i < NUM_WHEEL_MOTORS; i++) {
                int chip_id = 2;
                int motor_id = i % 2 + (i / 2) * 3;
                // int data_index = NUM_LEG_MOTORS + i;
                
                control_data_serial_offset->chip_datas[chip_id].motor_datas[motor_id].q = 
                    raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[motor_id].p_data_ * whl_side_sign[i];
                control_data_serial_offset->chip_datas[chip_id].motor_datas[motor_id].qd = 
                        raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[motor_id].v_data_ * whl_side_sign[i];
                control_data_serial_offset->chip_datas[chip_id].motor_datas[motor_id].tau = 
                    raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[motor_id].t_data_ / whl_side_sign[i];
                control_data_serial_offset->chip_datas[chip_id].motor_datas[motor_id].uq = 
                    raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[motor_id].uq_;
                control_data_serial_offset->chip_datas[chip_id].motor_datas[motor_id].ud = 
                    raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[motor_id].ud_;
                    // std::cout<<raw_data.usb_data_.usb_chip_data_[chip_id].data_pack[motor_id].v_data_<<std::endl;
            }

            memcpy(p_usbdata_serial_lcmdata, control_data_serial_offset, sizeof(USB_Data_t));
            lock.unlock();
            
            p_usbdata_serial_lcmdata->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            usb_data_LCM.publish("MOTOR DATA Serial", p_usbdata_serial_lcmdata);
            usb_data_LCM.publish("MOTOR DATA Diff", p_usbdata_diff_lcmdata);
        } else {
            std::cout << BOLDRED << "[USB2CAN ERROR]: " << RESET << "usb data checksum error!\n";
        }
    }

    /**
     * @brief USB cmd out sending handler
     */
    void Beast_USB2CAN::Deal_Usb_Out_Cmd() {
        // offset first, then serial to diff
        std::shared_lock<std::shared_mutex> lock(usb_shared_out_mutex);
        for (int i = 0; i < 6 * (NUMBER_CHIPS - 1); i++) {
            const int chip_id = i / 6;
            const int chip_motor_id = i % 6;
            control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[chip_motor_id].q_des =
                    control_cmd_serial->chip_cmds[chip_id].motor_cmds[chip_motor_id].q_des / leg_side_sign[i] +
                    leg_offset[i];
            control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[chip_motor_id].qd_des =
                    control_cmd_serial->chip_cmds[chip_id].motor_cmds[chip_motor_id].qd_des / leg_side_sign[i];
            control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[chip_motor_id].kp =
                    control_cmd_serial->chip_cmds[chip_id].motor_cmds[chip_motor_id].kp;
            control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[chip_motor_id].kd =
                    control_cmd_serial->chip_cmds[chip_id].motor_cmds[chip_motor_id].kd;
            control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[chip_motor_id].tau_ff =
                    control_cmd_serial->chip_cmds[chip_id].motor_cmds[chip_motor_id].tau_ff * leg_side_sign[i];
        }

        // 2. Handle wheel motor commands specifically (from old code)
        for (int i = 0; i < NUM_WHEEL_MOTORS; i++) {
            int chip_id = 2;
            int motor_id = i % 2 + (i / 2) * 3;
            // int cmd_index = NUM_LEG_MOTORS + i;
            control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[motor_id].q_des = 
                control_cmd_serial->chip_cmds[chip_id].motor_cmds[motor_id].q_des / whl_side_sign[i];
            control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[motor_id].qd_des = 
                control_cmd_serial->chip_cmds[chip_id].motor_cmds[motor_id].qd_des / whl_side_sign[i];
            control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[motor_id].tau_ff = 
                control_cmd_serial->chip_cmds[chip_id].motor_cmds[motor_id].tau_ff * whl_side_sign[i];
            control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[motor_id].kp = 
                control_cmd_serial->chip_cmds[chip_id].motor_cmds[motor_id].kp;
            control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[motor_id].kd = 
                control_cmd_serial->chip_cmds[chip_id].motor_cmds[motor_id].kd;
        }

        usb_cmd_u->usb_cmd_.usb_chip_cmd_[0].chip_flag[0] = control_cmd_serial->chip_cmds[0].chip_flg;
        usb_cmd_u->usb_cmd_.usb_chip_cmd_[1].chip_flag[0] = control_cmd_serial->chip_cmds[1].chip_flg;
        usb_cmd_u->usb_cmd_.usb_chip_cmd_[2].chip_flag[0] = control_cmd_serial->chip_cmds[2].chip_flg;
        memcpy(p_usbcmd_serial_lcmdata, control_cmd_serial, sizeof(usb_command_t));
        lock.unlock();
        for (int i = 0; i < 6 * (NUMBER_CHIPS - 1); i++) {
            const int chip_id = i / 6;
            const int chip_motor_id = i % 6;
            const int diff_convert = chip_motor_id % 3;
            const CCC_t *chip_cmd = &control_cmd_serial_offset->chip_cmds[chip_id];

            switch (diff_convert) {
                case 0:
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].p_cmd_ =
                        chip_cmd->motor_cmds[chip_motor_id].q_des +
                        chip_cmd->motor_cmds[chip_motor_id + 1].q_des;
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].v_cmd_ =
                        chip_cmd->motor_cmds[chip_motor_id].qd_des +
                        chip_cmd->motor_cmds[chip_motor_id + 1].qd_des;
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].kp_ = 
                        chip_cmd->motor_cmds[chip_motor_id].kp / 2.f;
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].kd_ = 
                        chip_cmd->motor_cmds[chip_motor_id].kd / 2.f;
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].t_ff_ =
                        (chip_cmd->motor_cmds[chip_motor_id].tau_ff +
                         chip_cmd->motor_cmds[chip_motor_id + 1].tau_ff) / 2.f;
                    break;
                case 1:
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].p_cmd_ =
                        -(chip_cmd->motor_cmds[chip_motor_id - 1].q_des - 
                          chip_cmd->motor_cmds[chip_motor_id].q_des);
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].v_cmd_ =
                        -(chip_cmd->motor_cmds[chip_motor_id - 1].qd_des - 
                          chip_cmd->motor_cmds[chip_motor_id].qd_des);
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].kp_ = 
                        chip_cmd->motor_cmds[chip_motor_id].kp / 2.f;
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].kd_ = 
                        chip_cmd->motor_cmds[chip_motor_id].kd / 2.f;
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].t_ff_ =
                        -(chip_cmd->motor_cmds[chip_motor_id - 1].tau_ff - 
                          chip_cmd->motor_cmds[chip_motor_id].tau_ff) / 2.f;
                    break;
                case 2:
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].p_cmd_ =
                        chip_cmd->motor_cmds[chip_motor_id].q_des;
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].v_cmd_ =
                        chip_cmd->motor_cmds[chip_motor_id].qd_des;
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].kp_ =
                        chip_cmd->motor_cmds[chip_motor_id].kp / (TIMING_RATIO * TIMING_RATIO);
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].kd_ =
                        chip_cmd->motor_cmds[chip_motor_id].kd / (TIMING_RATIO * TIMING_RATIO);
                    usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[chip_motor_id].t_ff_ =
                        chip_cmd->motor_cmds[chip_motor_id].tau_ff;
                    break;
                default:
                    break;
            }
        }

        for (int i = 0; i < NUM_WHEEL_MOTORS; i++) {
            int chip_id = 2;
            int motor_id = i % 2 + (i / 2) * 3;
            // int cmd_index = NUM_LEG_MOTORS + i;
            usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[motor_id].p_cmd_ = 
                control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[motor_id].q_des;
            usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[motor_id].v_cmd_ = 
                control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[motor_id].qd_des;
            usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[motor_id].kp_ = 
                control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[motor_id].kp;
            usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[motor_id].kd_ = 
                control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[motor_id].kd;
            usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[motor_id].t_ff_ = 
                control_cmd_serial_offset->chip_cmds[chip_id].motor_cmds[motor_id].tau_ff;
            // usb_cmd_u->usb_cmd_.usb_chip_cmd_[chip_id].cmd_pack[motor_id].t_ff_ = 
            //         - 0.5 * control_data_serial->chip_datas[chip_id].motor_datas[motor_id].qd;
            // std::cout<<control_data_serial->chip_datas[chip_id].motor_datas[motor_id].qd<<std::endl;
        }
        // std::cout<<"  "<<std::endl;

        // 3. Perform the reverse swap for commands
        // Chip 0 motor 0 ↔ Chip 2 motor 0
        std::swap(usb_cmd_u->usb_cmd_.usb_chip_cmd_[0].cmd_pack[0],
                 usb_cmd_u->usb_cmd_.usb_chip_cmd_[2].cmd_pack[0]);
        
        // Chip 0 motor 3 ↔ Chip 2 motor 1
        std::swap(usb_cmd_u->usb_cmd_.usb_chip_cmd_[0].cmd_pack[3],
                 usb_cmd_u->usb_cmd_.usb_chip_cmd_[2].cmd_pack[1]);
        
        // Chip 1 motor 0 ↔ Chip 2 motor 3
        std::swap(usb_cmd_u->usb_cmd_.usb_chip_cmd_[1].cmd_pack[0],
                 usb_cmd_u->usb_cmd_.usb_chip_cmd_[2].cmd_pack[3]);
        
        // Chip 1 motor 3 ↔ Chip 2 motor 4
        std::swap(usb_cmd_u->usb_cmd_.usb_chip_cmd_[1].cmd_pack[3],
                 usb_cmd_u->usb_cmd_.usb_chip_cmd_[2].cmd_pack[4]);

        usb_cmd_u->usb_cmd_.checksum = data_checksum((uint32_t *) usb_cmd_u, usb_motors_out_check_length);
        memcpy(p_usbcmd_diff_lcmdata, usb_cmd_u, sizeof(usb_command_t));

        usb_cmd_LCM.publish("MOTOR COMMAND Diff", p_usbcmd_diff_lcmdata);
        usb_cmd_LCM.publish("MOTOR COMMAND Serial", p_usbcmd_serial_lcmdata);
    }

    void Beast_USB2CAN::start_transfer() {
        libusb_fill_interrupt_transfer(transfer_tx, deviceHandle, epout_,
                                       usb_cmd_u->usb_cmd_buff, usb_motors_out_length, usb_motors_out_cbf_wrapper, this,
                                       0);
        libusb_fill_interrupt_transfer(transfer_rx, deviceHandle, epin_, usb_data_u->usb_data_buff,
                                       usb_motors_in_length, usb_motors_in_cbf_wrapper, this, 0);
        libusb_submit_transfer(transfer_tx);
        libusb_submit_transfer(transfer_rx);
        if ((!transfer_tx->status) & (!transfer_rx->status)) {
            std::cout << GREEN << "[USB2CAN GOOD]: " << RESET << "All endpoints start transfering!\n";
        } else {
            std::cout << RED << "[USB2CAN ERROR]: " << RESET << "Some endpoints not work\n";
            exit(EXIT_FAILURE);
        }
    }

    void usb_motors_in_cbf_wrapper(struct libusb_transfer *_transfer) {
        auto *temp = reinterpret_cast<USB_HARDWARE::Beast_USB2CAN *>(_transfer->user_data);
        temp->motor_epin_callback(_transfer);
    }

    void usb_motors_out_cbf_wrapper(struct libusb_transfer *_transfer) {
        auto *temp = reinterpret_cast<USB_HARDWARE::Beast_USB2CAN *>(_transfer->user_data);
        temp->motor_epout_callback(_transfer);
    }
}
