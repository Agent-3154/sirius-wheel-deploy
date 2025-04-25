//
// Created by lingwei on 4/3/24.
//
#include "../include/rt_usb2can.h"
#include "../../utilities/types/std_cout_colors.h"
#include "../../utilities/inc/utilities_fun.h"
#include "iostream"
#include <cstdlib>
#include <thread>

namespace USB_HARDWARE {

    Motor_Control_Board::Motor_Control_Board(uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin,
                                             uint8_t _motors_epout) : USB_Hardware_Base("Motor Control Board",
                                                                                        vendor_id,
                                                                                        product_id,
                                                                                        _motors_epin,
                                                                                        _motors_epout),
                                                                      usb_cmd_LCM(getLcmUrl(255)),
                                                                      usb_data_LCM(getLcmUrl(255)) {
        usb_cmd_u = new USB_Cmd_U();
        usb_data_u = new USB_Data_U();
        control_cmd = new USB_Command_t();
        control_data = new USB_Data_t();
        p_usbcmd_lcm = new usb_command_t();
        p_usbdata_lcm = new usb_data_t();
    }

    Motor_Control_Board::~Motor_Control_Board() {
        delete usb_cmd_u;
        delete usb_data_u;
        delete control_cmd;
        delete control_data;

        delete p_usbdata_lcm;
        delete p_usbcmd_lcm;
    }


    void Motor_Control_Board::USB2CAN_SetBuffer(USB_Command_t *_control_cmd, USB_Data_t *_controller_data) {
        control_cmd = _control_cmd;
        control_data = _controller_data;
    }

    void Motor_Control_Board::motor_epin_callback(struct libusb_transfer *_transfer) {
        if (_transfer->status != LIBUSB_TRANSFER_COMPLETED) {
            std::cout << RED << "[USB2CAN ERROR]: " << RESET << "Motor Ep81 IN Error! Transfer again!\n";

        } else if (_transfer->status == LIBUSB_TRANSFER_COMPLETED) {
            this->Deal_Usb_In_Data();
            libusb_submit_transfer(_transfer);
        }
    }

    void Motor_Control_Board::motor_epout_callback(struct libusb_transfer *_transfer) {
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
    void Motor_Control_Board::Deal_Usb_In_Data() {
        uint32_t t = data_checksum((uint32_t *) usb_data_u, usb_motors_in_check_length);
        volatile uint8_t leg_id;
        volatile uint8_t data_index;
        if (usb_data_u->usb_data.checksum == t) {
            {
                std::lock_guard<std::mutex> lock(usb_in_mutex);
                for (uint8_t i = 0; i < 2 * NUMBER_CHIPS; i++) {
                    leg_id = i / 2;
                    data_index = i % 2;
                    control_data->q_abad[i] =
                            (usb_data_u->usb_data.leg_data[leg_id].p_abad_data[data_index] - abad_offset[i]) *
                            abad_side_sign[i];
                    control_data->q_hip[i] =
                            (usb_data_u->usb_data.leg_data[leg_id].p_hip_data[data_index] - hip_offset[i]) *
                            hip_side_sign[i];
                    control_data->q_knee[i] =
                            (usb_data_u->usb_data.leg_data[leg_id].p_knee_data[data_index] - knee_offset[i]) *
                            knee_side_sign[i];
                    control_data->qd_abad[i] =
                            usb_data_u->usb_data.leg_data[leg_id].v_abad_data[data_index] * abad_side_sign[i];
                    control_data->qd_hip[i] =
                            usb_data_u->usb_data.leg_data[leg_id].v_hip_data[data_index] * hip_side_sign[i];
                    control_data->qd_knee[i] =
                            usb_data_u->usb_data.leg_data[leg_id].v_knee_data[data_index] * knee_side_sign[i];
                    control_data->tau_abad[i] =
                            usb_data_u->usb_data.leg_data[leg_id].t_abad_data[data_index] * abad_side_sign[i];
                    control_data->tau_hip[i] =
                            usb_data_u->usb_data.leg_data[leg_id].t_hip_data[data_index] * hip_side_sign[i];
                    control_data->tau_knee[i] =
                            usb_data_u->usb_data.leg_data[leg_id].t_knee_data[data_index] / knee_side_sign[i];

                    control_data->ud_abad[i] = usb_data_u->usb_data.leg_data[leg_id].ud_abad_data[data_index];
                    control_data->ud_hip[i]= usb_data_u->usb_data.leg_data[leg_id].ud_hip_data[data_index];
                    control_data->ud_knee[i] = usb_data_u->usb_data.leg_data[leg_id].ud_knee_data[data_index];

                    control_data->uq_abad[i] = usb_data_u->usb_data.leg_data[leg_id].uq_abad_data[data_index];
                    control_data->uq_hip[i] = usb_data_u->usb_data.leg_data[leg_id].uq_hip_data[data_index];
                    control_data->uq_knee[i] = usb_data_u->usb_data.leg_data[leg_id].uq_knee_data[data_index];
                }
                control_data->flags[0] = (int32_t) usb_data_u->usb_data.leg_data[0].leg_flag[0];
                control_data->flags[1] = (int32_t) usb_data_u->usb_data.leg_data[1].leg_flag[0];
                memcpy(p_usbdata_lcm, control_data, sizeof(USB_Data_t));
                p_usbdata_lcm->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                usb_data_LCM.publish("MOTOR DATA", p_usbdata_lcm);
            }
        } else {
            std::cout << BOLDRED << "[USB2CAN ERROR]: " << RESET << "usb data checksum error!\n";
        }
    }

/**
 * @brief USB cmd out sending handler
 */
    void Motor_Control_Board::Deal_Usb_Out_Cmd() {
        volatile uint8_t leg_id;
        volatile uint8_t cmd_index;
        {
            std::lock_guard<std::mutex> lock_out(usb_out_mutex);
            for (uint8_t i = 0; i < 2 * NUMBER_CHIPS; i++) {
                leg_id = i / 2;
                cmd_index = i % 2;
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].p_abad_cmd[cmd_index] =
                        (control_cmd->q_des_abad[i] * abad_side_sign[i]) + abad_offset[i];
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].p_hip_cmd[cmd_index] =
                        (control_cmd->q_des_hip[i] * hip_side_sign[i]) + hip_offset[i];
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].p_knee_cmd[cmd_index] =
                        (control_cmd->q_des_knee[i] / knee_side_sign[i]) + knee_offset[i];

                usb_cmd_u->usb_cmd.leg_cmd[leg_id].v_abad_cmd[cmd_index] =
                        control_cmd->qd_des_abad[i] * abad_side_sign[i];
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].v_hip_cmd[cmd_index] = control_cmd->qd_des_hip[i] * hip_side_sign[i];
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].v_knee_cmd[cmd_index] =
                        control_cmd->qd_des_knee[i] / knee_side_sign[i];

                usb_cmd_u->usb_cmd.leg_cmd[leg_id].kp_abad_cmd[cmd_index] = control_cmd->kp_abad[i];
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].kp_hip_cmd[cmd_index] = control_cmd->kp_hip[i];
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].kp_knee_cmd[cmd_index] = control_cmd->kp_knee[i];

                usb_cmd_u->usb_cmd.leg_cmd[leg_id].kd_abad_cmd[cmd_index] = control_cmd->kd_abad[i];
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].kd_hip_cmd[cmd_index] = control_cmd->kd_hip[i];
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].kd_knee_cmd[cmd_index] = control_cmd->kd_knee[i];

                usb_cmd_u->usb_cmd.leg_cmd[leg_id].t_abad_cmd[cmd_index] =
                        control_cmd->tau_abad_ff[i] * abad_side_sign[i];
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].t_hip_cmd[cmd_index] = control_cmd->tau_hip_ff[i] * hip_side_sign[i];
                usb_cmd_u->usb_cmd.leg_cmd[leg_id].t_knee_cmd[cmd_index] =
                        control_cmd->tau_knee_ff[i] * knee_side_sign[i];
            }
            usb_cmd_u->usb_cmd.leg_cmd[0].leg_flag[0] = control_cmd->flags[0];
            usb_cmd_u->usb_cmd.leg_cmd[1].leg_flag[0] = control_cmd->flags[1];
            usb_cmd_u->usb_cmd.checksum = data_checksum((uint32_t *) usb_cmd_u, usb_motors_out_check_length);
            memcpy(p_usbcmd_lcm, control_cmd, sizeof(usb_command_t));
            usb_cmd_LCM.publish("MOTOR COMMAND", p_usbcmd_lcm);
        }
    }

    void Motor_Control_Board::start_transfer() {
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
        auto *temp = reinterpret_cast<USB_HARDWARE::Motor_Control_Board *>(_transfer->user_data);
        temp->motor_epin_callback(_transfer);
    }

    void usb_motors_out_cbf_wrapper(struct libusb_transfer *_transfer) {
        auto *temp = reinterpret_cast<USB_HARDWARE::Motor_Control_Board *>(_transfer->user_data);
        temp->motor_epout_callback(_transfer);
    }
}


