//
// Created by lingwei on 7/4/24.
//
#include "my_fdsc.h"

#include <std_cout_colors.h>

#include "../../../utilities/inc/utilities_fun.h"

My_FDSC::My_FDSC(connection_type type) : imu_data_LCM(getLcmUrl(255)), usb_data_LCM(getLcmUrl(255)) {
    if (type == low_wired) {
        ptr_connection_ = std::make_unique<FDSC::UnitreeConnection>("LOW_WIRED_DEFAULTS");
    } else if (type == sim_default) {
        ptr_connection_ = std::make_unique<FDSC::UnitreeConnection>("SIM_DEFAULTS");
    } else if (type == high_wifi) {
        ptr_connection_ = std::make_unique<FDSC::UnitreeConnection>("HIGH_WIFI_DEFAULTS");
    } else {
        std::cout << RED << "[FDSC_ERROR]: " << RESET << "Can not match connection type!\n";
    }
}


void My_FDSC::set_buffer(USB_Command_t *usb_command, USB_Data_t *usb_data, USB_Imu_t *usb_imu) {
    usb_command_ = usb_command;
    usb_data_ = usb_data;
    usb_imu_ = usb_imu;
}

void My_FDSC::FDSC_init() {
    ptr_connection_->startRecv();
    std::vector<uint8_t> cmd_bytes = low_cmd_.buildCmd(false);
    ptr_connection_->send(cmd_bytes);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    FDSC::show_in_lowcmd();
}

void My_FDSC::show_info() {
    std::cout << "=============================" << std::endl;
    std::cout << SetForeGRN << "------------------HardSoft Version info: -------------------" << std::endl;
    std::cout << "SN: " << "\t";
    FDSC::byte_print(low_state_.SN, false);
    std::cout << FDSC::decode_sn(low_state_.SN) << std::endl;
    std::cout << "Version: " << "\t";
    FDSC::byte_print(low_state_.version, false);
    std::cout << FDSC::decode_version(low_state_.version) << std::endl;
    std::cout << "Bandwidth: " << "\t" << FDSC::hex_to_kp_kd(low_state_.bandWidth) << std::endl;
    std::cout << SetForeGRN << "------------------IMU info: -------------------" << std::endl;
    std::cout << "IMU Quaternion: ";
    FDSC::pretty_show_vector(low_state_.imu_quaternion);
    std::cout << std::endl;
    std::cout << "IMU Gyroscope: ";
    FDSC::pretty_show_vector(low_state_.imu_gyroscope);
    std::cout << std::endl;
    std::cout << "IMU Accelerometer: ";
    FDSC::pretty_show_vector(low_state_.imu_accelerometer);
    std::cout << std::endl;
    std::cout << "IMU RPY: ";
    FDSC::pretty_show_vector(low_state_.imu_rpy);
    std::cout << "IMU Temperature: " << static_cast<float>(low_state_.temperature_imu) << std::endl;
    // std::vector<uint8_t> imu_data(data.begin()+22,data.begin()+75);
    // FDSC::show_byte_data(data,8);
    // std::cout<<std::endl;
    std::cout << SetForeGRN << "------------------Joint info: -------------------" << std::endl;
    show_joint_info(low_state_.motorState);
    // BMS do not have any data from UDP data
    std::cout << SetForeRED << "------------------BMS info(No data): -------------------" << std::endl;
    std::cout << "SOC: " << static_cast<int>(low_state_.SOC) << std::endl;
    // std::cout << "Overall Voltage: " << FDSC::getVoltage(lstate.cell_vol) << "mV" << std::endl;
    std::cout << "Current: " << low_state_.current << "mA" << std::endl;
    std::cout << "Cycles: " << low_state_.cycle << std::endl;
    std::cout << "Temps BQ: " << low_state_.BQ_NTC[0] << "°C, " << low_state_.BQ_NTC[1] << "°C" << std::endl;
    std::cout << "Temps MCU: " << low_state_.MCU_NTC[0] << "°C, " << low_state_.MCU_NTC[1] << "°C" << std::endl;
    std::cout << SetForeRED << "------------------Foot Force info(No data In Go1 Air): -------------------" <<
            std::endl;
    std::cout << "Footforce: ";
    for (auto data_f: low_state_.footForce) {
        std::cout << static_cast<float>(data_f) << " ";
    }
    std::cout << std::endl;
    std::cout << "FootforceEst: ";
    for (auto data_f: low_state_.footForceEst) {
        std::cout << static_cast<float>(data_f) << " ";
    }
    std::cout << "=============================" << std::endl;
}

void My_FDSC::show_joint_info(const std::vector<FDSC::MotorState> &mobj) {
    for (int i = 0; i < 12; i++) {
        std::cout << "Joint: " << i << "   Motor Mode: " << mobj[i].mode << " Motor NTC: " << mobj[i].temperature <<
                " ";
        std::cout << " q: " << mobj[i].q << " dq: " << mobj[i].dq << " ddq: " << mobj[i].ddq << " TauEst: " << mobj[i].
                tauEst << std::endl;
    }
}

void My_FDSC::parse_data() {
    std::vector<std::vector<uint8_t> > dataall;
    ptr_connection_->getData(dataall);
    if (!dataall.empty()) {
        std::vector<uint8_t> data = dataall.at(dataall.size() - 1);
        low_state_.parseData(data); {
            std::lock_guard lk(data_mtx_);
            for (int i = 0; i < 3; i++) {
                usb_imu_->accel[i] = low_state_.imu_accelerometer[i];
                usb_imu_->gyro[i] = low_state_.imu_gyroscope[i];
                usb_imu_->q[i] = low_state_.imu_quaternion[i];
            }
            usb_imu_->q[3] = low_state_.imu_quaternion[3];
            memcpy(&imu_data_lcm_, usb_imu_, sizeof(USB_Imu_t));
            imu_data_lcm_.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();

            for (int i = 0; i < 4; i++) {
                usb_data_->q_abad[i] = low_state_.motorState[motors_abad[i]].q;
                usb_data_->qd_abad[i] = low_state_.motorState[motors_abad[i]].dq;
                usb_data_->q_hip[i] = low_state_.motorState[motors_hip[i]].q;
                usb_data_->qd_hip[i] = low_state_.motorState[motors_hip[i]].dq;
                usb_data_->q_knee[i] = low_state_.motorState[motors_knee[i]].q;
                usb_data_->qd_knee[i] = low_state_.motorState[motors_knee[i]].dq;
            }
            memcpy(&usbdata_lcm_, usb_data_, sizeof(USB_Data_t));
        }
        imu_data_LCM.publish("IMU_CHANNEL", &imu_data_lcm_);
        usb_data_LCM.publish("MOTOR DATA", &usbdata_lcm_);
    }
}

void My_FDSC::pack_cmd() { {
        std::lock_guard lk(cmd_mtx_);
        for (int i = 0; i < 4; i++) {
            std::vector<float> abad_joint{
                usb_command_->q_des_abad[i], usb_command_->qd_des_abad[i], usb_command_->tau_abad_ff[i],
                usb_command_->kp_abad[i], usb_command_->kd_abad[i]
            };
            motor_cmd_array_.setMotorCmd(motors_abad_name[i], FDSC::MotorModeLow::Servo, abad_joint);
            std::vector<float> hip_joint{
                usb_command_->q_des_hip[i], usb_command_->qd_des_hip[i], usb_command_->tau_hip_ff[i],
                usb_command_->kp_hip[i], usb_command_->kd_hip[i]
            };
            motor_cmd_array_.setMotorCmd(motors_hip_name[i], FDSC::MotorModeLow::Servo, hip_joint);
            std::vector<float> knee_joint{
                usb_command_->q_des_knee[i], usb_command_->qd_des_knee[i], usb_command_->tau_knee_ff[i],
                usb_command_->kp_knee[i], usb_command_->kd_knee[i]
            };
            motor_cmd_array_.setMotorCmd(motors_knee_name[i], FDSC::MotorModeLow::Servo, knee_joint);
        }
    }
    low_cmd_.motorCmd = motor_cmd_array_;
    std::vector<uint8_t> cmdBytes = low_cmd_.buildCmd(false);
    ptr_connection_->send(cmdBytes);
}
