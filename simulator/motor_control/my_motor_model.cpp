//
// Created by lingwei on 5/13/24.
//
#include "my_motor_model.h"
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "../../utilities/inc/LoadData.h"

Motor_Control::Motor_Model::Motor_Model(const std::string &motor_file_) {
// record this first, would be useful in future
    this->GetSettings(motor_file_, "Motor_Control");
}

float Motor_Control::Motor_Model::get_torque(int id_) const {
    return torq_out_(id_);
}


void Motor_Control::Motor_Model::GetSettings(const std::string &filename, const std::string &setting_name) {
//    boost::property_tree::ptree pt;
//    boost::property_tree::read_info(filename, pt);
//    loadData::loadPtreeValue(pt, motor_kp_, setting_name + ".passive_kp", true);
//    loadData::loadPtreeValue(pt, motor_kd_, setting_name + ".passive_kd", true);
//
//    loadData::loadPtreeValue(pt, passive_kp_, setting_name + ".passive_kp", true);
//    loadData::loadPtreeValue(pt, passive_kd_, setting_name + ".passive_kd", true);
//
//    loadData::loadPtreeValue(pt, stand_kp_, setting_name + ".stand_kp", true);
//    loadData::loadPtreeValue(pt, stand_kd_, setting_name + ".stand_kd", true);

    kp_mat_ = Eigen::Matrix<float, 12, 12>::Identity();
    kd_mat_ = Eigen::Matrix<float, 12, 12>::Identity();

    q_cmd_.setZero();
    q_data_.setZero();
    qd_cmd_.setZero();
    q_data_.setZero();
    torq_out_.setZero();
    t_ff_.setZero();
}

void Motor_Control::Motor_Model::pack_motor_cmd(const USB_Command_t *usb_cmd, const USB_Data_t *usb_data) {
    for (int i = 0; i < 4; i++) {
        q_cmd_(3 * i) = usb_cmd->q_des_abad[i];
        q_cmd_(3 * i + 1) = usb_cmd->q_des_hip[i];
        q_cmd_(3 * i + 2) = usb_cmd->q_des_knee[i];

        q_data_(3 * i) = usb_data->q_abad[i];
        q_data_(3 * i + 1) = usb_data->q_hip[i];
        q_data_(3 * i + 2) = usb_data->q_knee[i];

        qd_cmd_(3 * i) = usb_cmd->qd_des_abad[i];
        qd_cmd_(3 * i + 1) = usb_cmd->qd_des_hip[i];
        qd_cmd_(3 * i + 2) = usb_cmd->qd_des_knee[i];

        qd_data_(3 * i) = usb_data->qd_abad[i];
        qd_data_(3 * i + 1) = usb_data->qd_hip[i];
        qd_data_(3 * i + 2) = usb_data->qd_knee[i];

        t_ff_(3 * i) = usb_cmd->tau_abad_ff[i];
        t_ff_(3 * i + 1) = usb_cmd->tau_hip_ff[i];
        t_ff_(3 * i + 2) = usb_cmd->tau_knee_ff[i];

        kp_mat_(3 * i, 3 * i) = usb_cmd->kp_abad[i];
        kp_mat_(3 * i + 1, 3 * i + 1) = usb_cmd->kp_hip[i];
        kp_mat_(3 * i + 2, 3 * i + 2) = usb_cmd->kp_knee[i];

        kd_mat_(3 * i, 3 * i) = usb_cmd->kd_abad[i];
        kd_mat_(3 * i + 1, 3 * i + 1) = usb_cmd->kd_hip[i];
        kd_mat_(3 * i + 2, 3 * i + 2) = usb_cmd->kd_knee[i];
    }

    torq_out_ = kp_mat_ * (q_cmd_ - q_data_) + kd_mat_ * (qd_cmd_ - qd_data_) + t_ff_;
}

void Motor_Control::Motor_Model::set_motor_kp_kd(float kp, float kd) {
    kp_mat_ = Eigen::Matrix<float, 12, 12>::Identity() * kp;
    kd_mat_ = Eigen::Matrix<float, 12, 12>::Identity() * kd;
}

