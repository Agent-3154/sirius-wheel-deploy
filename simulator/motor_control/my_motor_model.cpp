//
// Created by lingwei on 5/13/24.
//
#include "my_motor_model.h"
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "../../utilities/inc/LoadData.h"
#include <easylogging++.h>

Motor_Control::Motor_Model::Motor_Model(const std::string &motor_file_) {
    // record this first, would be useful in future
    this->GetSettings(motor_file_, "Motor_Control");
}

float Motor_Control::Motor_Model::get_torque(int id_) const {
    // LOG(INFO) << "torq_out_: " << id_ << " " << torq_out_(id_) << std::endl;
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

    // kp_mat_ = Eigen::Matrix<float, 16, 16>::Identity();
    // kd_mat_ = Eigen::Matrix<float, 16, 16>::Identity();
    kp_mat_.setZero();
    kd_mat_.setZero();

    q_cmd_.setZero();
    q_data_.setZero();
    qd_cmd_.setZero();
    qd_data_.setZero();
    torq_out_.setZero();
    t_ff_.setZero();
}

void Motor_Control::Motor_Model::pack_motor_cmd(const USB_Command_t *usb_cmd, const USB_Data_t *usb_data) {
    kp_mat_.setZero();
    kd_mat_.setZero();
    for (int i = 0; i < 4; i++) {
        const int index = i / 2;
        const int index_shift = index * 2;
        q_cmd_(3 * i) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift)].q_des;
        q_cmd_(3 * i + 1) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].q_des;
        q_cmd_(3 * i + 2) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].q_des;

        q_data_(3 * i) = usb_data->chip_datas[index].motor_datas[3 * (i - index_shift)].q;
        q_data_(3 * i + 1) = usb_data->chip_datas[index].motor_datas[3 * (i - index_shift) + 1].q;
        q_data_(3 * i + 2) = usb_data->chip_datas[index].motor_datas[3 * (i - index_shift) + 2].q;

        qd_cmd_(3 * i) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift)].qd_des;
        qd_cmd_(3 * i + 1) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].qd_des;
        qd_cmd_(3 * i + 2) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].qd_des;

        qd_data_(3 * i) = usb_data->chip_datas[index].motor_datas[3 * (i - index_shift)].qd;
        qd_data_(3 * i + 1) = usb_data->chip_datas[index].motor_datas[3 * (i - index_shift) + 1].qd;
        qd_data_(3 * i + 2) = usb_data->chip_datas[index].motor_datas[3 * (i - index_shift) + 2].qd;

        t_ff_(3 * i) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift)].tau_ff;
        t_ff_(3 * i + 1) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].tau_ff;
        t_ff_(3 * i + 2) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].tau_ff;

        kp_mat_(3 * i, 3 * i) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift)].kp;
        kp_mat_(3 * i + 1, 3 * i + 1) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].kp;
        kp_mat_(3 * i + 2, 3 * i + 2) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].kp;

        kd_mat_(3 * i, 3 * i) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift)].kd;
        kd_mat_(3 * i + 1, 3 * i + 1) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].kd;
        kd_mat_(3 * i + 2, 3 * i + 2) = usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].kd;
        // std::cout << "kd_mat_" <<usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift)].kp<< std::endl;
        // std::cout << "kd_mat_" <<usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].kp<< std::endl;
        // std::cout << "kd_mat_" <<usb_cmd->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].kp<< std::endl;
    }

    //for wheel control
    for (int i = 0; i < 4; i++) {
        const int index = 2;
        const int index_shift = i / 2;
        q_cmd_(12 + i) = usb_cmd->chip_cmds[index].motor_cmds[i + index_shift].q_des;
        q_data_(12 + i) = usb_data->chip_datas[index].motor_datas[i + index_shift].q;
        qd_cmd_(12 + i) = usb_cmd->chip_cmds[index].motor_cmds[i + index_shift].qd_des;
        qd_data_(12 + i) = usb_data->chip_datas[index].motor_datas[i + index_shift].qd;
        t_ff_(12 + i) = usb_cmd->chip_cmds[index].motor_cmds[i + index_shift].tau_ff;
        kp_mat_(12 + i, 12 + i) = usb_cmd->chip_cmds[index].motor_cmds[i + index_shift].kp;
        kd_mat_(12 + i, 12 + i) = usb_cmd->chip_cmds[index].motor_cmds[i + index_shift].kd;
        // std::cout << "kd_mat_" <<usb_cmd->chip_cmds[index].motor_cmds[i + index_shift].kd<< std::endl;
    }

    torq_out_ = kp_mat_ * (q_cmd_ - q_data_) + kd_mat_ * (qd_cmd_ - qd_data_) + t_ff_;
//     for (int i = 0; i < 16; i++) {
//         for (int j = 0; j < 16; j++) {
//             // if (kp_mat_(i,j) != 0) {  
//                 // std::cout << "kp = " <<kp_mat_(i,j)<<std::endl;
//             // }
//         }
//     }
    // std::cout <<kd_mat_<<std::endl;
    // std::cout << "   "<<std::endl;
}

void Motor_Control::Motor_Model::set_motor_kp_kd(float kp, float kd) {
    kp_mat_ = Eigen::Matrix<float, 16, 16>::Identity() * kp;
    kd_mat_ = Eigen::Matrix<float, 16, 16>::Identity() * kd;
    //  std::cout << "   "<<std::endl;
}
