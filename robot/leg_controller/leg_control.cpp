//
// Created by lingwei on 4/17/24.
//
#include "leg_control.h"

#include <std_cout_colors.h>

#include "iostream"
#include "../../config/Config.h"
#include "../../config/robots_config.h"

template<typename T>
Leg_Controller<T>::Leg_Controller() {
    link1_ = Config::AbadLinkLength;
    link2_ = Config::HipLinkLength;
    link3_ = Config::KneeLinkLength;
    nlegs_ = 4;

    Zero_Command();
    Zero_Data();
}

template<typename T>
void Leg_Controller<T>::Zero_Command() {
    for (auto &leg: leg_command) {
        leg.q_des = Vec3<T>::Zero();
        leg.qd_des = Vec3<T>::Zero();
        leg.tau_ff = Vec3<T>::Zero();
        leg.foot_force = Vec3<T>::Zero();

        leg.whl_q_des = 0.0;
        leg.whl_qd_des = 0.0;
        leg.whl_tau_ff = 0.0;
        leg.whl_kp_joint = 0.0;
        leg.whl_kd_joint = 0.0;

        //foot
        leg.p_des = Vec3<T>::Zero();
        leg.v_des = Vec3<T>::Zero();
        leg.kp_cartisian = Mat3<T>::Zero();
        leg.kd_cartisian = Mat3<T>::Zero();
        leg.kp_joint = Mat3<T>::Zero();
        leg.kd_joint = Mat3<T>::Zero();
    }
}

template<typename T>
void Leg_Controller<T>::Zero_Data() {
    for (auto &leg: leg_data) {
        leg.q = Vec3<T>::Zero();
        leg.qd = Vec3<T>::Zero();
        leg.v = Vec3<T>::Zero();
        leg.p = Vec3<T>::Zero();
        leg.J = Mat3<T>::Zero();
        leg.tau = Vec3<T>::Zero();
        leg.whl_q = 0.0;
        leg.whl_qd = 0.0;
        leg.whl_tau = 0.0;
    }
}

template<typename T>
void Leg_Controller<T>::Setup_Command(USB_Command_t *usb_cmd) {
    for (int leg_id = 0; leg_id < nlegs_; leg_id++) {
        Vec3<T> leg_motor_torques = leg_command[leg_id].tau_ff;
        Vec3<T> leg_foot_force = leg_command[leg_id].foot_force;
        // kp_cartisian, 直接算力
        leg_foot_force += leg_command[leg_id].kp_cartisian * (leg_command[leg_id].p_des - leg_data[leg_id].p);
        leg_foot_force += leg_command[leg_id].kd_cartisian * (leg_command[leg_id].v_des - leg_data[leg_id].v);
        // std::cout << "leg_foot_force: id" << leg_id << "| " << leg_foot_force <<"\n";
        leg_motor_torques += leg_data[leg_id].J.transpose() * leg_foot_force;

        if (leg_motor_torques.array().isNaN().any()) {
            leg_motor_torques.setZero();
            Zero_Command();
            // std::cout << RED << "[ERROR]: " << RESET << " leg_motor_torques is NaN\n" << std::endl;
        }
        // set command
        const int index = leg_id / 2;
        const int index_shift = index * 2;
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift)].tau_ff = leg_motor_torques(0);
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift) + 1].tau_ff = leg_motor_torques(1);
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift) + 2].tau_ff = leg_motor_torques(2);

        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift)].kd = leg_command[leg_id].kd_joint(0, 0);
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift) + 1].kd = leg_command[leg_id].kd_joint(1, 1);
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift) + 2].kd = leg_command[leg_id].kd_joint(2, 2);

        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift)].kp = leg_command[leg_id].kp_joint(0, 0);
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift) + 1].kp = leg_command[leg_id].kp_joint(1, 1);
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift) + 2].kp = leg_command[leg_id].kp_joint(2, 2);

        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift)].q_des = leg_command[leg_id].q_des(0);
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift) + 1].q_des = leg_command[leg_id].q_des(1);
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift) + 2].q_des = leg_command[leg_id].q_des(2);

        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift)].qd_des = leg_command[leg_id].qd_des(0);
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift) + 1].qd_des = leg_command[leg_id].qd_des(1);
        usb_cmd->chip_cmds[index].motor_cmds[3 * (leg_id - index_shift) + 2].qd_des = leg_command[leg_id].qd_des(2);

        //TODO add enable and disable flags
    }
    
    //for wheel control
    for (int leg_id = 0; leg_id < nlegs_; leg_id++) {
        const int index = 2;
        const int index_shift = leg_id / 2;
        usb_cmd->chip_cmds[index].motor_cmds[leg_id + index_shift].tau_ff = leg_command[leg_id].whl_tau_ff;
        usb_cmd->chip_cmds[index].motor_cmds[leg_id + index_shift].kd = leg_command[leg_id].whl_kd_joint;
        usb_cmd->chip_cmds[index].motor_cmds[leg_id + index_shift].kp = leg_command[leg_id].whl_kp_joint;
        usb_cmd->chip_cmds[index].motor_cmds[leg_id + index_shift].q_des = leg_command[leg_id].whl_q_des;
        usb_cmd->chip_cmds[index].motor_cmds[leg_id + index_shift].qd_des = leg_command[leg_id].whl_qd_des;
        // std::cout <<usb_cmd->chip_cmds[index].motor_cmds[leg_id + index_shift].qd_des <<std::endl;
    }
    // std::cout << " "<< std::endl;

    // enable part
    enable_counter++;
    if (enable_counter < 50) {
        usb_cmd->chip_cmds[0].chip_flg = 0x01 << 1;
        usb_cmd->chip_cmds[1].chip_flg = 0x01 << 1;
        usb_cmd->chip_cmds[2].chip_flg = 0x01 << 1;
    } else {
        usb_cmd->chip_cmds[0].chip_flg = usb_cmd->chip_cmds[1].chip_flg =
                                         usb_cmd->chip_cmds[2].chip_flg = 0x01 | (0x01 << 2); // mit mode
    }
    if (enable_counter > 80) enable_counter = 80;
}

template<typename T>
void Leg_Controller<T>::Update_Data(const USB_Data_t *usb_data) {
    //    std::cout << usb_data->qd_knee[4];
    for (int leg = 0; leg < nlegs_; leg++) {
        const int index = leg / 2;
        const int index_shift = index * 2;
        leg_data[leg].q(0) = usb_data->chip_datas[index].motor_datas[3 * (leg - index_shift)].q;
        leg_data[leg].q(1) = usb_data->chip_datas[index].motor_datas[3 * (leg - index_shift) + 1].q;
        leg_data[leg].q(2) = usb_data->chip_datas[index].motor_datas[3 * (leg - index_shift) + 2].q;
        leg_data[leg].qd(0) = usb_data->chip_datas[index].motor_datas[3 * (leg - index_shift)].qd;
        leg_data[leg].qd(1) = usb_data->chip_datas[index].motor_datas[3 * (leg - index_shift) + 1].qd;
        leg_data[leg].qd(2) = usb_data->chip_datas[index].motor_datas[3 * (leg - index_shift) + 2].qd;
        leg_data[leg].tau(0) = usb_data->chip_datas[index].motor_datas[3 * (leg - index_shift)].tau;
        leg_data[leg].tau(1) = usb_data->chip_datas[index].motor_datas[3 * (leg - index_shift) + 1].tau;
        leg_data[leg].tau(2) = usb_data->chip_datas[index].motor_datas[3 * (leg - index_shift) + 2].tau;

        computeLegJacobianAndPosition(leg_data[leg].q, &(leg_data[leg].J), &(leg_data[leg].p), leg);
        leg_data[leg].v = leg_data[leg].J * leg_data[leg].qd;
    }
    
    //for wheel control
    for (int leg = 0; leg < nlegs_; leg++) {
        const int index = 2;
        const int index_shift = leg / 2;
        leg_data[leg].whl_q = usb_data->chip_datas[index].motor_datas[leg + index_shift].q;
        leg_data[leg].whl_qd = usb_data->chip_datas[index].motor_datas[leg + index_shift].qd;
        leg_data[leg].whl_tau = usb_data->chip_datas[index].motor_datas[leg + index_shift].tau;
        // std::cout << "leg_data[leg].q: " << leg_data[leg].whl_q << std::endl;
    }

}


template<typename T>
void Leg_Controller<T>::setLcm(leg_control_data_lcmt *lcmData, leg_control_command_lcmt *lcmCommand) {
    for (int leg = 0; leg < 4; leg++) {
        for (int axis = 0; axis < 3; axis++) {
            int idx = leg * 3 + axis;
            lcmData->q[idx] = leg_data[leg].q[axis];
            lcmData->qd[idx] = leg_data[leg].qd[axis];
            lcmData->p[idx] = leg_data[leg].p[axis];
            lcmData->v[idx] = leg_data[leg].v[axis];
            lcmData->tau_est[idx] = leg_data[leg].tau[axis];
            lcmData->tau_cmd[idx] = leg_command[leg].kp_joint(axis, axis) * (leg_command[leg].q_des[axis] - leg_data[leg].q[axis])
                                    + leg_command[leg].kd_joint(axis, axis) * (leg_command[leg].qd_des[axis] - leg_data[leg].qd[axis])
                                    + leg_command[leg].tau_ff[axis];

            lcmCommand->tau_ff[idx] = leg_command[leg].tau_ff[axis];
            lcmCommand->f_ff[idx] = leg_command[leg].foot_force[axis];
            lcmCommand->q_des[idx] = leg_command[leg].q_des[axis];
            lcmCommand->qd_des[idx] = leg_command[leg].qd_des[axis];
            lcmCommand->p_des[idx] = leg_command[leg].p_des[axis];
            lcmCommand->v_des[idx] = leg_command[leg].v_des[axis];
            lcmCommand->kp_cartesian[idx] = leg_command[leg].kp_cartisian(axis, axis);
            lcmCommand->kd_cartesian[idx] = leg_command[leg].kd_cartisian(axis, axis);
            lcmCommand->kp_joint[idx] = leg_command[leg].kp_joint(axis, axis);
            lcmCommand->kd_joint[idx] = leg_command[leg].kd_joint(axis, axis);
        }

        //for wheel control
        lcmData->q[leg + 12] = leg_data[leg].whl_q;
        lcmData->qd[leg + 12] = leg_data[leg].whl_qd;
        lcmData->tau_est[leg + 12] = leg_data[leg].whl_tau;
        lcmData->tau_cmd[leg + 12] = leg_command[leg].whl_kp_joint * (leg_command[leg].whl_q_des - leg_data[leg].whl_q)
                                    + leg_command[leg].whl_kd_joint * (leg_command[leg].whl_qd_des - leg_data[leg].whl_qd)
                                    + leg_command[leg].whl_tau_ff;
        

        lcmCommand->tau_ff[leg + 12] = leg_command[leg].whl_tau_ff;
        lcmCommand->q_des[leg + 12] = leg_command[leg].whl_q_des;
        lcmCommand->qd_des[leg + 12] = leg_command[leg].whl_qd_des;
        lcmCommand->kp_joint[leg + 12] = leg_command[leg].whl_kp_joint;
        lcmCommand->kd_joint[leg + 12] = leg_command[leg].whl_kd_joint;
    }
}

static int getSideSign(int leg) {
    const int sideSign[4] = {-1, 1, -1, 1};
    // assert(leg >= 0 && leg < 4);
    return sideSign[leg];
}

template<typename T>
void Leg_Controller<T>::computeLegJacobianAndPosition(Vec3<T> &q, Mat3<T> *J, Vec3<T> *p, int leg) {
    auto sideSign = getSideSign(leg);
    T s1 = std::sin(q(0));
    T s2 = std::sin(q(1));
    T s3 = std::sin(q(2));

    T c1 = std::cos(q(0));
    T c2 = std::cos(q(1));
    T c3 = std::cos(q(2));
    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;
    if (J) {
        J->operator()(0, 0) = 0;
        J->operator()(0, 1) = -link3_ * c23 + -link2_ * c2;
        J->operator()(0, 2) = -link3_ * c23;
        J->operator()(1, 0) = link3_ * c1 * c23 + link2_ * c1 * c2 - link1_ * sideSign * s1;
        //        J->operator()(1, 0) = link3_ * c1 * c23 + link2_ * c1 * c2 - link1_ * s1;
        J->operator()(1, 1) = -link3_ * s1 * s23 - link2_ * s1 * s2;
        J->operator()(1, 2) = -link3_ * s1 * s23;
        J->operator()(2, 0) = link3_ * s1 * c23 + link2_ * c2 * s1 + link1_ * sideSign * c1;
        //        J->operator()(2, 0) = link3_ * s1 * c23 + link2_ * c2 * s1 + link1_ * c1;
        J->operator()(2, 1) = link3_ * c1 * s23 + link2_ * c1 * s2;
        J->operator()(2, 2) = link3_ * c1 * s23;
    }
    if (p) {
        p->operator()(0) = -link3_ * s23 - link2_ * s2;
        p->operator()(1) = link1_ * sideSign * c1 + link3_ * (s1 * c23) + link2_ * c2 * s1;
        p->operator()(2) = link1_ * sideSign * s1 - link3_ * (c1 * c23) - link2_ * c1 * c2;
        //        p->operator()(1) = link1_ * c1 + link3_ * (s1 * c23) + link2_ * c2 * s1;
        //        p->operator()(2) = link1_ * s1 - link3_ * (c1 * c23) - link2_ * c1 * c2;
    }
}

template
struct Leg_Control_Data<double>;
template
struct Leg_Control_Command<double>;

template
class Leg_Controller<double>;
