//
// Created by lingwei on 5/20/24.
//
#include "BodyPosTask.h"
#include <iostream>

template<typename T>
BodyPosTask<T>::BodyPosTask(const Quadruped_Base *base_model):Task_Base<T>(3), robot_(base_model) {
    Task_Base<T>::Jt_.setZero();
    Task_Base<T>::Jt_.block(0, 0, 3, 3).setIdentity();
    Task_Base<T>::Jtdqd_ = DVec<T>::Zero(Task_Base<T>::dim_task_);
    kp_kin_ = DVec<T>::Constant(Task_Base<T>::dim_task_, 1.0);
    kp_ = DVec<T>::Constant(Task_Base<T>::dim_task_, 50.0);
    kd_ = DVec<T>::Constant(Task_Base<T>::dim_task_, 1.0);
}

template<typename T>
bool BodyPosTask<T>::UpdateTaskJdqd() {
    return true;
}

template<typename T>
bool BodyPosTask<T>::UpdateTaskJacobian() {
    Quat<T> quat = robot_->get_model_quat_copy();
    Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
    Task_Base<T>::Jt_.block(0, 0, 3, 3) = Rot.transpose();
//    std::cout << "Pos: \n" << Task_Base<T>::Jt_ << std::endl;
    return true;
}

template<typename T>
bool BodyPosTask<T>::UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des) {
    auto *pos_cmd = (Vec3<T> *) pos_des;
    Vec3<T> link_pos = robot_->get_model_pos_copy();
    // Quat<T> quat = robot_->get_model_quat_copy();
    // Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);

    Vec3<T> curr_vel_w = robot_->get_model_vel_copy();
    // Vec3<T> curr_vel_b =
    //         Rot.transpose() * curr_vel_w;  //body to world, but why? Given speed is relate to the local frame.

    // X, Y, Z
    // std::cout << "ori pos: " << pos_cmd->transpose() << std::endl;
    for (int i(0); i < 3; ++i) {
        Task_Base<T>::pos_err_[i] = kp_kin_[i] * ((*pos_cmd)[i] - link_pos[i]);
        Task_Base<T>::vel_des_[i] = vel_des[i];
        Task_Base<T>::acc_des_[i] = acc_des[i];

        Task_Base<T>::op_cmd_[i] = kp_[i] * ((*pos_cmd)[i] - link_pos[i]) +
                                   kd_[i] * (Task_Base<T>::vel_des_[i] - curr_vel_w[i]) +
                                   Task_Base<T>::acc_des_[i];
    }
    // std::cout << "pos CMD: " << Task_Base<T>::op_cmd_.transpose() << std::endl;
    return true;
}

template
class BodyPosTask<double>;
