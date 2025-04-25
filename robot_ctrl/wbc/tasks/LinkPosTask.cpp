//
// Created by lingwei on 5/20/24.
//
#include "LinkPosTask.h"
#include <iostream>

template<typename T>
bool LinkPoseTask<T>::UpdateTaskJdqd() {
    Task_Base<T>::Jtdqd_ = robot_->get_jcdqd(link_id_);
    return true;
}

template<typename T>
bool LinkPoseTask<T>::UpdateTaskJacobian() {
    Task_Base<T>::Jt_ = robot_->get_jc(link_id_);
//    std::cout << "LinkTask: " << link_id_ << "\n" << Task_Base<T>::Jt_ << std::endl;
    return true;
}

template<typename T>
bool LinkPoseTask<T>::UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des) {

    auto *pos_cmd = (Vec3<T> *) pos_des;
    Vec3<T> link_pos;

    link_pos = robot_->get_pGC(link_id_);
    // std::cout << "\n ID: " << link_id_ << " | cmd: " << pos_cmd->transpose() << " pos: " << link_pos.transpose() << "\n";
    // X, Y, Z
    for (int i(0); i < 3; ++i) {
        Task_Base<T>::pos_err_[i] = kp_kin_[i] * ((*pos_cmd)[i] - link_pos[i]);
        Task_Base<T>::vel_des_[i] = vel_des[i];
        Task_Base<T>::acc_des_[i] = acc_des[i];
    }

    // Op acceleration command
    for (int i(0); i < Task_Base<T>::dim_task_; ++i) {
        Task_Base<T>::op_cmd_[i] =
                kp_[i] * Task_Base<T>::pos_err_[i] +
                kd_[i] * (Task_Base<T>::vel_des_[i] - robot_->get_vGC(link_id_)(i)) + Task_Base<T>::acc_des_[i];
    }
    return true;
}

template<typename T>
LinkPoseTask<T>::LinkPoseTask(const Quadruped_Base *base_model, int link_id):Task_Base<T>(3), link_id_(link_id),
                                                                             robot_(base_model) {
    Task_Base<T>::Jtdqd_ = DVec<T>::Zero(Task_Base<T>::dim_task_);
    kp_ = DVec<T>::Constant(Task_Base<T>::dim_task_, 100.);
    kd_ = DVec<T>::Constant(Task_Base<T>::dim_task_, 5.);
    kp_kin_ = DVec<T>::Constant(Task_Base<T>::dim_task_, 1.);
}

template
class LinkPoseTask<double>;
