//
// Created by lingwei on 5/20/24.
//
#include "BodyOriTask.h"
#include <iostream>

template<typename T>
BodyOriTask<T>::BodyOriTask(const Quadruped_Base *base_model): Task_Base<T>(3), robot_(base_model) {
    // Contact Jacobian fixed
    Task_Base<T>::Jt_.setZero();
    Task_Base<T>::Jt_.block(0, 3, 3, 3).setIdentity();
    Task_Base<T>::Jtdqd_ = DVec<T>::Zero(Task_Base<T>::dim_task_);
    kp_kin_ = DVec<T>::Constant(Task_Base<T>::dim_task_, 1.0);
    kp_ = DVec<T>::Constant(Task_Base<T>::dim_task_, 50.);
    kd_ = DVec<T>::Constant(Task_Base<T>::dim_task_, 1.0);
}

template<typename T>
bool BodyOriTask<T>::UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des) {
    auto *ori_cmd = (Quat<T> *) pos_des;
    Quat<T> link_ori = robot_->get_model_quat_copy();
    Quat<T> link_ori_invert;
    link_ori_invert[0] = link_ori[0];
    link_ori_invert[1] = -link_ori[1];
    link_ori_invert[2] = -link_ori[2];
    link_ori_invert[3] = -link_ori[3];

    // quat sub
    Quat<T> ori_err = ori::quatProduct(*ori_cmd, link_ori_invert);
    if (ori_err[0] < 0.0) {
        ori_err *= (-1.);
    }

    Vec3<T> ori_err_so3;
    ori::quaternionToso3(ori_err, ori_err_so3);
    // Configuration space: Local
    // Operational Space: Global
    Mat3<T> Rot = ori::quaternionToRotationMatrix(link_ori);
    Vec3<T> curr_vel = robot_->get_model_omega_copy(); //base frame
    Vec3<T> vel_err = Rot.transpose() * (Task_Base<T>::vel_des_ - curr_vel);
    // Vec3<T> vel_err = Task_Base<T>::vel_des_ - curr_vel;
    for (int i = 0; i < 3; i++) {
        Task_Base<T>::pos_err_[i] = kp_kin_[i] * ori_err_so3[i];
        Task_Base<T>::vel_des_[i] = vel_des[i];
        Task_Base<T>::acc_des_[i] = acc_des[i];
        Task_Base<T>::op_cmd_[i] = kp_[i] * ori_err_so3[i] + kd_[i] * vel_err[i] + Task_Base<T>::acc_des_[i];
    }
    return true;
}

template<typename T>
bool BodyOriTask<T>::UpdateTaskJdqd() {
    return true;
}

template<typename T>
bool BodyOriTask<T>::UpdateTaskJacobian() {
    Quat<T> quat = robot_->get_model_quat_copy();
    Mat3<T> rot = ori::quaternionToRotationMatrix(quat);
    Task_Base<T>::Jt_.block(0, 3, 3, 3) = rot.transpose(); // base to world
    //    std::cout << "Ori: \n" << Task_Base<T>::Jt_ << std::endl;
    return false;
}

template
class BodyOriTask<double>;
