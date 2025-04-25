//
// Created by lingwei on 5/16/24.
//

#include "FSM_State.h"

void FSM_State::get_joint_state(Vec19<double> &joint_q, Vec18<double> &joint_qd, Vec3<double> &accel) const {
    const Vec3<double> p_w = this->fsm_data_->estimators_->get_result_world_position();
    const Vec3<double> v_w = this->fsm_data_->estimators_->get_result_world_velocity();
    const Quat<double> quat = this->fsm_data_->estimators_->get_result_quat();
    const Vec3<double> angular_v = this->fsm_data_->estimators_->get_result_angular_body();
    accel = this->fsm_data_->estimators_->get_reult_acc_w();

    Vec3<double> leg_1 = this->fsm_data_->leg_controller_->leg_data[0].q;
    Vec3<double> leg_2 = this->fsm_data_->leg_controller_->leg_data[1].q;
    Vec3<double> leg_3 = this->fsm_data_->leg_controller_->leg_data[2].q;
    Vec3<double> leg_4 = this->fsm_data_->leg_controller_->leg_data[3].q;
    Vec3<double> leg_1d = this->fsm_data_->leg_controller_->leg_data[0].qd;
    Vec3<double> leg_2d = this->fsm_data_->leg_controller_->leg_data[1].qd;
    Vec3<double> leg_3d = this->fsm_data_->leg_controller_->leg_data[2].qd;
    Vec3<double> leg_4d = this->fsm_data_->leg_controller_->leg_data[3].qd;
    joint_q.setZero();
    joint_qd.setZero();
    joint_q << p_w, quat, leg_1, leg_2, leg_3, leg_4;
    joint_qd << v_w, angular_v, leg_1d, leg_2d, leg_3d, leg_4d;
}
