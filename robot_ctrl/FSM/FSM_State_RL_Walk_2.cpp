#include "FSM_State_RL_Walk_2.h"
#include <iostream>

FSM_State_RL_Walk_2::FSM_State_RL_Walk_2(Control_FSM_Data *_controlFSMData, Control_Parameters_t *control_para): FSM_State(
    _controlFSMData, control_para, RL_WALK) {
    rl_controller_ = std::make_shared<RLController2>();
}

bool FSM_State_RL_Walk_2::state_on_enter() {
    std::cout << YELLOW << "[FSM State]: Enter RL WALK.\n" << RESET;
    rl_controller_->init();
    desired_vel_xyw_last.setZero();
    return true;
}

void FSM_State_RL_Walk_2::state_on_exit() {
}

void FSM_State_RL_Walk_2::run_state() {
    // Demo Send CMD

    // Demo Get Data
    // 1. get all data.
    Vec19<double> joint_q;
    Vec18<double> joint_qd;
    Vec3<double> accel;
    Vec3<double> desired_vel_xyw;
    Vec3<double> desired_vel_xyw_command;
    //x y z | q_w q_x q_y q_z | joint data |
    get_joint_state(joint_q, joint_qd, accel);
    // desired_vel_xyw << fsm_data_->rc_->rc_control_.v_des[0]*1, fsm_data_->rc_->rc_control_.v_des[1]*0.3, fsm_data_->rc_->rc_control_.v_des[2]*1.5;
    // std::cout << "desired_vel_xyw: " << desired_vel_xyw.transpose() << std::endl;
    desired_vel_xyw_command << fsm_data_->rc_->rc_control_.v_des[0]*1, fsm_data_->rc_->rc_control_.v_des[1]*0.3, fsm_data_->rc_->rc_control_.v_des[2]*1.5;
    desired_vel_xyw = desired_vel_xyw_last*0.99 + desired_vel_xyw_command*0.01;
    desired_vel_xyw_last = desired_vel_xyw;
    //please note the position and velocity respect to the inertial frame are null
    // 2. get from estimators
    (void) this->fsm_data_->estimators_->get_result_quat();
    (void) this->fsm_data_->estimators_->get_reult_acc_w();
    (void) this->fsm_data_->leg_controller_->leg_data[0].q(0);
    rl_controller_->step(&joint_q, &joint_qd, &accel, &desired_vel_xyw);
    // 3. send to leg controller
    int i = 0;
    for (auto &leg: this->fsm_data_->leg_controller_->leg_command) {
        leg.kp_joint = Vec3<double>(80, 80, 80).asDiagonal();
        leg.kd_joint = Vec3<double>(2, 2, 2).asDiagonal();
        leg.q_des = Vec3<double>(rl_controller_->desired_positions[i*3], rl_controller_->desired_positions[i*3+1], rl_controller_->desired_positions[i*3+2]);
        leg.qd_des = Vec3<double>(0, 0, 0);
        i++;
    }
}

bool FSM_State_RL_Walk_2::is_busy() {
    return false;
}
