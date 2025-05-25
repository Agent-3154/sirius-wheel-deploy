#include "FSM_State_RL_Walk.h"
#include <iostream>

FSM_State_RL_Walk::FSM_State_RL_Walk(Control_FSM_Data *_controlFSMData, Control_Parameters_t *control_para): FSM_State(
    _controlFSMData, control_para, RL_WALK) {
    rl_controller_ = std::make_shared<RLController>();
}

bool FSM_State_RL_Walk::state_on_enter() {
    std::cout << YELLOW << "[FSM State]: Enter RL WALK.\n" << RESET;
    rl_controller_->init();
    return true;
}

void FSM_State_RL_Walk::state_on_exit() {
}

void FSM_State_RL_Walk::run_state() {
    // Demo Send CMD
    for (auto &leg: this->fsm_data_->leg_controller_->leg_command) {
        leg.kp_joint = Vec3<double>(1, 1, 1).asDiagonal();
        leg.kd_joint = Vec3<double>(1, 1, 1).asDiagonal();
        leg.q_des = Vec3<double>(0, 0, 0);
        leg.qd_des = Vec3<double>(0, 0, 0);
    }
    // Demo Get Data
    // 1. get all data.
    Vec19<double> joint_q;
    Vec18<double> joint_qd;
    Vec3<double> accel;
    //x y z | q_w q_x q_y q_z | joint data |
    get_joint_state(joint_q, joint_qd, accel);
    //please note the position and velocity respect to the inertial frame are null
    // 2. get from estimators
    (void) this->fsm_data_->estimators_->get_result_quat();
    (void) this->fsm_data_->estimators_->get_reult_acc_w();
    (void) this->fsm_data_->leg_controller_->leg_data[0].q(0);
    rl_controller_->step(&joint_q, &joint_qd, &accel);
}

bool FSM_State_RL_Walk::is_busy() {
    return false;
}
