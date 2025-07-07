//
// Created by lingwei on 6/14/24.
//
#include "FSM_State_damping.h"
#include <iostream>

FSM_State_Damping::FSM_State_Damping(Control_FSM_Data *_controlFSMData, Control_Parameters_t *control_para): FSM_State(
    _controlFSMData, control_para, DAMPING) {
}

bool FSM_State_Damping::state_on_enter() {
    std::cout << YELLOW << "[FSM State]: Enter Damping.\n" << RESET;
    return true;
}

void FSM_State_Damping::state_on_exit() {
    state_iter_ = 0;
}

void FSM_State_Damping::run_state() {
    Vec4<double> contactState;
    contactState << 0.5, 0.5, 0.5, 0.5;
    fsm_data_->estimators_->setContactPhase(contactState);
    const Vec3<double> kd(Config::damping_kd, Config::damping_kd, Config::damping_kd);
    fsm_data_->leg_controller_->Zero_Command();
    for (auto &leg: fsm_data_->leg_controller_->leg_command) {
        leg.kd_joint = kd.asDiagonal();
        leg.whl_kd_joint = Config::damping_kd;
    }
    state_iter_++;
}

bool FSM_State_Damping::is_busy() {
    return false;
}
