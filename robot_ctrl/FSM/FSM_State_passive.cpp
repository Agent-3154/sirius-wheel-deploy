#include "FSM_State_passive.h"
#include <iostream>

FSM_State_Passive::FSM_State_Passive(Control_FSM_Data_t *controlFSMData, Control_Parameters_t *control_para)
        : FSM_State(controlFSMData, control_para, PASSIVE) {
    this->safty_check_ = false;
}

bool FSM_State_Passive::state_on_enter() {
    std::cout << YELLOW << "[FSM State]: Enter Passive State.\n";
    return true;
}

void FSM_State_Passive::state_on_exit() {
    state_iter_ = 0;
}

// maybe mutex lock is unnecessary
void FSM_State_Passive::run_state() {
    Vec4<double> contactState;
    contactState << 0.5, 0.5, 0.5, 0.5;
    this->fsm_data_->estimators_->setContactPhase(contactState);
    this->fsm_data_->leg_controller_->Zero_Command();
    state_iter_++;
}

bool FSM_State_Passive::is_busy() {
    return false;
}




