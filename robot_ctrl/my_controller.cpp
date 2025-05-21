//
// Created by lingwei on 5/13/24.
//

#include "my_controller.h"


void My_Controller::Controller_Init() {
    controlfsm_ = new ControlFSM(ctrl_rc_, leg_controller_, estimators_);
}

void My_Controller::run() {
    controlfsm_->ControlFSM_run();
    if (controlfsm_->state_current_->draw_traj_request_) {
        control_draw_cond_.store(true);
    } else {
        control_draw_cond_.store(false);
    }
    // std::cout << "Control_draw_cond: " << control_draw_cond_.load() << std::endl;
}
