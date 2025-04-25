#include "FSM_State_external.h"
#include "../../utilities/inc/debug_tools.h"
#include <algorithm>
#include <iostream>

FSM_State_Extern::FSM_State_Extern(Control_FSM_Data_t *controlFSMdata, Control_Parameters_t *control_para)
    : FSM_State(controlFSMdata, control_para, EXTERNAL) {
    this->safty_check_ = false;
}

bool FSM_State_Extern::state_on_enter() {
    std::cout << YELLOW << "[FSM State]: Enter External State.\n" << RESET;
    // initialize threads here.
    return true;
}

void FSM_State_Extern::state_on_exit() {
    state_iter_ = 0;
}

void FSM_State_Extern::run_state() {
  // ************ get orientation ****************************
  // this->fsm_data_->estimators_->get_result_quat();

  // ********** get velocity *********************************
  //  this->fsm_data_->estimators_->get_result_angular_body();

  // ********* get acceleration ******************************
  //  this->fsm_data_->estimators_->get_reult_acc_w();

  // ****** can also get full state **************************
  // q: p_w, q_w, q_x, q_y, q_z + 12 motors
  //  this->get_joint_state(fsm_data_->joint_q_, fsm_data_->joint_qd_, fsm_data_->accel_);

  // ************** hand out cmds ****************************
//  auto *motor_cmd = fsm_data_->leg_controller_;
//   motor_cmd->Zero_Command();
//   for (int leg = 0; leg < Config::num_legs; ++leg) {
//        //        std::cout << "leg id: "<< leg << std::endl;
//        for (int jidx = 0; jidx < Config::num_joints_on_leg; ++jidx) {
//            cmd->leg_command[leg].tau_ff[jidx] = tau_ff_[Config::num_joints_on_leg * leg + jidx];
//            cmd->leg_command[leg].q_des[jidx] = des_jpos_[Config::num_joints_on_leg * leg + jidx];
//            cmd->leg_command[leg].qd_des[jidx] = des_jvel_[Config::num_joints_on_leg * leg + jidx];
//            cmd->leg_command[leg].kp_joint(jidx, jidx) = kp_joint_[jidx];
//            cmd->leg_command[leg].kd_joint(jidx, jidx) = kd_joint_[jidx];
//            //            std::cout << "tau | des_p | de_v : " <<tau_ff_[Config::num_joints_on_leg * leg + jidx] << " | " << des_jpos_[Config::num_joints_on_leg * leg + jidx]
//            //            << " | " <<des_jvel_[Config::num_joints_on_leg * leg + jidx] << std::endl;
//        }
//    }

    state_iter_++;
}

bool FSM_State_Extern::is_busy() {
    return false;
}
