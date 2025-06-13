//
// Created by lingwei on 5/13/24.
//
#include "Control_FSM.h"

#include <easylogging++.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "../../utilities/inc/LoadData.h"

INITIALIZE_EASYLOGGINGPP

ControlFSM::ControlFSM(usb_controller::logic_remote_controller *rc,
                       Leg_Controller<double> *leg_control, StateEstimatorContainer<double> *stateesti) {
    control_data_.leg_controller_ = leg_control;
    control_data_.estimators_ = stateesti;
    control_data_.rc_ = rc;

    Get_Settings();

    state_list_.s_sitdown = new FSM_State_SitDown(&control_data_, &control_para_);
    state_list_.s_standup = new FSM_State_Stand_Up(&control_data_, &control_para_);
    state_list_.s_passive = new FSM_State_Passive(&control_data_, &control_para_);
    state_list_.s_damping = new FSM_State_Damping(&control_data_, &control_para_);
    state_list_.s_rl_walk = new FSM_State_RL_Walk(&control_data_, &control_para_);
    state_list_.s_rl_walk_2 = new FSM_State_RL_Walk_2(&control_data_, &control_para_);
    state_list_.s_user_interface = new FSM_State_User_Interface(&control_data_, &control_para_);
    //    std::cout << "ok\n";

    state_current_ = state_list_.s_passive;
    state_current_->state_on_enter();
    state_next_ = state_current_;
}

void ControlFSM::ControlFSM_run() {
    if (state_next_ == state_current_) {
        switch (state_current_->fsm_name_) {
            case PASSIVE:
                if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::RECOVER_STAND) {
                    state_list_.s_standup->fold_flag_ = true;
                    state_next_ = state_list_.s_standup;
                } else if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::USER_INTERFACE) {
                    state_next_ = state_list_.s_user_interface;
                } else { control_data_.rc_->rc_control_.mode = usb_controller::RC_MODE::PASSIVE; }
                break;
            case STAND_UP:
                if (state_current_->is_busy()) break;
                if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::SITDOWN) {
                    state_next_ = state_list_.s_sitdown;
                } else if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::DAMPING) {
                    state_next_ = state_list_.s_damping;
                } else if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::RL_WALK) {
                    state_next_ = state_list_.s_rl_walk;
                } else if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::RL_WALK_2) {
                    state_next_ = state_list_.s_rl_walk_2;
                } else {
                    control_data_.rc_->rc_control_.mode = usb_controller::RC_MODE::RECOVER_STAND;
                }
                break;
            case SIT_DOWN:
                if (state_current_->is_busy()) break;
                if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::PASSIVE) {
                    state_next_ = state_list_.s_passive;
                } else if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::RECOVER_STAND) {
                    state_list_.s_standup->fold_flag_ = true;
                    state_next_ = state_list_.s_standup;
                } else {
                    control_data_.rc_->rc_control_.mode = usb_controller::RC_MODE::SITDOWN;
                }
                break;
            case DAMPING:
                if (state_current_->is_busy()) break;
                if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::RECOVER_STAND) {
                    state_list_.s_standup->fold_flag_ = true;
                    state_next_ = state_list_.s_standup;
                } else if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::PASSIVE) {
                    state_next_ = state_list_.s_passive;
                } else {
                    control_data_.rc_->rc_control_.mode = usb_controller::RC_MODE::DAMPING;
                }
                break;
            case RL_WALK:
                if (state_current_->is_busy()) break;
                if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::RECOVER_STAND) {
                    state_list_.s_standup->fold_flag_ = false;
                    state_next_ = state_list_.s_standup;
                } else if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::DAMPING) {
                    state_next_ = state_list_.s_damping;
                } else {
                    control_data_.rc_->rc_control_.mode = usb_controller::RC_MODE::RL_WALK;
                }
                break;
            case RL_WALK_2:
                if (state_current_->is_busy()) break;
                if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::RECOVER_STAND) {
                    state_list_.s_standup->fold_flag_ = false;
                    state_next_ = state_list_.s_standup;
                } else if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::DAMPING) {
                    state_next_ = state_list_.s_damping;
                } else {
                    control_data_.rc_->rc_control_.mode = usb_controller::RC_MODE::RL_WALK_2;
                }
                break;
            case USER_INTERFACE:
                if (state_current_->is_busy())break;
                if (control_data_.rc_->rc_control_.mode == usb_controller::RC_MODE::DAMPING) {
                    state_next_ = state_list_.s_damping;
                }
            default:
                break;
        }
    }

    // safety check
    for (auto &i: control_data_.leg_controller_->leg_data) {
        for (int j = 0; j < 3; j++) {
            if (i.qd(j) > Config::qd_danger) {
                danger_times_++;
            }
        }
    }

    if (danger_times_ > 10) {
        LOG(WARNING) << "Reach the danger velocity!";
        state_next_ = state_list_.s_damping;
        danger_times_ = 0;
    }

    // if (state_current_->state_iter_ % 100 == 0) {
    //     LOG(INFO) << "Current State: " << state_current_->fsm_name_;
    // }

    // switch state
    if (state_next_ != state_current_) {
        if (!state_current_->is_busy()) {
            state_current_->state_on_exit();
            if (state_next_->state_on_enter()) {
                state_current_ = state_next_;
            } else { state_next_ = state_current_; }
        }
    }
    state_current_->run_state();
}

void ControlFSM::Get_Settings() {
    const std::string setting_name = "State_Parameters";
    const std::string file_name = Config::path_2_config_directory + "config/Control_Parameters.info";
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(file_name, pt);
    loadData::loadEigenMatrix(file_name, "kp_stand_vec", control_para_.kp_stand_);
    loadData::loadEigenMatrix(file_name, "kd_stand_vec", control_para_.kd_stand_);
    loadData::loadPtreeValue(pt, control_para_.control_dt_, setting_name + ".control_dt", true);
    loadData::loadPtreeValue(pt, control_para_.stand_time_, setting_name + ".stand_up_time", true);
    loadData::loadPtreeValue(pt, control_para_.sit_down_time_, setting_name + ".sit_down_time", true);
    loadData::loadPtreeValue(pt, control_para_.use_wbc_, setting_name + ".use_wbc", true);
}
