#include "FSM_State_sit_down.h"
#include "iostream"
#include <algorithm>
#include <std_cout_colors.h>
#include "../../utilities/inc/Interpolation.h"
#include "../../config/robots_config.h"

FSM_State_SitDown::FSM_State_SitDown(Control_FSM_Data *_controlFSMData, Control_Parameters_t *control_para)
        : FSM_State(_controlFSMData, control_para, SIT_DOWN) {
    joint_pos_ini_.resize(4);
    joint_pos_end_.resize(4);
}

bool FSM_State_SitDown::state_on_enter() {
    std::cout << YELLOW << "[FSM State]: Enter SitDown State.\n";
    state_iter_ = 0;
    double l1 = Config::HipLinkLength;
    double l2 = Config::KneeLinkLength;
    double h = Config::Stand_Up_Height;
    double theta1 = acos((l1 * l1 + h * h - l2 * l2) / (2 * l1 * h));
    double theta2 = -M_PI + acos((l1 * l1 + l2 * l2 - h * h) / (2 * l1 * l2));
    double h_down = Config::Sit_Down_Height;
    double theta1_down = acos((l1 * l1 + h_down * h_down - l2 * l2) / (2 * l1 * h_down));
    double theta2_down = -M_PI + acos((l1 * l1 + l2 * l2 - h_down * h_down) / (2 * l1 * l2));
    for (size_t leg(0); leg < 4; ++leg) {
        joint_pos_ini_[leg] = this->fsm_data_->leg_controller_->leg_data[leg].q;
        joint_pos_end_[leg][0] = 0;
        joint_pos_end_[leg][1] = theta1_down;
        joint_pos_end_[leg][2] = theta2_down;
    }
#if defined(DG_ENGINEER) || defined(SIRIUS_WHEEL)
    for (size_t leg(2); leg < 4; ++leg) {
        joint_pos_end_[leg][0] = 0;
        joint_pos_end_[leg][1] = -theta1_down;
        joint_pos_end_[leg][2] = -theta2_down;
    }
#endif
    return true;
}

void FSM_State_SitDown::state_on_exit() {

}

void FSM_State_SitDown::run_state() {
    Vec4<double> contactState;
    contactState << 0.5, 0.5, 0.5, 0.5;
    this->fsm_data_->estimators_->setContactPhase(contactState);

    state_iter_++;

    double sit_down_time = this->fsm_para_->sit_down_time_;
    double t = ((double) state_iter_ * this->fsm_para_->control_dt_) / sit_down_time;

    t = std::min(t, 1.0);
    Vec3<double> kp(this->fsm_para_->kp_stand_(0),
                   this->fsm_para_->kp_stand_(1),
                   this->fsm_para_->kp_stand_(2));
    Vec3<double> kd(this->fsm_para_->kd_stand_(0),
                   this->fsm_para_->kd_stand_(1),
                   this->fsm_para_->kd_stand_(2));
    for (int leg = 0; leg < 4; leg++) {
        this->fsm_data_->leg_controller_->leg_command[leg].kp_joint = kp.asDiagonal();
        this->fsm_data_->leg_controller_->leg_command[leg].kd_joint = kd.asDiagonal();
        //数据插值
        this->fsm_data_->leg_controller_->leg_command[leg].q_des
                = Interpolate::cubicBezier<Vec3<double>>(joint_pos_ini_[leg], joint_pos_end_[leg], t);
        this->fsm_data_->leg_controller_->leg_command[leg].qd_des
                = Interpolate::cubicBezierFirstDerivative<Vec3<double>>(joint_pos_ini_[leg], joint_pos_end_[leg], t) /
                  sit_down_time;
    }
    for (int leg = 0; leg < 4; leg++) {
        this->fsm_data_->leg_controller_->leg_command[leg].whl_kp_joint = 0.0;
        this->fsm_data_->leg_controller_->leg_command[leg].whl_kd_joint = 1.0;
        this->fsm_data_->leg_controller_->leg_command[leg].whl_qd_des = 0.0;
        this->fsm_data_->leg_controller_->leg_command[leg].whl_tau_ff = 0.0;
      }
}

bool FSM_State_SitDown::is_busy() {
    double t = (state_iter_ * this->fsm_para_->control_dt_) /
              (this->fsm_para_->sit_down_time_);
    return (t < 1.0);
}
