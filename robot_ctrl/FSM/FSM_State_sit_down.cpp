#include "FSM_State_sit_down.h"
#include "iostream"
#include <algorithm>
#include <std_cout_colors.h>

#include "../../utilities/inc/Interpolation.h"

FSM_State_SitDown::FSM_State_SitDown(Control_FSM_Data *_controlFSMData, Control_Parameters_t *control_para)
    : FSM_State(_controlFSMData, control_para, SIT_DOWN) {
    joint_pos_ini_.resize(4);
    joint_pos_end_.resize(4);
}

bool FSM_State_SitDown::state_on_enter() {
    std::cout << YELLOW << "[FSM State]: Enter SitDown State.\n";
    state_iter_ = 0;
    double l1 = this->fsm_data_->quadruped_model_->get_hipLinkLength();
    double l2 = this->fsm_data_->quadruped_model_->get_kneeLinkLength();
    double h = Config::Stand_Up_Height;
    double h_down = Config::Sit_Down_Height;
    double theta1_down = acos((l1 * l1 + h_down * h_down - l2 * l2) / (2 * l1 * h_down));
    double theta2_down = -M_PI + acos((l1 * l1 + l2 * l2 - h_down * h_down) / (2 * l1 * l2));


    for (size_t leg(0); leg < 4; ++leg) {
        joint_pos_ini_[leg] = this->fsm_data_->leg_controller_->leg_data[leg].q;
        joint_pos_end_[leg][0] = 0;
        joint_pos_end_[leg][1] = theta1_down;
        joint_pos_end_[leg][2] = theta2_down;
    }
#if defined DG_ENGINEER
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
    const Vec3<double> kp(50, 50, 50);
    const Vec3<double> kd(2, 2, 2);
    for (int leg = 0; leg < 4; leg++) {
        this->fsm_data_->leg_controller_->leg_command[leg].kp_joint = kp.asDiagonal();
        this->fsm_data_->leg_controller_->leg_command[leg].kd_joint = kd.asDiagonal();
        //数据插值
        this->fsm_data_->leg_controller_->leg_command[leg].q_des
                = Interpolate::cubicBezier<Vec3<double> >(joint_pos_ini_[leg], joint_pos_end_[leg], t);
        this->fsm_data_->leg_controller_->leg_command[leg].qd_des
                = Interpolate::cubicBezierFirstDerivative<Vec3<double> >(joint_pos_ini_[leg], joint_pos_end_[leg], t) /
                  sit_down_time;
    }
}

bool FSM_State_SitDown::is_busy() {
    double t = (state_iter_ * this->fsm_para_->control_dt_) /
               (this->fsm_para_->sit_down_time_);
    return (t < 1.0);
}
