#include "FSM_State_stand_up.h"
#include "../../utilities/inc/debug_tools.h"
#include <algorithm>
#include <iostream>
#include "../../utilities/inc/Interpolation.h"

FSM_State_Stand_Up::FSM_State_Stand_Up(Control_FSM_Data_t *controlFSMdata, Control_Parameters_t *control_para)
    : FSM_State(controlFSMdata, control_para, STAND_UP) {
    joint_pos_ini_.resize(4);
    joint_pos_stand_.resize(4);
    joint_pos_fold_.resize(4);
    this->safty_check_ = false;
}

bool FSM_State_Stand_Up::state_on_enter() {
    std::cout << YELLOW << "[FSM State]: Enter Stand State.\n" << RESET;
    state_iter_ = 0;
    double l1 = Config::HipLinkLength;
    double l2 = Config::KneeLinkLength;
    double h = Config::Stand_Up_Height;
    double end_theta1 = acos((l1 * l1 + h * h - l2 * l2) / (2 * l1 * h));
    double end_theta2 = -M_PI + acos((l1 * l1 + l2 * l2 - h * h) / (2 * l1 * l2));
#if defined DG_ENGINEER
    double end_theta_hind = -end_theta1;
    double end_theta_hind2 = -end_theta2;
#endif

#if defined SIRIUS_WHEEL
    double end_theta_hind_whl = -end_theta1;
    double end_theta_hind2_whl = -end_theta2;
#endif

    //求趴下时，hip与knee
    h = Config::Sit_Down_Height;
    double mid_theta1 = acos((l1 * l1 + h * h - l2 * l2) / (2 * l1 * h));
    double mid_theta2 = -M_PI + acos((l1 * l1 + l2 * l2 - h * h) / (2 * l1 * l2));
#if defined DG_ENGINEER
    double mid_theta_hind = -mid_theta1;
    double mid_theta_hind2 = -mid_theta2;
#endif

#if defined SIRIUS_WHEEL
    double mid_theta_hind_whl = -mid_theta1;
    double mid_theta_hind2_whl = -mid_theta2;
#endif

    for (size_t leg(0); leg < 4; ++leg) {
        joint_pos_ini_[leg] = this->fsm_data_->leg_controller_->leg_data[leg].q;
        joint_pos_fold_[leg][0] = 0;
        joint_pos_fold_[leg][1] = mid_theta1;
        joint_pos_fold_[leg][2] = mid_theta2;
        joint_pos_stand_[leg][0] = 0;
        joint_pos_stand_[leg][1] = end_theta1;
        joint_pos_stand_[leg][2] = end_theta2;
        //        std::cout << "Initial Pos: " << leg  << " " <<joint_pos_ini_[leg].transpose() << std::endl;
    }
#if defined DG_ENGINEER
    for (size_t leg(2); leg < 4; ++leg) {
        joint_pos_ini_[leg] = this->fsm_data_->leg_controller_->leg_data[leg].q;
        joint_pos_fold_[leg][0] = 0;
        joint_pos_fold_[leg][1] = mid_theta_hind;
        joint_pos_fold_[leg][2] = mid_theta_hind2;
        joint_pos_stand_[leg][0] = 0;
        joint_pos_stand_[leg][1] = end_theta_hind;
        joint_pos_stand_[leg][2] = end_theta_hind2;
        //        std::cout << "Initial Pos: " << leg  << " " <<joint_pos_ini_[leg].transpose() << std::endl;
    }
#endif

#if defined SIRIUS_WHEEL
    for (size_t leg(2); leg < 4; ++leg) {
        joint_pos_ini_[leg] = this->fsm_data_->leg_controller_->leg_data[leg].q;
        joint_pos_fold_[leg][0] = 0;
        joint_pos_fold_[leg][1] = mid_theta_hind_whl;
        joint_pos_fold_[leg][2] = mid_theta_hind2_whl;
        joint_pos_stand_[leg][0] = 0;
        joint_pos_stand_[leg][1] = end_theta_hind_whl;
        joint_pos_stand_[leg][2] = end_theta_hind2_whl;
        //        std::cout << "Initial Pos: " << leg  << " " <<joint_pos_ini_[leg].transpose() << std::endl;
    }
#endif

    // if (fsm_data_->estimators_->get_result_world_position()(2) < 0.2) {
        // fold_flag_ = true;
    // }
    return true;
}

void FSM_State_Stand_Up::state_on_exit() {
    state_iter_ = 0;
    fold_flag_ = false;
}

void FSM_State_Stand_Up::run_state() {
    Vec4<double> constactState;
    constactState << 0.5, 0.5, 0.5, 0.5;
    this->fsm_data_->estimators_->setContactPhase(constactState);
    state_iter_++;


    const Vec3<double> kp(this->fsm_para_->kp_stand_(0), this->fsm_para_->kp_stand_(1),
                          this->fsm_para_->kp_stand_(2));
    const Vec3<double> kd(this->fsm_para_->kd_stand_(0), this->fsm_para_->kd_stand_(1),
                          this->fsm_para_->kd_stand_(2));

    if (fold_flag_) {
        const double t_stand = this->fsm_para_->stand_time_;
        double t = state_iter_ * this->fsm_para_->control_dt_ / t_stand;
        t = std::min(t, 1.);
        if (t < 1.0 / 3.0) {
            const double t1 = t / (1.F / 3.F);
            for (int leg = 0; leg < 4; leg++) {
                this->fsm_data_->leg_controller_->leg_command[leg].kp_joint = kp.asDiagonal();
                this->fsm_data_->leg_controller_->leg_command[leg].kd_joint = kd.asDiagonal();
                this->fsm_data_->leg_controller_->leg_command[leg].q_des
                        = Interpolate::cubicBezier<Vec3<double> >(joint_pos_ini_[leg], joint_pos_fold_[leg], t1);
                this->fsm_data_->leg_controller_->leg_command[leg].qd_des
                        = Interpolate::cubicBezierFirstDerivative<Vec3<double> >(
                              joint_pos_ini_[leg], joint_pos_fold_[leg],
                              t1) / (1.0 / 3.0 * t_stand);
            }
        }

        if (t >= 1.0 / 3.0) {
            const double t2 = (t - 1.0 / 3.0) / (2.0 / 3.0);
            for (int leg = 0; leg < 4; leg++) {
                this->fsm_data_->leg_controller_->leg_command[leg].kp_joint = kp.asDiagonal();
                this->fsm_data_->leg_controller_->leg_command[leg].kd_joint = kd.asDiagonal();
                this->fsm_data_->leg_controller_->leg_command[leg].q_des
                        = Interpolate::cubicBezier<Vec3<double> >(joint_pos_fold_[leg], joint_pos_stand_[leg], t2);
                this->fsm_data_->leg_controller_->leg_command[leg].qd_des
                        = Interpolate::cubicBezierFirstDerivative<Vec3<double> >(
                              joint_pos_fold_[leg], joint_pos_stand_[leg],
                              t2) / (2.f / 3.f * t_stand);
            }
        }
        for (int leg = 0; leg < 4; leg++) {
            this->fsm_data_->leg_controller_->leg_command[leg].whl_kp_joint = 0.0;
            this->fsm_data_->leg_controller_->leg_command[leg].whl_kd_joint = 1.0;
            this->fsm_data_->leg_controller_->leg_command[leg].whl_qd_des = 0.0;
            this->fsm_data_->leg_controller_->leg_command[leg].whl_tau_ff = 0.0;
      }
    } else {
        constexpr double t_stand = 1;
        double t = state_iter_ * this->fsm_para_->control_dt_ / t_stand;
        t = std::min(t, 1.);
        for (int leg = 0; leg < 4; leg++) {
            this->fsm_data_->leg_controller_->leg_command[leg].kp_joint = kp.asDiagonal();
            this->fsm_data_->leg_controller_->leg_command[leg].kd_joint = kd.asDiagonal();
            this->fsm_data_->leg_controller_->leg_command[leg].q_des
                    = Interpolate::cubicBezier<Vec3<double> >(joint_pos_ini_[leg], joint_pos_stand_[leg], t);
            this->fsm_data_->leg_controller_->leg_command[leg].qd_des
                    = Interpolate::cubicBezierFirstDerivative<Vec3<double> >(
                        joint_pos_ini_[leg], joint_pos_stand_[leg], t);
        }

      for (int leg = 0; leg < 4; leg++) {
        this->fsm_data_->leg_controller_->leg_command[leg].whl_kp_joint = 0.0;
        this->fsm_data_->leg_controller_->leg_command[leg].whl_kd_joint = 1.0;
        this->fsm_data_->leg_controller_->leg_command[leg].whl_qd_des = 0.0;
        this->fsm_data_->leg_controller_->leg_command[leg].whl_tau_ff = 0.0;
      }
    }
}

bool FSM_State_Stand_Up::is_busy() {
    double ret = (state_iter_ * this->fsm_para_->control_dt_) / (this->fsm_para_->stand_time_);
    return (ret < 1.F);
}
