#include "FSM_State_bs.h"

#include "../../config/robots_config.h"

FSM_State_BS::FSM_State_BS(Control_FSM_Data_t *controlFSMdata, Control_Parameters_t *control_para) : FSM_State(
    controlFSMdata, control_para, BALANCE_STAND) {
    wbc_ctrl_ = new LocomotionCtrl<double>(controlFSMdata->quadruped_model_);
    wbc_data_ = new LocomotionCtrlData<double>();
    wbc_ctrl_->setFloatingBaseWeight(1000.);
    rpy_last_.setZero();
}

bool FSM_State_BS::state_on_enter() {
    std::cout << YELLOW << "[FSM State]: Enter Balance Stand.\n" << RESET;
    total_weight_ = fsm_data_->quadruped_model_->robotTotalMass_ * Config::G;
    // total_weight_ = 26.7 * Config::G;
    start_pos_ = this->fsm_data_->estimators_->get_result_world_position();
    start_pos_(2) = Config::mpc_height;
    rpy_last_.setZero();
    start_rpy_ = ori::quatToRPY(this->fsm_data_->estimators_->get_result_quat());
    // rpy_last_ = start_rpy_;
    std::cout << "start_rpy: " << start_rpy_.transpose() << std::endl;
    return true;
}

void FSM_State_BS::state_on_exit() {
    state_iter_ = 0;
}

void FSM_State_BS::run_state() {
    Vec4<double> contactState;
    contactState << 0.5, 0.5, 0.5, 0.5;
    this->fsm_data_->estimators_->setContactPhase(contactState);
    wbc_data_->pBody_RPY_des.setZero();
    // note here
    Vec3<double> ori_des;
    for (int i = 0; i < 3; i++) {
        ori_des(i) = this->fsm_data_->rc_->rc_control_.rpy_des[i] * Config::bs_rpy_filter + (1 - Config::bs_rpy_filter)
                     * rpy_last_(i);
        // rpy_last_[i] = wbc_data_->pBody_RPY_des[i];
    }
    rpy_last_ = ori_des;
    // std::cout << wbc_data_->pBody_RPY_des.transpose() << std::endl;
    // std::cout << wbc_data_->pBody_RPY_des.transpose() << std::endl;
    wbc_data_->pBody_des = start_pos_;
    // const double angle = sqrt(ori_des[0] * ori_des[0] / 0.6 / 0.6 + ori_des[1] * ori_des[1] / 0.2 / 0.2);
    // const double length = this->fsm_data_->quadruped_model_->bodyLength_ +
    //                       this->fsm_data_->quadruped_model_->bodyWidth_ +
    //                       this->fsm_data_->quadruped_model_->abadLinkLength_;
    // wbc_data_->pBody_des[2] -= 0.1 * sin(fabs(angle)) * length;
    wbc_data_->vBody_des.setZero();
    wbc_data_->aBody_des.setZero();
    wbc_data_->vBody_Ori_des.setZero();
    wbc_data_->pBody_RPY_des = ori_des + start_rpy_;
    // Orientation
    for (int i = 0; i < 4; i++) {
        wbc_data_->pFoot_des[i].setZero();
        wbc_data_->vFoot_des[i].setZero();
        wbc_data_->aFoot_des[i].setZero();
        wbc_data_->Fr_des[i].setZero();
        wbc_data_->Fr_des[i][2] = total_weight_ / 4.;
        wbc_data_->contact_state[i] = true;
    }
    Vec19<double> q_joint;
    Vec18<double> qd_joint;
    Vec3<double> q_acc; // q_acc = measure - 9.8
    this->get_joint_state(q_joint, qd_joint, q_acc); // the acc is not correct
    //    std::cout << "qd_joint: " <<q_joint.transpose() << std::endl;
    this->fsm_data_->quadruped_model_->update_model(q_joint, qd_joint, q_acc);
    wbc_ctrl_->run(wbc_data_, *this->fsm_data_);
}

bool FSM_State_BS::is_busy() {
    return false;
}
