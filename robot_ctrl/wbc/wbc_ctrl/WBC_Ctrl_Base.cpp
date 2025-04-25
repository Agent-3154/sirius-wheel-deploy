//
// Created by lingwei on 5/20/24.
//
#include "WBC_Ctrl_Base.h"
#include "../../config/Config.h"
#include <memory>
#include <std_cout_colors.h>

template<typename T>
WBC_Ctrl_Base<T>::WBC_Ctrl_Base(Quadruped_Base *quadruped_model): full_config_(Config::num_act_joints + 6),
                                                                  tau_ff_(Config::num_act_joints),
                                                                  des_jpos_(Config::num_act_joints),
                                                                  des_jvel_(Config::num_act_joints) {
    tau_ff_.setZero();
    iter_ = 0;
    full_config_.setZero();
    robot_model_ = quadruped_model;
    kin_wbc_ = std::make_shared<KinWBC<T>>(Config::num_dim_config);
    wbic_ = std::make_shared<WBIC<T>>(Config::num_dim_config, &contact_list_, &task_list_);
    wbic_extra_data_ = new WBIC_ExtraData<T>();
    wbic_extra_data_->w_floating_ = DVec<T>::Constant(6, 0.1);
    wbic_extra_data_->w_rf_ = DVec<T>::Constant(12, 1.);

    kp_joint_.resize(Config::num_joints_on_leg, 5.0);
    kd_joint_.resize(Config::num_joints_on_leg, 1.5);
}

template<typename T>
WBC_Ctrl_Base<T>::~WBC_Ctrl_Base() {
    delete wbic_extra_data_;
    auto iter = task_list_.begin();
    while (iter < task_list_.end()) {
        delete *iter;
        iter++;
    }
    task_list_.clear();

    auto iter_1 = contact_list_.begin();
    while (iter_1 < contact_list_.end()) {
        delete *iter_1;
        iter_1++;
    }
    contact_list_.clear();
}

/**
 * @note call this after updating the model
 * @tparam T
 * @param input
 * @param ctrl_data
 */
template<typename T>
void WBC_Ctrl_Base<T>::run(void *input, Control_FSM_Data &ctrl_data) {
    UpdateModel();
    ContactTaskUpdate(input, ctrl_data);
    ComputeWBC();
    UpdateLegCMD(ctrl_data);
    LCM_PublishData();
}

template<typename T>
void WBC_Ctrl_Base<T>::setFloatingBaseWeight(const T &weight) {
    wbic_extra_data_->w_floating_ = DVec<T>::Constant(6, weight);
}

template<typename T>
void WBC_Ctrl_Base<T>::UpdateModel() {
    robot_model_->get_Mq_Matrix(Mq_);
    robot_model_->get_Cqqd_Matrix(Cqqd_);
    // std::cout << "M:\n" << std::setprecision(3) << Mq_ << "\n Cqqd\n:" << Cqqd_.transpose() << std::endl;
    Mqinv_ = Mq_.inverse();
    for (int i = 0; i < 3; i++) {
        for (int leg = 0; leg < 4; leg++) {
            full_config_[3 * leg + i + 6] = robot_model_->q_pos_[3 * leg + i + Config::abad_pos_addr_offset];
        }
    }
    // std::cout << "Full config: " << full_config_.transpose() << std::endl;
}

template<typename T>
void WBC_Ctrl_Base<T>::UpdateLegCMD(Control_FSM_Data &data) {
    Leg_Controller<T> *cmd = data.leg_controller_;
    cmd->Zero_Command();
    for (int leg = 0; leg < Config::num_legs; ++leg) {
//        std::cout << "leg id: "<< leg << std::endl;
        for (int jidx = 0; jidx < Config::num_joints_on_leg; ++jidx) {
            if (isinf(tau_ff_[Config::num_joints_on_leg * leg + jidx])) {
                tau_ff_[Config::num_joints_on_leg * leg + jidx] = 0;
                std::cout << BOLDRED << "[WBC INF]: " << RESET << " torque out put infinity!\n";
            }
            cmd->leg_command[leg].tau_ff[jidx] = tau_ff_[Config::num_joints_on_leg * leg + jidx];
            cmd->leg_command[leg].q_des[jidx] = des_jpos_[Config::num_joints_on_leg * leg + jidx];
            cmd->leg_command[leg].qd_des[jidx] = des_jvel_[Config::num_joints_on_leg * leg + jidx];
            cmd->leg_command[leg].kp_joint(jidx, jidx) = kp_joint_[jidx];
            cmd->leg_command[leg].kd_joint(jidx, jidx) = kd_joint_[jidx];
//            std::cout << "tau | des_p | de_v : " <<tau_ff_[Config::num_joints_on_leg * leg + jidx] << " | " << des_jpos_[Config::num_joints_on_leg * leg + jidx]
//            << " | " <<des_jvel_[Config::num_joints_on_leg * leg + jidx] << std::endl;
        }
    }

    // T min_knee_pos = ori::deg2rad(66.261);
    // T danger_dis = ori::deg2rad(3.0);
    // for (size_t leg(0); leg < Config::num_legs; ++leg) {
    //     if (cmd->leg_command[leg].q_des[2] < min_knee_pos) {
    //         cmd->leg_command[leg].q_des[2] = min_knee_pos;
    //     }
    //
    //     T dis = (T) fabs(data.leg_controller_->leg_data[leg].q[2] - min_knee_pos);
    //     if (dis < danger_dis) {
    //         cmd->leg_command[leg].tau_ff[2] = 1. / (dis * dis * 500.0 + 1.0 / 36.0);
    //     }
    // }
    // wbc首次运行会出现随机数
     static bool first_run = true;
     if (first_run) {
         first_run = false;
         cmd->Zero_Command();
     }
}

template<typename T>
void WBC_Ctrl_Base<T>::ComputeWBC() {
    kin_wbc_->FindConfiguration(full_config_, task_list_, contact_list_, des_jpos_, des_jvel_);
    wbic_->UpdateSettings(Mq_, Mqinv_, Cqqd_, nullptr);
    wbic_->MakeTorque(tau_ff_, wbic_extra_data_);
}

template
class WBC_Ctrl_Base<double>;
