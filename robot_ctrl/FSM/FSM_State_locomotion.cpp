#include "FSM_State_locomotion.h"

#include <easylogging++.h>

#include "../config/robots_config.h"

FSM_State_Locomotion::FSM_State_Locomotion(Control_FSM_Data_t *controlFSMdata,
                                           Control_Parameters_t *control_para): FSM_State(
    controlFSMdata, control_para, LOCOMOTION) {
    wbc_ctrl_ = new LocomotionCtrl<double>(controlFSMdata->quadruped_model_);
    wbc_data_ = new LocomotionCtrlData<double>();
    // wbc_ctrl_->setFloatingBaseWeight(Config::wbc_weight_base);
    linear_mpc_ = new Linear_MPC(control_para->control_dt_, Config::mpc_iteration_segment);
    linear_planner_ = new Linear_Planner(control_para->control_dt_, Config::gait_iteration_segment);

    thread_mpc_ = new Thread::thread_mpc("Thread_mpc", Config::mpc_thread_fre);
}

bool FSM_State_Locomotion::state_on_enter() {
    std::cout << GREEN << "[FSM State]: " << RESET << "Enter Locomotion!\n";
    linear_mpc_->resetLinearMPC();
    linear_planner_->use_wbc_ = fsm_para_->use_wbc_;
    start_pos_ = this->fsm_data_->estimators_->get_result_world_position();
    const Vec3<double> start_rpy_ = ori::quatToRPY(this->fsm_data_->estimators_->get_result_quat());
    linear_planner_->set_start_rpy(start_rpy_);

    if (first_run_) {
        first_run_ = false;
        linear_mpc_->t_exit.store(false);
        linear_planner_->update_.store(false);
        mpc_exit_.store(false);
    }
    return true;
}

void FSM_State_Locomotion::state_on_exit() {
    linear_mpc_->t_exit.store(true);
    linear_planner_->firstRun_ = true;
    loco_thread.join();
    // linear_planner_->first_schedule_.store(false);
    first_run_ = true;
    state_iter_ = 0;
    draw_traj_request_ = false;
    mpc_thread_launched = false;
}

void FSM_State_Locomotion::thread_function(){
    std::cout << "[Thread MPC OK]: " << "Initialize MPC thread!\n";
    while (!linear_mpc_->t_exit.load()) {
        thread_mpc_->thread_enter_task();
        linear_mpc_->busy_.store(true);
        // wait for planner update.
        while (!linear_planner_->update_.load()) {
        };
        fsm_data_->fsm_data_mutex_.lock();
        linear_mpc_->SetupCommand(linear_planner_->desired_);
        fsm_data_->fsm_data_mutex_.unlock();
        linear_mpc_->run(*fsm_data_, linear_planner_->update_);
        linear_mpc_->busy_.store(false);
        thread_mpc_->thread_finish_task();
    }
    LOG(WARNING) << "MPC Thread Exit!";
    linear_mpc_->desire_data_.setDesireZero();
    fsm_data_->leg_controller_->Zero_Command();
    // loco_thread.join();
}

void FSM_State_Locomotion::run_state() {
    if (fsm_data_->rc_->rc_control_.variables[1] == 1 && !draw_traj_request_) {
        draw_traj_request_ = true;
    } else if (fsm_data_->rc_->rc_control_.variables[1] == 2 && draw_traj_request_) {
        draw_traj_request_ = false;
    }

    // start the mpc thread.
    linear_planner_->run(*fsm_data_);

    if (!mpc_thread_launched) {
        // std::unique_lock lk(linear_planner_->wait_mtx_);
        // linear_planner_->planner_cond_.wait(lk, [this] { return linear_planner_->first_schedule_.load(); });
        mpc_thread_launched = true;
        // tp_mpc_->Schedule([this] {
        //     thread_mpc_->thread_loop(linear_mpc_, *fsm_data_, linear_planner_->desired_,
        //                              linear_planner_->update_);
        //     mpc_exit_.store(true);
        // });
        // LOG(INFO) << "Here!";
        loco_thread = std::thread(&FSM_State_Locomotion::thread_function,this);
        // LOG(INFO) << "Here!!";
        mpc_exit_.store(true);
    }

    while (linear_mpc_->firstRun_){}; // wait for the first schedule.
    if (fsm_para_->use_wbc_) {
        state_iter_++;
        wbc_data_->pBody_RPY_des = linear_planner_->desired_.pBody_RPY_des_;
        wbc_data_->pBody_des = linear_planner_->desired_.pBody_des_;
        wbc_data_->vBody_des = linear_planner_->desired_.vBody_des_;
        wbc_data_->aBody_des = linear_planner_->desired_.aBody_des_;
        wbc_data_->vBody_Ori_des = linear_planner_->desired_.vBody_Ori_des_;
        for (int i = 0; i < 4; i++) {
            wbc_data_->pFoot_des[i] = linear_planner_->desired_.pFoot_des_[i];
            wbc_data_->vFoot_des[i] = linear_planner_->desired_.vFoot_des_[i];
            wbc_data_->aFoot_des[i] = linear_planner_->desired_.aFoot_des_[i];
            wbc_data_->Fr_des[i] = linear_mpc_->desire_data_.Fr_des_[i];
        }
        wbc_data_->contact_state = linear_planner_->contact_state_;
        // wbc_data_->contact_state << 1, 1, 1, 1;
        this->fsm_data_->estimators_->setContactPhase(wbc_data_->contact_state);

        // std::cout << "Contact State: " << wbc_data_->contact_state.transpose() << std::endl;
        Vec19<double> q_joint;
        Vec18<double> qd_joint;
        Vec3<double> q_acc; // q_acc = measure - 9.8
        this->get_joint_state(q_joint, qd_joint, q_acc); // the acc is not correct
        this->fsm_data_->quadruped_model_->update_model(q_joint, qd_joint, q_acc);
        wbc_ctrl_->run(wbc_data_, *fsm_data_);
    } else {
        for (int i = 0; i < 4; i++) {
            fsm_data_->leg_controller_->leg_command[i].foot_force = linear_mpc_->f_ff_[i];
        }
    }
}

// TODO add busy.
bool FSM_State_Locomotion::is_busy() {
    return linear_mpc_->busy_.load();
}
