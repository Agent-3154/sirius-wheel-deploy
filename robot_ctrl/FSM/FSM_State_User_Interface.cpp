#include "FSM_State_User_Interface.h"
#include "../../utilities/inc/debug_tools.h"
#include <memory>
#include <iostream>

FSM_State_User_Interface::FSM_State_User_Interface(Control_FSM_Data_t *controlFSMdata, Control_Parameters_t *control_para)
    : FSM_State(controlFSMdata, control_para, USER_INTERFACE),
      iceoryx_state_publisher_({"ROBOT", "REAL", "ROBOT_STATE"}),
      iceoryx_motor_subscriber_({"ROBOT", "REAL", "MOTOR_CMD"}) {
    this->safty_check_ = false;
    user_interface_timer_ = std::make_shared<Thread::thread_timer>("User Interface Timer", 2000);
    exit_state_.store(false);
}

bool FSM_State_User_Interface::state_on_enter() {
    subscriber_thread_ = std::thread(&FSM_State_User_Interface::subscriber_thread_func, this);
    exit_state_.store(false);
    return true;
}

void FSM_State_User_Interface::state_on_exit() {
    exit_state_.store(true);
    state_iter_ = 0;
}

void FSM_State_User_Interface::run_state() {
    // first publish the state
    get_joint_state(state_q_, state_qd_, state_accel);
    iceoryx_state_publisher_.loan()
            .and_then([this](auto &sample) {
                for (int i = 0; i < 3; i++) {
                    sample->acc[i] = state_accel(i);
                    sample->gyro[i] = state_qd_(i + 3);
                    sample->quat[i] = state_q_(i + 3);
                }
                sample->quat[3] = state_q_(6);
                for (int i = 0; i < 12; i++) {
                    sample->q[i] = state_q_(i + 7);
                    sample->qd[i] = state_qd_(i + 6);
                }
                sample.publish();
            })
            .or_else([](auto &result) {
                std::cerr << "Unable to loan sample, error: " << result << std::endl;
            });

    // second execute the command
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (int i = 0; i < 4; i++) {
            this->fsm_data_->leg_controller_->leg_command[i].q_des = Vec3<double>(q_des[3 * i], q_des[3 * i + 1], q_des[3 * i + 2]);
            this->fsm_data_->leg_controller_->leg_command[i].qd_des = Vec3<double>(qd_des[3 * i], qd_des[3 * i + 1], qd_des[3 * i + 2]);
            this->fsm_data_->leg_controller_->leg_command[i].kp_joint = Vec3<double>(kp_joint[3 * i], kp_joint[3 * i + 1], kp_joint[3 * i + 2]).asDiagonal();
            this->fsm_data_->leg_controller_->leg_command[i].kd_joint = Vec3<double>(kd_joint[3 * i], kd_joint[3 * i + 1], kd_joint[3 * i + 2]).asDiagonal();
            this->fsm_data_->leg_controller_->leg_command[i].tau_ff = Vec3<double>(tau_ff[3 * i], tau_ff[3 * i + 1], tau_ff[3 * i + 2]);
        }
    }
}

bool FSM_State_User_Interface::is_busy() {
    return false;
}

void FSM_State_User_Interface::subscriber_thread_func() {
    while (!exit_state_.load()) {
        user_interface_timer_->thread_enter_task();
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        iceoryx_motor_subscriber_.take().and_then([this](auto &sample) {
            for (int i = 0; i < 18; i++) {
                q_des[i] = sample->q[i];
                qd_des[i] = sample->qd[i];
                kp_joint[i] = sample->kp[i];
                kd_joint[i] = sample->kd[i];
                tau_ff[i] = sample->tau_ff[i];
            }
        }).or_else([](auto &result) {
            if (result != iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE) {
                std::cout << "Error receiving chunk." << std::endl;
            }
        });
        user_interface_timer_->thread_finish_task();
    }
}
