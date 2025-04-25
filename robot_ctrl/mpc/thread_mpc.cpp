//
// Created by lingwei on 6/8/24.
//
#include "thread_mpc.h"

#include <easylogging++.h>


Thread::thread_mpc::thread_mpc(const std::string &task_name, int task_frequency): thread_timer(
    task_name, task_frequency) {
}

void Thread::thread_mpc::thread_loop(MPC_Base *handle, Control_FSM_Data &data, const planner_desire<double> &p_data,
                                     std::atomic_bool &update) {
    std::cout << "[Thread MPC OK]: " << "Initialize MPC thread!\n";
    while (!handle->t_exit.load()) {
        this->thread_enter_task();
        handle->busy_.store(true);
        // wait for planner update.
        while (!update.load()) {
        };
        handle->SetupCommand(p_data);
        handle->run(data, update);
        handle->busy_.store(false);
        this->thread_finish_task();
    }
    LOG(WARNING) << "MPC Thread Exit!";
    handle->desire_data_.setDesireZero();
    data.leg_controller_->Zero_Command();
}
