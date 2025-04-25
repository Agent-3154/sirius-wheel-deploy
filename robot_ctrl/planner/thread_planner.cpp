//
// Created by lingwei on 6/8/24.
//
#include "thread_planner.h"

Thread::thread_planner::thread_planner(const std::string &task_name, int task_frequency): thread_timer(
    task_name, task_frequency) {
}

// void Thread::thread_planner::thread_loop(Planner_Base<double> *handle, Control_FSM_Data &data) {
//     std::cout << "[Thread Planner OK]: " << "Initialize Planner thread!\n";
//     // while (!handle->t_exit_.load()) {
//     //     this->thread_enter_task();
//     //     handle->busy.store(true);
//     //     handle->run(data);
//     //     handle->busy.store(false);
//     //     this->thread_finish_task();
//     //     handle->first_schedule_.store(true);
//     //     handle->planner_cond_.notify_all();
//     // }
// }
