//
// Created by lingwei on 5/6/24.
//

#include "../inc/debug_tools.h"
#include "iostream"

Debugging::test_timer::test_timer(int timer_id, double warning_us): timer_id_(timer_id), warning_thread_(warning_us) {
}

void Debugging::test_timer::timer_exit(int id_) {
    timer_exit_finish_tp = std::chrono::steady_clock::now();
    std::chrono::duration<int, std::micro> task_period = std::chrono::duration_cast<std::chrono::duration<int,
        std::micro> >(
        timer_exit_finish_tp - timer_enter_tp);
    if (task_period.count() > warning_thread_)
        std::cerr << "[Timer " << timer_id_ << " Place " << id_ << "]: exit in " << task_period.count() << " us \n";
}

void Debugging::test_timer::timer_record() {
    timer_enter_tp = std::chrono::steady_clock::now();
}
