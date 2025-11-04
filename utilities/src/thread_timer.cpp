//
// Created by lingwei on 4/5/24.
//
#include "../inc/thread_timer.h"
#include "../types/std_cout_colors.h"
#include <chrono>
#include <iostream>
#include <cmath>
#include <utility>
#include <unistd.h>

#include "../inc/easylogging++.h"

using namespace std::chrono;

namespace Thread {
    thread_timer::thread_timer(std::string task_name, int task_frequency, bool print_info) : task_name_(std::move(task_name)), print_info_(print_info) {
        if (task_frequency != 0) {
            timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
            thread_total_t = static_cast<int>((float) 1 / (float) task_frequency * 1000000);
            int seconds = static_cast<int> ((float) 1 / (float) task_frequency);
            int nanoseconds = (int) (1e9 * std::fmod((float) 1 / (float) task_frequency, 1.f));
            std::cout << BLUE << task_name_ << " Schedule time " << RESET << seconds << "s " << nanoseconds << " nanoseconds \n";
            timerSpec.it_interval.tv_sec = seconds;
            timerSpec.it_value.tv_sec = seconds;
            timerSpec.it_value.tv_nsec = nanoseconds;
            timerSpec.it_interval.tv_nsec = nanoseconds;
            timerfd_settime(timerfd, 0, &timerSpec, nullptr);
        }
    }

    void thread_timer::thread_enter_task() {
        thread_enter_tp = high_resolution_clock::now();
    }

    void thread_timer::thread_finish_task() {
        thread_task_finish_tp = high_resolution_clock::now();
        duration<int, std::micro> task_period = duration_cast<duration<int, std::micro>>(
                thread_task_finish_tp - thread_enter_tp);
        thread_sleep_du = thread_total_t - task_period.count();
        if ((thread_sleep_du < 0) && print_info_) {
            auto warning_msg = "[Threading Warning]: " + task_name_ + " Consuming time " + std::to_string(task_period.count()) + "/" + std::to_string(thread_total_t) + " us\n";
            std::cout << BOLDRED << warning_msg << RESET;
        }
        unsigned long long missed = 0;
        const int m = read(timerfd, &missed, sizeof(missed));
        (void) m;
    }
}
