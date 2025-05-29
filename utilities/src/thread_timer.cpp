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
        thread_enter_tp = std::chrono::high_resolution_clock::now();
    }

    void thread_timer::thread_finish_task() {
        thread_task_finish_tp = std::chrono::high_resolution_clock::now();
        std::chrono::duration<int, std::micro> task_period = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(
                thread_task_finish_tp - thread_enter_tp);
        thread_sleep_du = thread_total_t - task_period.count();
        if (thread_sleep_du < 0 & print_info_) {
            std::cout << "[Task Run Error]: " << task_name_
                         << " Consuming time " << thread_sleep_du << " us is longer than given schedule period!\n";
            // std::cout << BOLDRED << "[Task Run Error]: " << RESET << task_name_
            //           << " Consuming time "<< thread_sleep_du <<" us is longer than given schedule period!\n";
//        } else {
////            std::cout << "Thread Sleep!"  << thread_sleep_du << "\n";
//            std::this_thread::sleep_until(thread_enter_tp + std::chrono::duration<int, std::micro>(thread_total_t));
//        }
        }
        unsigned long long missed = 0;
        const int m = read(timerfd, &missed, sizeof(missed));
        (void) m;
    }
}
