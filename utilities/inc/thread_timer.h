//
// Created by lingwei on 4/5/24.
//

#ifndef MY_MUJOCO_SIMULATOR_THREAD_TIMER_H
#define MY_MUJOCO_SIMULATOR_THREAD_TIMER_H
#include <chrono>
#include <string>
#include <sys/timerfd.h>

#include "../../config/Config.h"

namespace Thread{
    class thread_timer{
    public:
        thread_timer(std::string  task_name, int task_frequency, bool print_info = Config::thread_time_spy);
        virtual ~thread_timer() = default;
        virtual void thread_enter_task();
        virtual void thread_finish_task();
    private:
        // unit: microseconds
        std::chrono::high_resolution_clock::time_point thread_enter_tp;
        std::chrono::high_resolution_clock::time_point thread_task_finish_tp;
        int thread_sleep_du{};
        int thread_total_t;
        std::string task_name_;
        itimerspec timerSpec{};
        int timerfd;
        bool print_info_;
    };
}

#endif //MY_MUJOCO_SIMULATOR_THREAD_TIMER_H
