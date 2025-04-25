//
// Created by lingwei on 5/6/24.
//

#ifndef MY_MUJOCO_SIMULATOR_DEBUG_TOOLS_H
#define MY_MUJOCO_SIMULATOR_DEBUG_TOOLS_H


#include <chrono>
#include <string>
#include <sys/timerfd.h>

namespace Debugging {
    class test_timer {
    public:
        explicit test_timer(int timer_id, double warning_us);

        ~test_timer() = default;

        void timer_exit(int id_);

        void timer_record();

    private:
        // unit: microseconds
        std::chrono::steady_clock::time_point timer_enter_tp;
        std::chrono::steady_clock::time_point timer_exit_finish_tp;
        int thread_sleep_du{};
        int thread_total_t{};
        int timer_id_;
        int places_id_ = 0;
        double warning_thread_;
    };
}

#endif //MY_MUJOCO_SIMULATOR_DEBUG_TOOLS_H
