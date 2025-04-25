//
// Created by lingwei on 6/8/24.
//

#ifndef THREAD_PLANNER_H
#define THREAD_PLANNER_H

#include <iostream>
#include "planner_base.h"
#include "../../utilities/inc/thread_timer.h"

namespace Thread {
    class thread_planner : public thread_timer {
    public:
        thread_planner(const std::string &task_name, int task_frequency);

        ~thread_planner() override = default;

        void thread_loop(Planner_Base<double> *handle, Control_FSM_Data &data);
    };
}

#endif //THREAD_PLANNER_H
