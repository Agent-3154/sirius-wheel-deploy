//
// Created by lingwei on 6/8/24.
//

#ifndef THREAD_MPC_H
#define THREAD_MPC_H
#include <iostream>
#include "../../utilities/inc/thread_timer.h"
#include "../planner/planner_base.h"
#include "mpc_base.h"

namespace Thread {
    class thread_mpc : public thread_timer {
    public:
        thread_mpc(const std::string &task_name, int task_frequency);

        ~thread_mpc() override = default;

        void thread_loop(MPC_Base *handle, Control_FSM_Data &data, const planner_desire<double> &p_data,
                                      std::atomic_bool& update_);
    };
}

#endif //THREAD_MPC_H
