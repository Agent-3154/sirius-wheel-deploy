//
// Created by lingwei on 4/5/24.
//

#ifndef MY_MUJOCO_SIMULATOR_THREAD_RC_H
#define MY_MUJOCO_SIMULATOR_THREAD_RC_H

#include "../../utilities/inc/thread_timer.h"
#include "../../hardwares/usb/include/rt_remote_controller.h"

namespace Thread {
    class thread_rc : public thread_timer {
    public:
        thread_rc(const std::string &task_name, int task_frequency);

        ~thread_rc() = default;

        [[noreturn]] void thread_loop(usb_controller::logic_remote_controller *handle,  bool print_rc_data = false);
    };
}
#endif //MY_MUJOCO_SIMULATOR_THREAD_RC_H
