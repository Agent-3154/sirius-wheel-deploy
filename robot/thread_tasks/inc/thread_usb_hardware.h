//
// Created by lingwei on 5/10/24.
//

#ifndef MY_MUJOCO_SIMULATOR_THREAD_USB_HARDWARE_H
#define MY_MUJOCO_SIMULATOR_THREAD_USB_HARDWARE_H

#include "../../utilities/inc/thread_timer.h"
#include "../../hardwares/usb/include/rt_usb_base.h"

#include <string>
#include <memory>

namespace Thread {
    class thread_usb_hardwares : public thread_timer {
    public:
        thread_usb_hardwares(std::string task_name, int task_frequency);

        ~thread_usb_hardwares() override = default;

        [[noreturn]]  static void thread_loop(USB_HARDWARE::USB_Hardware_Containers *handle);
    };
}

#endif //MY_MUJOCO_SIMULATOR_THREAD_USB_HARDWARE_H
