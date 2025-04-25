//
// Created by lingwei on 7/4/24.
//

#ifndef THREAD_FDSC_H
#define THREAD_FDSC_H

#include "../../utilities/inc/thread_timer.h"
#include "../../hardwares/fdsc_utils/inc/free_dog_sdk_h.hpp"
#include "../../hardwares/fdsc_utils/my_fdsc.h"
#include <string>

namespace Thread {
    class thread_fdsdk_hardwares : public thread_timer {
    public:
        thread_fdsdk_hardwares(std::string task_name, int task_frequency);

        ~thread_fdsdk_hardwares() override = default;

        [[noreturn]] void thread_loop(My_FDSC *handle);
    };
}

#endif //THREAD_FDSC_H
