//
// Created by lingwei on 4/5/24.
//
#include "../inc/thread_rc.h"
#include "../../utilities/types/std_cout_colors.h"
#include <iostream>

#include "../../../utilities/inc/easylogging++.h"

namespace Thread {

    thread_rc::thread_rc(const std::string &task_name, int task_frequency) : thread_timer(task_name,
                                                                                                     task_frequency) {

    }

    [[noreturn]] void thread_rc::thread_loop(usb_controller::logic_remote_controller *handle, bool print_rc_data) {
        (void ) print_rc_data;
        std::cout << GREEN << "[Thread RC OK]: " << RESET << "Initialize RC thread!\n";
        while (true) {
            this->thread_enter_task();
            ssize_t read_bit = handle->rc_complete();
            this->thread_finish_task();
        }

    }
}
