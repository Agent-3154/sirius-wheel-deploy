//
// Created by lingwei on 7/4/24.
//
#include "../inc/thread_fdsc.h"
#include "../../utilities/types/std_cout_colors.h"
#include <iostream>
#include <utility>

Thread::thread_fdsdk_hardwares::thread_fdsdk_hardwares(std::string task_name, int task_frequency): thread_timer(
    std::move(task_name), task_frequency) {
}

void Thread::thread_fdsdk_hardwares::thread_loop(My_FDSC *handle) {
    handle->FDSC_init();
    std::cout << GREEN << "[Thread FDSDK OK]: " << RESET << "Initialize free dog SDK thread!\n";
    while (true) {
        this->thread_enter_task();
        handle->parse_data();
        handle->pack_cmd();
        this->thread_finish_task();
    }
}
