//
// Created by lingwei on 5/6/24.
//
#include "thread_robot_runner.h"

#include <utility>


Thread::thread_robot_runner::thread_robot_runner(std::string task_name, int task_frequency) : thread_timer(
        std::move(task_name),
        task_frequency) {


}

void Thread::thread_robot_runner::thread_loop(RobotRunner *handle) {
    while (true){
        this->thread_enter_task();
        handle->run();
        this->thread_finish_task();
    }
}
