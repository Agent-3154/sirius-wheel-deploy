//
// Created by lingwei on 5/6/24.
//

#ifndef MY_MUJOCO_SIMULATOR_THREAD_ROBOT_RUNNER_H
#define MY_MUJOCO_SIMULATOR_THREAD_ROBOT_RUNNER_H

#include "../../utilities/inc/thread_timer.h"
#include "../../robot_runner/Robot_Runner.h"

namespace Thread {
    class thread_robot_runner : public thread_timer {
    public:
        thread_robot_runner(std::string task_name, int task_frequency);

        ~thread_robot_runner() override = default;

        [[noreturn]]  void thread_loop(RobotRunner *handle);
    };
}

#endif //MY_MUJOCO_SIMULATOR_THREAD_ROBOT_RUNNER_H
