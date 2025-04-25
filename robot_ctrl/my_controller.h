//
// Created by lingwei on 5/13/24.
//

#ifndef MY_MUJOCO_SIMULATOR_MY_CONTROLLER_H
#define MY_MUJOCO_SIMULATOR_MY_CONTROLLER_H

#include "robot_ctrl_base.h"
#include "FSM/Control_FSM.h"

class My_Controller : public Robot_Controller_Base {
public:
    My_Controller() = default;

    ~My_Controller() override { delete controlfsm_; };

    void Controller_Init() override;

    void run() override;

private:
    ControlFSM *controlfsm_{};
};

#endif //MY_MUJOCO_SIMULATOR_MY_CONTROLLER_H
