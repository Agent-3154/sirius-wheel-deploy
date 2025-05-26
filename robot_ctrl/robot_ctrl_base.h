//
// Created by lingwei on 5/7/24.
//

#ifndef MY_MUJOCO_SIMULATOR_ROBOT_CTRL_BASE_H
#define MY_MUJOCO_SIMULATOR_ROBOT_CTRL_BASE_H
#include <atomic>
//#include "../robot/robot_runner/Robot_Runner.h"
#include "../robot/hardwares/usb/include/rt_remote_controller.h"
#include "../robot/leg_controller/leg_control.h"
#include "../robot/estimators/Estimator_Base.h"

class Robot_Controller_Base {
    friend class RobotRunner; // robotrunner can access robot_ctrl private variables
public:
    Robot_Controller_Base() = default;

    virtual ~Robot_Controller_Base()= default;

    virtual void Controller_Init() = 0;

    virtual void run() = 0;
    std::atomic_bool control_draw_cond_{};
protected:
    Leg_Controller<double> *leg_controller_ = nullptr;
    StateEstimateOutput<double> *state_esti_ouput_ = nullptr;
    StateEstimatorContainer<double> *estimators_ = nullptr;
    usb_controller::logic_remote_controller *ctrl_rc_ = nullptr;

};

#endif //MY_MUJOCO_SIMULATOR_ROBOT_CTRL_BASE_H
