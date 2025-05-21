//
// Created by lingwei on 5/14/24.
//

#ifndef MY_MUJOCO_SIMULATOR_CONTROL_FSM_DATA_H
#define MY_MUJOCO_SIMULATOR_CONTROL_FSM_DATA_H

#include "../../robot/hardwares/usb/include/rt_remote_controller.h"
#include "../../robot/leg_controller/leg_control.h"
#include "../../robot/estimators/Estimator_Base.h"

// the data shared in controlFSMs
typedef struct Control_FSM_Data {
    Leg_Controller<double> *leg_controller_ = nullptr;
    StateEstimatorContainer<double> *estimators_ = nullptr;
    usb_controller::logic_remote_controller *rc_ = nullptr;
    std::mutex fsm_data_mutex_;
} Control_FSM_Data_t;

// maybe use eigen?
typedef struct control_parameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3<double> kp_stand_;
    Vec3<double> kd_stand_;
    double control_dt_;
    double stand_time_;
    double sit_down_time_;
    int use_wbc_;
} Control_Parameters_t;

#endif //MY_MUJOCO_SIMULATOR_CONTROL_FSM_DATA_H
