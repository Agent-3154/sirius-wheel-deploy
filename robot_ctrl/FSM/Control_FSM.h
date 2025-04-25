//
// Created by lingwei on 5/13/24.
//

#ifndef MY_MUJOCO_SIMULATOR_CONTROL_FSM_H
#define MY_MUJOCO_SIMULATOR_CONTROL_FSM_H

#include "FSM_State_sit_down.h"
#include "FSM_State_stand_up.h"
#include "FSM_State_passive.h"
#include "FSM_State_locomotion.h"
#include "Control_FSM_Data.h"
#include "FSM_State_bs.h"
#include "FSM_State_damping.h"
#include "FSM_State_external.h"

struct FSM_StateList {
    FSM_State_Stand_Up *s_standup;
    FSM_State_SitDown *s_sitdown;
    FSM_State_Passive *s_passive;
    FSM_State_BS *s_bs;
    FSM_State_Locomotion *s_locomotion;
    FSM_State_Damping* s_damping;
    FSM_State_Extern* s_extern;
};

class ControlFSM {
public:
    ControlFSM(usb_controller::logic_remote_controller *rc, Quadruped_Base *qb,
               Leg_Controller<double> *leg_control, StateEstimatorContainer<double> *stateEsti);

    void ControlFSM_run();

    Control_FSM_Data_t control_data_;
    Control_Parameters_t control_para_;
    FSM_State *state_current_;
    FSM_State *state_next_;
    FSM_StateList state_list_{};

private:
    void Get_Settings();
    int danger_times_ = 0;
};

#endif //MY_MUJOCO_SIMULATOR_CONTROL_FSM_H
