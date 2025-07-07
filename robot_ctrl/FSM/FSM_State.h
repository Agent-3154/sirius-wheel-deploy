//
// Created by lingwei on 5/13/24.
//

#ifndef MY_MUJOCO_SIMULATOR_FSM_STATE_H
#define MY_MUJOCO_SIMULATOR_FSM_STATE_H

#include "Control_FSM_Data.h"
#include "../../utilities/types/std_cout_colors.h"
#include "../../config/robots_config.h"
#include "../../config/Config.h"

enum FSM_StateName {
    PASSIVE = 0,
    STAND_UP,
    SIT_DOWN,
    DAMPING,
    RL_WALK,
    RL_WALK_2,
    RL_RUNNING,
    RL_FALL_RECOVER,
    RL_WALK_STAIRS,
    USER_INTERFACE,
};

class FSM_State {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FSM_State(Control_FSM_Data_t *controlfsmdata, Control_Parameters_t *control_para, FSM_StateName fsm_name_in)
        : fsm_data_(controlfsmdata), fsm_para_(control_para),
          fsm_name_(fsm_name_in) {
    };

    virtual ~FSM_State() = default;

    virtual bool state_on_enter() = 0;

    virtual void state_on_exit() = 0;

    virtual void run_state() = 0;

    virtual bool is_busy() = 0;

    // void get_joint_state(Vec19<double> &joint_q, Vec18<double> &joint_qd, Vec3<double> &accel) const;
    void get_joint_state(Vec23<double> &joint_q, Vec22<double> &joint_qd, Vec3<double> &accel) const;

    Control_FSM_Data_t *fsm_data_;
    Control_Parameters_t *fsm_para_;
    FSM_StateName fsm_name_;
    uint32_t state_iter_ = 0;
    bool safty_check_ = false;
    bool draw_traj_request_ = false;
};

#endif //MY_MUJOCO_SIMULATOR_FSM_STATE_H
