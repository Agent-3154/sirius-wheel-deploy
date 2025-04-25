#ifndef MY_MUJOCO_SIMULATOR_FSM_STATE_LOCOMOTION_H
#define MY_MUJOCO_SIMULATOR_FSM_STATE_LOCOMOTION_H

#include "FSM_State.h"
#include "../wbc/wbc_ctrl/WBC_Locomotion_Ctrl.h"
#include "../planner/linear_planner.h"
#include "../planner/thread_planner.h"
#include "../mpc/thread_mpc.h"
#include "../mpc/linear_mpc.h"

class FSM_State_Locomotion final : public FSM_State {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FSM_State_Locomotion(Control_FSM_Data_t *controlFSMdata, Control_Parameters_t *control_para);

    ~FSM_State_Locomotion() override = default;

    bool state_on_enter() override;

    void state_on_exit() override;

    void run_state() override;

    bool is_busy() override;

    void thread_function();

private:
    WBC_Ctrl_Base<double> *wbc_ctrl_;
    LocomotionCtrlData<double> *wbc_data_;
    Linear_MPC *linear_mpc_;
    Linear_Planner *linear_planner_{};
    Vec3<double> start_pos_;
    Vec3<double> rpy_last_;

    Thread::thread_mpc *thread_mpc_;
    std::thread loco_thread;
    std::atomic_bool mpc_exit_{};
    bool mpc_thread_launched = false;
    bool first_run_ = true;
};

#endif
