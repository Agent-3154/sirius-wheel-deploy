#ifndef MY_MUJOCO_SIMULATOR_FSM_STATE_BS_H
#define MY_MUJOCO_SIMULATOR_FSM_STATE_BS_H

#include "FSM_State.h"
#include "../wbc/wbc_ctrl/WBC_Locomotion_Ctrl.h"

class FSM_State_BS final : public FSM_State {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FSM_State_BS(Control_FSM_Data_t *controlFSMdata, Control_Parameters_t *control_para);

    ~FSM_State_BS() override = default;

    bool state_on_enter() override;

    void state_on_exit() override;

    void run_state() override;

    bool is_busy() override;

private:
    WBC_Ctrl_Base<double> *wbc_ctrl_;
    LocomotionCtrlData<double> *wbc_data_;
    double total_weight_{};
    Vec3<double> start_pos_;
    Vec3<double> start_rpy_;
    Vec3<double> rpy_last_;
};

#endif
