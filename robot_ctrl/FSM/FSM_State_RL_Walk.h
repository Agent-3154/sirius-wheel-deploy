#ifndef FSM_STATE_RL_WALK_H_
#define FSM_STATE_RL_WALK_H_

#include "FSM_State.h"
#include "../rl_controller/rl_controller.h"
class FSM_State_RL_Walk final : public FSM_State {
public:
    FSM_State_RL_Walk(Control_FSM_Data *_controlFSMData, Control_Parameters_t *control_para);

    ~FSM_State_RL_Walk() override = default;

    bool state_on_enter() override;

    void state_on_exit() override;

    void run_state() override;

    bool is_busy() override;

private:
    // Keep track of the control iterations
    std::vector<Vec3<double> > joint_pos_ini_;
    std::vector<Vec3<double> > joint_pos_end_;
    std::shared_ptr<RLController> rl_controller_;

    // Used for filter
    Vec3<double> desired_vel_xyw_last;
    
};

#endif  // FSM_STATE_RL_WALK_H_
