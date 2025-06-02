#ifndef FSM_STATE_EXTERNAL_CONTROL
#define FSM_STATE_EXTERNAL_CONTROL

#include "FSM_State.h"

class FSM_State_External_Control final : public FSM_State {
public:
    FSM_State_External_Control(Control_FSM_Data *_controlFSMData, Control_Parameters_t *control_para);

    ~FSM_State_External_Control() override = default;

    bool state_on_enter() override;

    void state_on_exit() override;

    void run_state() override;

    bool is_busy() override;

private:
    // Keep track of the control iterations
    std::vector<Vec3<double> > joint_pos_ini_;
    std::vector<Vec3<double> > joint_pos_end_;
};

#endif  // FSM_STATE_EXTERNAL_CONTROL
