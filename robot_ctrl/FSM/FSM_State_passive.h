#ifndef FSM_STATE_PASSIVE_H
#define FSM_STATE_PASSIVE_H

#include "FSM_State.h"

class FSM_State_Passive : public FSM_State {
public:
    explicit FSM_State_Passive(Control_FSM_Data_t *controlFSMData, Control_Parameters_t* control_para);

    ~FSM_State_Passive() override = default;

    bool state_on_enter() override;

    void state_on_exit() override;

    void run_state() override;

    bool is_busy() override;
};

#endif  // FSM_STATE_PASSIVE_H
