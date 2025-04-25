//
// Created by lingwei on 6/14/24.
//

#ifndef FSM_STATE_DAMPING_H
#define FSM_STATE_DAMPING_H

#include "FSM_State.h"

class FSM_State_Damping final : public FSM_State {
public:
    FSM_State_Damping(Control_FSM_Data *_controlFSMData, Control_Parameters_t *control_para);

    ~FSM_State_Damping() override = default;

    bool state_on_enter() override;

    void state_on_exit() override;

    void run_state() override;

    bool is_busy() override;

};

#endif //FSM_STATE_DAMPING_H
