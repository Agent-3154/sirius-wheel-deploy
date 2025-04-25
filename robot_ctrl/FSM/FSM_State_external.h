#ifndef FSM_EXTERNAL_H_
#define FSM_EXTERNAL_H_

#include "FSM_State.h"

class FSM_State_Extern final : public FSM_State {
public:
    FSM_State_Extern(Control_FSM_Data_t *controlFSMdata, Control_Parameters_t *control_para);

    ~FSM_State_Extern() override = default;

    bool state_on_enter() override;

    void state_on_exit() override;

    void run_state() override;

    bool is_busy() override;

private:

};


#endif
