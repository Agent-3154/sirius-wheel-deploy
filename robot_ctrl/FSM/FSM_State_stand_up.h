#ifndef FSM_STAND_UP_H_
#define FSM_STAND_UP_H_

#include "FSM_State.h"

class FSM_State_Stand_Up final : public FSM_State {
public:
    FSM_State_Stand_Up(Control_FSM_Data_t *controlFSMdata, Control_Parameters_t *control_para);

    ~FSM_State_Stand_Up() override = default;

    bool state_on_enter() override;

    void state_on_exit() override;

    void run_state() override;

    bool is_busy() override;

    bool fold_flag_ = false;

private:
    std::vector<Vec3<double> > joint_pos_ini_;
    std::vector<Vec3<double> > joint_pos_stand_;
    std::vector<Vec3<double> > joint_pos_fold_;
};


#endif
