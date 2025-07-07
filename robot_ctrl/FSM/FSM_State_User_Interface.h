#ifndef FSM_State_User_Interface_H_
#define FSM_State_User_Interface_H_

#include <iceoryx_posh/popo/subscriber.hpp>

#include "FSM_State.h"
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "../../quadruped_share_data/robot_state_protocols.h"
#include "../../utilities/inc/thread_timer.h"
#include <atomic>
#include <shared_mutex>

class FSM_State_User_Interface final : public FSM_State {
public:
    FSM_State_User_Interface(Control_FSM_Data_t *controlFSMdata, Control_Parameters_t *control_para);

    ~FSM_State_User_Interface() override = default;

    bool state_on_enter() override;

    void state_on_exit() override;

    void run_state() override;

    bool is_busy() override;

private:
    iox::popo::Publisher<Robot_State> iceoryx_state_publisher_;
    iox::popo::Subscriber<Robot_Control_Motor_Cmd> iceoryx_motor_subscriber_;
    std::thread subscriber_thread_;

    void subscriber_thread_func();
    std::atomic<bool> exit_state_{};
    std::shared_ptr<Thread::thread_timer> user_interface_timer_;
    double q_des[18]{}, qd_des[18]{}, kp_joint[18]{}, kd_joint[18]{}, tau_ff[18]{};
    // Vec19<double> state_q_;
    // Vec18<double> state_qd_;
    Vec23<double> state_q_;
    Vec22<double> state_qd_;
    Vec3<double> state_accel;
    std::shared_mutex state_mutex_;
};


#endif
