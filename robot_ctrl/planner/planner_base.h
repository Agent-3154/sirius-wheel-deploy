//
// Created by lingwei on 6/8/24.
//

#ifndef PLANNER_BASE_H
#define PLANNER_BASE_H

#include "../../utilities/inc/utilities_fun.h"
#include "../gait_scheduler/offset_duration_gait.h"
#include "../FSM/Control_FSM_Data.h"
#include "../foot_trajectory/foot_trajec_bezier.h"
#include "../../config/robots_config.h"
#include "../../lcm-types/cpp/planner_lcmt.hpp"
#include <condition_variable>
#if defined (WORK_COMPUTOR)
#include "iceoryx/v2.95.4/iceoryx_posh/popo/publisher.hpp"
#include "iceoryx/v2.95.4/iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx/v2.95.4/iceoryx_posh/runtime/posh_runtime.hpp"
#include "iceoryx/v2.95.4/iox/signal_watcher.hpp"
#include "sim_memory_share_data.h"
#elif defined(LapTop)
#include "iceoryx/v/iceoryx_posh/popo/publisher.hpp"
#include "iceoryx/v/iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx/v/iceoryx_posh/runtime/posh_runtime.hpp"
#include "iceoryx/v/iox/signal_watcher.hpp"
#endif
#include "../../simulator/sim_memory_share_data.h"

template<typename T>
struct planner_desire {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    gait_number current_gait_;
    int *contact_table_;
    Vec3<T> rpy_int_;
    Vec3<T> rpy_comp_;
    Vec3<T> pBody_des_;
    Vec3<T> vBody_des_;
    Vec3<T> aBody_des_;
    Vec3<T> pBody_RPY_des_;
    Vec3<T> vBody_Ori_des_;
    Vec3<T> vBody_Ori_des_w_;
    // note: these only work in swing phase
    Vec3<T> pFoot_des_[4];
    Vec3<T> vFoot_des_[4];
    Vec3<T> aFoot_des_[4];
    Vec3<T> pFoot_[4];
    Vec3<T> Fr_des_[4];

    void setDesireZero();
};

template<typename T>
void planner_desire<T>::setDesireZero() {
    pBody_RPY_des_.setZero();
    vBody_des_.setZero();
    aBody_des_.setZero();
    pBody_RPY_des_.setZero();
    vBody_Ori_des_.setZero();
    for (int i = 0; i < 4; i++) {
        pFoot_des_[i].setZero();
        vFoot_des_[i].setZero();
        aFoot_des_[i].setZero();
        Fr_des_[i].setZero();
    }
}

template<typename T>
class Planner_Base {
public:
    Planner_Base(double dt, int swing_segment);

    virtual ~Planner_Base() = default;

    virtual void run(Control_FSM_Data &data) =0;

    virtual void SetupCommand(const Control_FSM_Data &data) = 0;

    virtual void set_lcm() = 0;

    virtual void publish_trajectory_memory() = 0;

    bool use_wbc_{};
    planner_desire<T> desired_;
    // std::atomic_bool first_schedule_{};
    Vec4<double> contact_state_;
    // std::condition_variable_any planner_cond_;
    // std::mutex wait_mtx_;
    bool firstRun_ = true;

protected:
    T dt_{};
    T dtFoot_{};
    T default_dtFoot_{};
    int swing_segment_;

    T body_height_;
    double stand_traj_[6]{};
    bool firstSwing_[4]{};

    Vec3<double> world_position_desired_; // world postion: x,y, yaw
    Vec3<double> world_pos_;
    Vec3<T> pf_[4];
    Vec3<T> pf_ini_[4];
    Vec3<double> pw_ini_;
    Vec3<double> rpy_ini_;

    Vec3<double> rpy_des_b_, rpy_vel_des_b_;
    Vec3<double> rpy_des_w_, rpy_vel_des_w_;
    Vec3<double> rpy_des_w_last_;
    double x_comp_integral_ = 0;
    double x_vel_des_{}, y_vel_des_{}, step_height_{};

    // gait
    gait_number currentGait_{};
    gait_number gaitNumber_{};
    Vec4<double> swingStates;
    Vec4<T> swingTime_;
    Foot_Trajec_Bezier<double> footSwingTrajectories_[4];
    OffsetDurationGait<double> trotting_, stand_, walk_, running_;
    Gait_Base<T> *work_gait_{};
    int *contact_table_{};
    double swingTimeRemained_[4]{};

    //X_drag? Configure the dimension of the vector
    double trajectoryAll_[12 * 36]{};
    long iterCounter_{};
    lcm::LCM planner_lcm_;
    planner_lcmt lcm_data_{};
    iox::popo::Publisher<Sim_Plot> plot_publisher;
};

template<typename T>
Planner_Base<T>::Planner_Base(double dt, int swing_segment) : dt_(dt), swing_segment_(swing_segment), trotting_(
                                                                  Config::trot_horizonLength,
                                                                  Config::trot_offset,
                                                                  Config::trot_duration, "Trotting"),
                                                              stand_(Config::horizonLength, Config::stand_offset,
                                                                     Config::stand_duration, "Standing"),
                                                              walk_(Config::walk_horizonLength, Config::walk_offset,
                                                                    Config::walk_duration, "Walk"),
                                                              running_(Config::trot_running_horizonLength,
                                                                       Config::trot_running_offset,
                                                                       Config::trot_running_duration, "Running"),
                                                              planner_lcm_(getLcmUrl(255)),
                                                              plot_publisher({"Robot", "Plot", "State"}) {
    dtFoot_ = dt * swing_segment;
    default_dtFoot_ = swing_segment;
    // TODO add the solver setup_problem
    for (bool &i: firstSwing_) {
        i = true;
    }
    swingTime_.setZero();
    desired_.setDesireZero();
    world_position_desired_.setZero();
    world_pos_.setZero();
    gaitNumber_ = STAND;
    currentGait_ = STAND;
    body_height_ = Config::mpc_height;
    work_gait_ = &stand_;

    for (int i = 0; i < 4; i++) {
        pf_[i].setZero();
        pf_ini_[i].setZero();
    }
}
#endif //PLANNER_BASE_H
