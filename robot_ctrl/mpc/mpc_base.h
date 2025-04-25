//
// Created by lingwei on 5/30/24.
//
#ifndef MY_MUJOCO_SIMULATOR_MPC_BASE_H
#define MY_MUJOCO_SIMULATOR_MPC_BASE_H

#include "../gait_scheduler/offset_duration_gait.h"
#include "../FSM/Control_FSM_Data.h"
#include "../planner/planner_base.h"

template<typename T>
struct mpc_desire {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3<T> pBody_des_;
    Vec3<T> vBody_des_;
    Vec3<T> pBody_RPY_des_;
    Vec3<T> vBody_Ori_des_;
    Vec3<double> pFoot_[4]; // note: these only work in swing phase
    Vec3<T> Fr_des_[4];

    void setDesireZero();
};

template<typename T>
void mpc_desire<T>::setDesireZero() {
    pBody_RPY_des_.setZero();
    vBody_des_.setZero();
    pBody_RPY_des_.setZero();
    vBody_Ori_des_.setZero();
    for (auto &Fr_de: Fr_des_) {
        Fr_de.setZero();
    }
}

class MPC_Base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MPC_Base() = default;

    virtual ~MPC_Base() = default;

    virtual void run(Control_FSM_Data &data,  std::atomic_bool& update) = 0;

    virtual void SetupCommand(const planner_desire<double> &data) = 0;

    virtual void mpc_lcm_publish() = 0;

    mpc_desire<double> desire_data_;
    // std::mutex mpc_mtx_;
    std::atomic_bool t_exit{}, busy_{};

protected:
    double dtMPC_{};
};

#endif //MY_MUJOCO_SIMULATOR_MPC_BASE_H
