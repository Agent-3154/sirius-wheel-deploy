//
// Created by lingwei on 5/20/24.
//

#ifndef MY_MUJOCO_SIMULATOR_WBC_BASE_H
#define MY_MUJOCO_SIMULATOR_WBC_BASE_H

#include "../../utilities/inc/utilities_fun.h"
#include "../../utilities/types/hardware_types.h"
#include "../tasks/TaskBase.h"

template<typename T>
class WBC_Base {
public:
    explicit WBC_Base(int dim_vel) : num_act_joint_(dim_vel - 6), num_joint_total_(dim_vel) {
        Sv_ = DMat<T>::Zero(6, num_joint_total_);
        Sv_.block(0, 0, 6, 6).setIdentity();
    }

    virtual  ~WBC_Base() = default;

    virtual void UpdateSettings(const DMat<T> &Mq, const DMat<T> &Mqinv,
                                const DVec<T> &cqqd, void *extra_setting) = 0;

    virtual void MakeTorque(DVec<T> &cmd, void *extra_input) = 0;

protected:
    void WeightedInverse(const DMat<T> &J, const DMat<T> &Winv, DMat<T> &Jinv,
                         double threshold = 0.0001) {
        DMat<T> lambda(J * Winv * J.transpose());
        DMat<T> lambda_inv;
        pseudoInverse(lambda, threshold, lambda_inv);
        Jinv = Winv * J.transpose() * lambda_inv;
    }

    int num_act_joint_;
    int num_joint_total_; //including floating base

    DMat<T> Mq_;
    DMat<T> Mqinv_;
    DVec<T> Cqqd_; // including gravity
    DMat<T> Sv_; // virtual selection matrix

};

#endif //MY_MUJOCO_SIMULATOR_WBC_BASE_H
