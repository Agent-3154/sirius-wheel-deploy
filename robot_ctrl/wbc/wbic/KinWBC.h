//
// Created by lingwei on 5/20/24.
//

#ifndef MY_MUJOCO_SIMULATOR_KINWBC_H
#define MY_MUJOCO_SIMULATOR_KINWBC_H

#include "../tasks/TaskBase.h"
#include <vector>


template<typename T>
class KinWBC {
public:
    explicit KinWBC(int num_vel);

    ~KinWBC() = default;

    bool FindConfiguration(const DVec<T> &curr_config, const std::vector<Task_Base<T> *> &task_list,
                           const std::vector<Contact_Task_Base<T> *> &constact_list,
                           DVec<T> &jpos_cmd, DVec<T> &jvel_cmd);

    DMat<T> Mqinv_;

private:
    void BuildProjectMatrix(const DMat<T> &J, DMat<T> &N);

    double threshould_;
    int num_total_joints_;
    int num_act_joints_;
    DMat<T> identity_matrix_;
};

#endif //MY_MUJOCO_SIMULATOR_KINWBC_H
