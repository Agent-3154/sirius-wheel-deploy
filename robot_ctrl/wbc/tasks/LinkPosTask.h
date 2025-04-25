//
// Created by lingwei on 5/20/24.
//

#ifndef MY_MUJOCO_SIMULATOR_LINKPOSTASK_H
#define MY_MUJOCO_SIMULATOR_LINKPOSTASK_H

#include "TaskBase.h"
#include "../../robot/robot_model/Quadruped_Model_Base.h"

template<typename T>
class LinkPoseTask : public Task_Base<T> {
public:
    LinkPoseTask(const Quadruped_Base *base_model, int link_id); // the contact leg id

    virtual ~LinkPoseTask() = default;

    DVec<T> kp_, kd_;
protected:
    virtual bool UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des);

    virtual bool UpdateTaskJacobian();

    virtual bool UpdateTaskJdqd();

private:
    DVec<T> kp_kin_; // what is this?

    int link_id_; // note: this parameter should be in 0~4
    const Quadruped_Base *robot_;
};

#endif //MY_MUJOCO_SIMULATOR_LINKPOSTASK_H
