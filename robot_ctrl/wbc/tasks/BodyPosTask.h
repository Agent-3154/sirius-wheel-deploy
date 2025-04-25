//
// Created by lingwei on 5/20/24.
//

#ifndef MY_MUJOCO_SIMULATOR_BODYPOSTASK_H
#define MY_MUJOCO_SIMULATOR_BODYPOSTASK_H

#include "TaskBase.h"
#include "../../robot/robot_model/Quadruped_Model_Base.h"

template<typename T>
class BodyPosTask : public Task_Base<T> {
public:
    BodyPosTask(const Quadruped_Base *base_model);

    virtual ~BodyPosTask() = default;

    DVec<T> kp_, kd_;

protected:
    virtual bool UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des);

    virtual bool UpdateTaskJacobian();

    virtual bool UpdateTaskJdqd();

private:
    DVec<T> kp_kin_; // what is this?

    const Quadruped_Base *robot_;
};

#endif //MY_MUJOCO_SIMULATOR_BODYPOSTASK_H
