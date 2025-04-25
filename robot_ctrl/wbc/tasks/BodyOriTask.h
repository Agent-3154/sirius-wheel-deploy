//
// Created by lingwei on 5/20/24.
//

#ifndef MY_MUJOCO_SIMULATOR_BODYORITASK_H
#define MY_MUJOCO_SIMULATOR_BODYORITASK_H

#include "TaskBase.h"
#include "../../robot/robot_model/Quadruped_Model_Base.h"

template<typename T>
class BodyOriTask : public Task_Base<T> {
public:
    BodyOriTask(const Quadruped_Base *base_model);

    virtual ~BodyOriTask() = default;

    DVec<T> kp_, kd_;

protected:
    bool UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des) override;

    bool UpdateTaskJacobian() override;

    bool UpdateTaskJdqd() override;

private:
    DVec<T> kp_kin_; // what is this?

    size_t ori_link_id_;
    const Quadruped_Base *robot_;
};


#endif //MY_MUJOCO_SIMULATOR_BODYORITASK_H
