//
// Created by lingwei on 5/20/24.
//

#ifndef MY_MUJOCO_SIMULATOR_CONTACTTASK_H
#define MY_MUJOCO_SIMULATOR_CONTACTTASK_H

#include "TaskBase.h"

#include "../../robot/robot_model/Quadruped_Model_Base.h"

template<typename T>
class ContactTask : public Contact_Task_Base<T> {
public:
    ContactTask(const Quadruped_Base *base_model, int contact_id);

    virtual ~ContactTask() = default;

protected:

    bool UpdateTaskJacobian() override;

    bool UpdateTaskJdqd() override;

    bool UpdateUf() override;

    bool UpdateInequalityVector() override;

private:
    int dim_u_;
    T max_Fz_;
    const Quadruped_Base *robot_;
    int contact_id_;
    T mu_;
};
#endif //MY_MUJOCO_SIMULATOR_CONTACTTASK_H
