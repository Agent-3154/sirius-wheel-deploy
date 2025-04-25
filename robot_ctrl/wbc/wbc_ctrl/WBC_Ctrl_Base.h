//
// Created by lingwei on 5/20/24.
//

#ifndef MY_MUJOCO_SIMULATOR_WBC_CTRL_BASE_H
#define MY_MUJOCO_SIMULATOR_WBC_CTRL_BASE_H

#include <memory>

#include "../../../utilities/types/hardware_types.h"
#include "../../robot/robot_model/Quadruped_Model_Base.h"
#include "../../FSM/Control_FSM_Data.h"
#include "../wbic/KinWBC.h"
#include "../wbic/WBIC.h"
#include "../../../robot/estimators/Estimator_Base.h"

template<typename T>
class WBC_Ctrl_Base {
public:
    explicit WBC_Ctrl_Base(Quadruped_Base *quadruped_model);

    virtual  ~WBC_Ctrl_Base();

    void run(void *input, Control_FSM_Data &ctrl_data);

    void setFloatingBaseWeight(const T &weight);

protected:
    virtual void ContactTaskUpdate(void *input, Control_FSM_Data &data) = 0;

    virtual void LCM_PublishData() {}

    void UpdateModel();

    void UpdateLegCMD(Control_FSM_Data &data);

    void ComputeWBC();

    std::shared_ptr<KinWBC<T>> kin_wbc_;
    std::shared_ptr<WBIC<T>> wbic_;
    WBIC_ExtraData<T> *wbic_extra_data_;
    Quadruped_Base *robot_model_;
    double current_time_{};

    std::vector<Contact_Task_Base<T> *> contact_list_;
    std::vector<Task_Base<T> *> task_list_;

    // prameters
    DMat<T> Mq_;
    DMat<T> Mqinv_;
    DVec<T> Cqqd_;
    DVec<T> full_config_;
    DVec<T> tau_ff_;
    DVec<T> des_jpos_;
    DVec<T> des_jvel_;
    unsigned long long iter_;
    std::vector<T> kp_joint_, kd_joint_;
};

#endif //MY_MUJOCO_SIMULATOR_WBC_CTRL_BASE_H
