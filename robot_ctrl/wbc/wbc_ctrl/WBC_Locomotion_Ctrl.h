//
// Created by lingwei on 5/20/24.
//

#ifndef MY_MUJOCO_SIMULATOR_WBC_LOCOMOTION_CTRL_H
#define MY_MUJOCO_SIMULATOR_WBC_LOCOMOTION_CTRL_H

#include "WBC_Ctrl_Base.h"
#include "../../../lcm-types/cpp/wbc_test_data_lcmt.hpp"

template<typename T>
struct LocomotionCtrlData {
    Vec3<T> pBody_des;
    Vec3<T> vBody_des;
    Vec3<T> aBody_des;
    Vec3<T> pBody_RPY_des;
    Vec3<T> vBody_Ori_des;

    Vec3<T> pFoot_des[4];
    Vec3<T> vFoot_des[4];
    Vec3<T> aFoot_des[4];
    Vec3<T> Fr_des[4];

    Vec4<T> contact_state;
};

template<typename T>
class LocomotionCtrl : public WBC_Ctrl_Base<T> {
public:
    explicit LocomotionCtrl(Quadruped_Base *model);

    virtual ~LocomotionCtrl() {
        delete body_pos_task_;
        delete body_ori_task_;
        for (int i = 0; i < 4; i++) {
            delete foot_swing_task_[i];
            delete foot_contact_[i];
        }
    };

    void LCM_PublishData() override;

protected:
    void ContactTaskUpdate(void *input, Control_FSM_Data &data) override;

    void SetupParameter();

    void Clean();

    LocomotionCtrlData<T> *input_data_;
    Task_Base<T> *body_pos_task_;
    Task_Base<T> *body_ori_task_;
    Task_Base<T> *foot_swing_task_[4];
    Contact_Task_Base<T> *foot_contact_[4];
    Vec3<T> Fr_result_[4];
    Quat<T> quat_des_;
    lcm::LCM wbc_lcm_;
    wbc_test_data_lcmt wbc_data{};
    int print_dim{};
};

#endif //MY_MUJOCO_SIMULATOR_WBC_LOCOMOTION_CTRL_H
