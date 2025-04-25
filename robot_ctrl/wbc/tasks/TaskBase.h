//
// Created by lingwei on 5/20/24.
//

#ifndef MY_MUJOCO_SIMULATOR_TASKBASE_H
#define MY_MUJOCO_SIMULATOR_TASKBASE_H

#include "../../utilities/types/hardware_types.h"
#include "../../utilities/inc/utilities_fun.h"

template<typename T>
class Task_Base {
public:
    explicit Task_Base(int task_dim) : dim_task_(task_dim), op_cmd_(dim_task_), Jtdqd_(dim_task_), pos_err_(task_dim),
                                       vel_des_(task_dim), acc_des_(task_dim) {
        op_cmd_.setZero();
        pos_err_.setZero();
    }

    virtual ~Task_Base() = default;

    bool UpdateTasks(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des) {
        UpdateTaskJacobian();
        UpdateTaskJdqd();
        UpdateCommand(pos_des, vel_des, acc_des);
        return true;
    }

    void getCommand(DVec<T> &op_cmd) { op_cmd = op_cmd_; }

    void getTaskJacobian(DMat<T> &Jt) { Jt = Jt_; }

    void getTaskJacobianDotQdot(DVec<T> &JtDotQdot) { JtDotQdot = Jtdqd_; }

    const DVec<T> &getPosError() { return pos_err_; }

    const DVec<T> &getDesVel() { return vel_des_; }

    const DVec<T> &getDesAcc() { return acc_des_; }

protected:
    // update cmd in each task.
    virtual bool UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des) = 0;

    virtual bool UpdateTaskJacobian() = 0;

    virtual bool UpdateTaskJdqd() = 0;

    // for pos task
    int dim_task_;
    DVec<T> op_cmd_;
    DVec<T> Jtdqd_;
    D3Mat18<T> Jt_;
    DVec<T> pos_err_;
    DVec<T> vel_des_;
    DVec<T> acc_des_;

    // for contact task
    DVec<T> ieq_vec_;
    DVec<T> Fr_des_;
    int dim_contact_{};
    DMat<T> uf_;
};


template<typename T>
class Contact_Task_Base {
public:
    explicit Contact_Task_Base(int task_dim) : dim_task_(task_dim), Jtdqd_(dim_task_) {
    }

    virtual ~Contact_Task_Base() = default;

    [[nodiscard]] int get_dim_contact() const { return dim_contact_; }

    [[nodiscard]] int get_dim_RFConstrain() const { return uf_.rows(); };

    void get_contact_jacobian(DMat<T> &Jc) const { Jc = Jt_; };

    void get_contact_jdqd(DVec<T> &Jcdqd) const { Jcdqd = Jtdqd_; }

    void getRFConstraintMtx(DMat<T> &Uf) { Uf = uf_; }

    void getRFConstraintVec(DVec<T> &ieq_vec) { ieq_vec = ieq_vec_; }

    const DVec<T> &getRFDesired() { return Fr_des_; }

    void setRFDesired(const DVec<T> &Fr_des) { Fr_des_ = Fr_des; }

    bool UpdateTasks() {
        UpdateTaskJacobian();
        UpdateTaskJdqd();
        UpdateUf();
        UpdateInequalityVector();
        return true;
    }

protected:
    virtual bool UpdateTaskJacobian() = 0;

    virtual bool UpdateTaskJdqd() = 0;

    virtual bool UpdateUf() = 0;

    virtual bool UpdateInequalityVector() = 0;

    // for pos task
    int dim_task_;
    DVec<T> Jtdqd_;
    D3Mat18<T> Jt_;
    // for contact task
    DVec<T> ieq_vec_;
    DVec<T> Fr_des_;
    int dim_contact_{};
    DMat<T> uf_;
};

#endif //MY_MUJOCO_SIMULATOR_TASKBASE_H
