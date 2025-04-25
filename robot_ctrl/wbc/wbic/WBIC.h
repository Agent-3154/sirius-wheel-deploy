//
// Created by lingwei on 5/20/24.
//

#ifndef MY_MUJOCO_SIMULATOR_WBIC_H
#define MY_MUJOCO_SIMULATOR_WBIC_H

#include <lcm/lcm-cpp.hpp>

#include "../QuadProg++/QuadProg++.hh"
#include "WBC_Base.h"

template <typename T>
struct WBIC_ExtraData{
    DVec<T> opt_result_;
    DVec<T> qqdot_;
    DVec<T> Fr_;
    DVec<T> w_floating_; // weight of floating
    DVec<T> w_rf_;
};

template<typename T>
class WBIC : public WBC_Base<T> {
public:
    WBIC(int num_vel, const std::vector<Contact_Task_Base<T> *> *constact_list,
         const std::vector<Task_Base<T> *> *task_list);

    virtual ~WBIC() = default;

    void UpdateSettings(const DMat<T> &Mq, const DMat<T> &Mqinv,
                        const DVec<T> &cqqd, void *extra_setting) override;

    void MakeTorque(DVec<T> &cmd, void *extra_input) override;

private:
    const std::vector<Contact_Task_Base<T> *> *contact_list_;
    const std::vector<Task_Base<T> *> *task_list_;

    void SetEqualityConstraint(const DVec<T> &qddot);

    void SetInEqualityConstraint();

    void ContactBuilding();

    void GetSolution(const DVec<T> &qddot, DVec<T> &cmd);

    void SetCost();

    void SetOptimizationSize();

    int32_t dim_opt_{};
    int32_t dim_eq_cstr_{}; // equality constraints
    int32_t dim_rf_{}; // dim of inequality constraints
    int32_t dim_uf_{};
    int32_t dim_floating_{};
    WBIC_ExtraData<T>* data_;

    // solver:
    GolDIdnani::GVect<T> z_;
    GolDIdnani::GMatr<T> G_;
    GolDIdnani::GVect<T> g0_;
    GolDIdnani::GMatr<T> CE_;
    GolDIdnani::GVect<T> ce0_;
    GolDIdnani::GMatr<T> CI_;
    GolDIdnani::GVect<T> ci0_;

    DMat<T> dyn_CE_;
    DVec<T> dyn_ce0_;
    DMat<T> dyn_CI_;
    DVec<T> dyn_ci0_;
    DMat<T> eye_;
    DMat<T> eye_floating_;
    DMat<T> S_delta_;
    DMat<T> uf_;
    DVec<T> uf_ieq_vec_;
    DMat<T> Jc_;
    DVec<T> Jcdqd_;
    DVec<T> Fr_des_;
    DMat<T> B_;
    DVec<T> c_;
    DVec<T> task_cmd_;
};

#endif //MY_MUJOCO_SIMULATOR_WBIC_H
