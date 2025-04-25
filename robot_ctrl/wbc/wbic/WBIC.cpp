//
// Created by lingwei on 5/20/24.
//
#include "WBIC.h"
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>

template<typename T>
void WBIC<T>::SetOptimizationSize() {
    dim_rf_ = dim_uf_ = 0;
    for (size_t i = 0; i < (*contact_list_).size(); i++) {
        dim_rf_ += ((*contact_list_)[i])->get_dim_contact();
        dim_uf_ += ((*contact_list_)[i])->get_dim_RFConstrain();
    }

    // dim_rf_ is not equal to 3, is a dynamic number
    //    std::cout << "Dim_floating: " << dim_floating_ << std::endl;
    dim_opt_ = dim_floating_ + dim_rf_; // only includes the contact foot
    dim_eq_cstr_ = dim_floating_;

    G_.resize(0, dim_opt_, dim_opt_);
    g0_.resize(0, dim_opt_);
    CE_.resize(0., dim_opt_, dim_eq_cstr_);
    ce0_.resize(0., dim_eq_cstr_);

    // Eigen Matrix Setting
    dyn_CE_ = DMat<T>::Zero(dim_eq_cstr_, dim_opt_);
    dyn_ce0_ = DVec<T>(dim_eq_cstr_);
    if (dim_rf_ > 0) {
        CI_.resize(0., dim_opt_, dim_uf_);
        ci0_.resize(0., dim_uf_);
        dyn_CI_ = DMat<T>::Zero(dim_uf_, dim_opt_);
        dyn_ci0_ = DVec<T>(dim_uf_);
        Jc_ = DMat<T>(dim_rf_, WBC_Base<T>::num_joint_total_);
        Jcdqd_ = DVec<T>(dim_rf_);
        Fr_des_ = DVec<T>(dim_rf_);
        uf_ = DMat<T>(dim_uf_, dim_uf_);
        uf_.setZero();
        uf_ieq_vec_ = DVec<T>(dim_uf_);
    } else {
        CI_.resize(0., dim_opt_, 1);
        ci0_.resize(0., 1);
    }
}

template<typename T>
void WBIC<T>::SetCost() {
    int idx_offset = 0;
    for (int i = 0; i < dim_floating_; ++i) {
        G_[i + idx_offset][i + idx_offset] = data_->w_floating_[i];
    }
    idx_offset += dim_floating_;
    for (int i(0); i < dim_rf_; ++i) {
        G_[i + idx_offset][i + idx_offset] = data_->w_rf_[i];
    }
}

template<typename T>
void WBIC<T>::GetSolution(const DVec<T> &qddot, DVec<T> &cmd) {
    DVec<T> tot_tau;
    if (dim_rf_ > 0) {
        data_->Fr_ = DVec<T>(dim_rf_);
        // get Reaction forces
        //        std::cout << "Fr_output: ";
        for (int i(0); i < dim_rf_; ++i) {
            data_->Fr_[i] = z_[i + dim_floating_] + Fr_des_[i];
            //            std::cout << data_->Fr_[i] << " | ";
        }
        //        std::cout << "\n\n";
        tot_tau = WBC_Base<T>::Mq_ * qddot + WBC_Base<T>::Cqqd_ - Jc_.transpose() * data_->Fr_;
    } else {
        tot_tau = WBC_Base<T>::Mq_ * qddot + WBC_Base<T>::Cqqd_;
    }
    //    Vec18<T> tem_1 = WBC_Base<T>::Mq_ * qddot;
    //    Vec18<T> tem_2 = WBC_Base<T>::Cqqd_;
    //    Vec18<T> tem_3 = Jc_.transpose() * data_->Fr_;
    //    std::cout << "check J_C:\n" << Jc_ << "\n";
    data_->qqdot_ = qddot;
    //    std::cout << "First:\n" << tem_1.transpose() << "\nSecond:\n" << tem_2.transpose() << "\nThird:\n"
    //              << tem_3.transpose() << "\n";
    cmd = tot_tau.tail(WBC_Base<T>::num_act_joint_);
}

/**
 * @brief update equation and total Jacobian Matrix
 * @tparam T
 */
template<typename T>
void WBIC<T>::ContactBuilding() {
    DMat<T> Uf;
    DVec<T> Uf_ieq_vec;
    DMat<T> Jc;
    DVec<T> JcDotQdot;
    (*contact_list_)[0]->get_contact_jacobian(Jc);
    (*contact_list_)[0]->get_contact_jdqd(JcDotQdot);
    (*contact_list_)[0]->getRFConstraintMtx(Uf);
    (*contact_list_)[0]->getRFConstraintVec(Uf_ieq_vec);
    //    std::cout << "Jc1:\n" << Jc << "\n";
    size_t dim_accumul_rf = (*contact_list_)[0]->get_dim_contact(); // 3
    size_t dim_accumul_uf = (*contact_list_)[0]->get_dim_RFConstrain(); // 3

    Jc_.block(0, 0, dim_accumul_rf, WBC_Base<T>::num_joint_total_) = Jc;
    Jcdqd_.head(dim_accumul_rf) = JcDotQdot;
    uf_.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
    uf_ieq_vec_.head(dim_accumul_uf) = Uf_ieq_vec;
    Fr_des_.head(dim_accumul_rf) = (*contact_list_)[0]->getRFDesired();

    for (size_t i(1); i < (*contact_list_).size(); ++i) {
        (*contact_list_)[i]->get_contact_jacobian(Jc);
        (*contact_list_)[i]->get_contact_jdqd(JcDotQdot);
        //        std::cout << "Jc:" << i << "\n" << Jc << "\n";
        size_t dim_new_rf = (*contact_list_)[i]->get_dim_contact();
        size_t dim_new_uf = (*contact_list_)[i]->get_dim_RFConstrain();

        // Jc append
        Jc_.block(dim_accumul_rf, 0, dim_new_rf, WBC_Base<T>::num_joint_total_) = Jc;

        // JcDotQdot append
        Jcdqd_.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

        // Uf
        (*contact_list_)[i]->getRFConstraintMtx(Uf);
        uf_.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

        // Uf inequality vector
        (*contact_list_)[i]->getRFConstraintVec(Uf_ieq_vec);
        uf_ieq_vec_.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

        // Fr desired
        Fr_des_.segment(dim_accumul_rf, dim_new_rf) = (*contact_list_)[i]->getRFDesired();
        dim_accumul_rf += dim_new_rf;
        dim_accumul_uf += dim_new_uf;
    }
    //    std::cout << "FR_DES: " << Fr_des_.transpose() << std::endl;
}

template<typename T>
void WBIC<T>::SetInEqualityConstraint() {
    dyn_CI_.block(0, dim_floating_, dim_uf_, dim_rf_) = uf_;
    dyn_ci0_ = uf_ieq_vec_ - uf_ * Fr_des_;
    for (int i(0); i < dim_uf_; ++i) {
        for (int j(0); j < dim_opt_; ++j) {
            CI_[j][i] = dyn_CI_(i, j);
        }
        ci0_[i] = -dyn_ci0_[i];
    }
}

/**
 * @brief
 * @tparam T
 * @param cmd
 * @param extra_input
 */
int count = 0;
template<typename T>
void WBIC<T>::MakeTorque(DVec<T> &cmd, void *extra_input) {
    if (extra_input != nullptr) {
        data_ = static_cast<WBIC_ExtraData<T> *>(extra_input);
    }
    SetOptimizationSize();
    SetCost();

    DVec<T> qdd_pre;
    DMat<T> Jc_bar;
    DMat<T> Npre; // zero space
    /**
     * @note get JtBar: M^{-1}J^T(JM^{-1}J^T)^{-1}, this Jocobian is only for acceleration
     * N = I - J^{+}J
     * Neq = N_{pre} N(Jt * N_{pre}), thus Jt_{pre} = Jt * N_{pre}
     * Neq = Neq * (I - Jt_{pre}^{+} * Jtpre)
     * for acceleration: Jt_{pre}^{+} = JtBar, the formula is the same as velocity and position.
     */
    // std::cout << "dim_rf: " << dim_rf_ << std::endl;
    if (dim_rf_ > 0) {
        ContactBuilding(); // build Jc_ in this function
        SetInEqualityConstraint();
        //        std::cout << "Jc_ :\n" << Jc_ << "\nMqinv_:\n" <<WBC_Base<T>::Mqinv_ << "\n";
        WBC_Base<T>::WeightedInverse(Jc_, WBC_Base<T>::Mqinv_, Jc_bar);
        qdd_pre = Jc_bar * (-Jcdqd_);
        Npre = eye_ - Jc_bar * Jc_;
    } else {
        qdd_pre = DVec<T>::Zero(WBC_Base<T>::num_joint_total_);
        Npre = eye_;
    }
    //    std::cout << "Jc_Bar:\n" << Jc_bar << "\n";
    //    std::cout << "Jcdqd:\n" << Jcdqd_ <<"\n";
    // std::cout << "\nqdd_pre_1:  " << qdd_pre.transpose() << std::endl;
    // next task
    DMat<T> Jt, JtBar, JtPre;
    DVec<T> JtDotQdot, xddot;
    // iterative acceleration only.
    for (size_t i(0); i < (*task_list_).size(); ++i) {
        // std::cout << "task id: " << i << std::endl;
        Task_Base<T> *task = (*task_list_)[i];
        task->getTaskJacobian(Jt);
        // std::cout << Jt << "\n\n";
        task->getTaskJacobianDotQdot(JtDotQdot);
        // std::cout << JtDotQdot.transpose() << "\n\n";
        task->getCommand(xddot);

        JtPre = Jt * Npre;
        WBC_Base<T>::WeightedInverse(JtPre, WBC_Base<T>::Mqinv_, JtBar);
        qdd_pre += JtBar * (xddot - JtDotQdot - Jt * qdd_pre);
        Npre = Npre * (eye_ - JtBar * JtPre);
    }
    SetEqualityConstraint(qdd_pre);
    //    std::cout << "qdd_pre_3:  " << qdd_pre.transpose() << std::endl;
    T f = solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, z_);
    (void) f;
    for (int i(0); i < dim_floating_; ++i) qdd_pre[i] += z_[i];

    //    std::cout << "qdd_pre_4:  " << qdd_pre.transpose() << std::endl;
    GetSolution(qdd_pre, cmd);
    //    std::cout << "troque: " << cmd.transpose() << "\n";
    data_->opt_result_ = DVec<T>(dim_opt_);
    for (int i(0); i < dim_opt_; ++i) {
        data_->opt_result_[i] = z_[i];
    }
    //    std::cout << " opt result: " << data_->opt_result_.transpose() << std::endl;
}

template<typename T>
void WBIC<T>::UpdateSettings(const DMat<T> &Mq, const DMat<T> &Mqinv, const DVec<T> &cqqd, void *extra_setting) {
    (void) extra_setting;
    WBC_Base<T>::Mq_ = Mq;
    WBC_Base<T>::Mqinv_ = Mqinv;
    WBC_Base<T>::Cqqd_ = cqqd;
}

/**
 * @brief initiate WBIC
 * @tparam T
 * @param num_vel the number of joints.
 * @param constact_list
 * @param task_list
 */
template<typename T>
WBIC<T>::WBIC(int num_vel, const std::vector<Contact_Task_Base<T> *> *constact_list,
              const std::vector<Task_Base<T> *> *task_list) : WBC_Base<T>(num_vel), dim_floating_(6) {
    contact_list_ = constact_list;
    task_list_ = task_list;

    eye_ = DMat<T>::Identity(WBC_Base<T>::num_joint_total_, WBC_Base<T>::num_joint_total_);
    eye_floating_ = DMat<T>::Identity(dim_floating_, dim_floating_);
}

template<typename T>
void WBIC<T>::SetEqualityConstraint(const DVec<T> &qddot) {
    if (dim_rf_ > 0) {
        dyn_CE_.block(0, 0, dim_eq_cstr_, dim_floating_) =
                WBC_Base<T>::Mq_.block(0, 0, dim_floating_, dim_floating_);
        dyn_CE_.block(0, dim_floating_, dim_eq_cstr_, dim_rf_) = -WBC_Base<T>::Sv_ * Jc_.transpose();
        dyn_ce0_ = -WBC_Base<T>::Sv_ * (WBC_Base<T>::Mq_ * qddot + WBC_Base<T>::Cqqd_ - Jc_.transpose() * Fr_des_);
    } else {
        dyn_CE_.block(0, 0, dim_eq_cstr_, dim_floating_) =
                WBC_Base<T>::Mq_.block(0, 0, dim_floating_, dim_floating_);
        dyn_ce0_ = -WBC_Base<T>::Sv_ * (WBC_Base<T>::Mq_ * qddot + WBC_Base<T>::Cqqd_);
    }

    for (int i(0); i < dim_eq_cstr_; ++i) {
        for (int j(0); j < dim_opt_; ++j) {
            CE_[j][i] = dyn_CE_(i, j);
        }
        ce0_[i] = -dyn_ce0_[i];
    }
    //    std::cout << "qdd: " <<qddot.transpose() << "\n\n";
    //    std::cout << "Equality:\n" << dyn_CE_ << "\n" << "dyn_ce0:\n" << dyn_ce0_ << "\n";
    //    std::cout << "1: \n" << (WBC_Base<T>::Mq_ * qddot) << "\n\n" << "2: \n:" << WBC_Base<T>::Cqqd_.transpose() << "\n"
    //              << "3: \n" << Jc_.transpose() * Fr_des_ << "\n\n";
}

template
class WBIC<double>;
