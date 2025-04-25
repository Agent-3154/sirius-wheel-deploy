//
// Created by lingwei on 5/20/24.
//
#include <iostream>
#include "KinWBC.h"

/**
 *
 * @param num_vel the dimension of the speed
 */
template<typename T>
KinWBC<T>::KinWBC(int num_vel) : threshould_(0.001), num_total_joints_(num_vel),
                                 num_act_joints_(num_vel - 6) {
    identity_matrix_ = DMat<T>::Identity(num_vel, num_vel);
}

template<typename T>
bool KinWBC<T>::FindConfiguration(const DVec<T> &curr_config, const std::vector<Task_Base<T> *> &task_list,
                                  const std::vector<Contact_Task_Base<T> *> &contact_list, DVec<T> &jpos_cmd,
                                  DVec<T> &jvel_cmd) {
    DMat<T> N_c(num_total_joints_, num_total_joints_);
    N_c.setIdentity();
    // contact task
    // composite Jacobian
    if (!contact_list.empty()) {
        DMat<T> Jc, Jc_ind;
        contact_list[0]->get_contact_jacobian(Jc);
//        std::cout << "Jc: \n" << Jc << std::endl;
        int num_rows = Jc.rows();
        for (size_t i = 1; i < contact_list.size(); i++) {
            contact_list[i]->get_contact_jacobian(Jc_ind);
            int num_new_rows = Jc_ind.rows();
            Jc.conservativeResize(num_rows + num_new_rows, num_total_joints_);
            Jc.block(num_rows, 0, num_new_rows, num_total_joints_) = Jc_ind;
            num_rows += num_new_rows;
        }
        BuildProjectMatrix(Jc, N_c);
    } // contact task done

    Vec18<T> delta_q, qdot;
    DMat<T> Jt, JtPre, JtPre_pinv, N_nx, N_pre;

    Task_Base<T> *task = task_list[0];
    task->getTaskJacobian(Jt);
//    std::cout << "Jt: \n" << Jt << std::endl;
    JtPre = Jt * N_c;
//    std::cout << "JtPre: \n" << JtPre << std::endl;
    pseudoInverse(JtPre, threshould_, JtPre_pinv);
//    std::cout << "JtPre_inv: \n" << JtPre_pinv << std::endl;
//    std::cout << "Pos Error: " << task->getPosError().transpose() << std::endl;
    delta_q = JtPre_pinv * task->getPosError();
//    std::cout << "delta_q: " << delta_q.transpose() << "\n";
    qdot = JtPre_pinv * task->getDesVel();
//    std::cout << "Qdot: " << qdot.transpose() << "\n";
    Vec18<T> prev_delta_q = delta_q;
    Vec18<T> prev_qdot = qdot;

    BuildProjectMatrix(JtPre, N_nx);
    N_pre = N_c * N_nx;

    for (size_t i(1); i < task_list.size(); ++i) {
//        std::cout << "loop: " << i << std::endl;
        task = task_list[i];
        task->getTaskJacobian(Jt);
//        std::cout << "Jt: \n" << Jt << std::endl;
        JtPre = Jt * N_pre;
//        std::cout << "JtPre: \n" << JtPre << std::endl;
        pseudoInverse(JtPre, threshould_, JtPre_pinv);
//        std::cout << "JtPre_inv: \n" << JtPre_pinv << std::endl;
        delta_q = prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q);
        qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot);

//        std::cout << "delta_q | qdot:" << delta_q.transpose() << "\n" <<qdot.transpose()<<std::endl;
        BuildProjectMatrix(JtPre, N_nx);
        N_pre *= N_nx;
        prev_delta_q = delta_q;
        prev_qdot = qdot;
    }
    for (int i = 0; i < num_act_joints_; ++i) {
        jpos_cmd[i] = curr_config[i + 6] + delta_q[i + 6];
        jvel_cmd[i] = qdot[i + 6];
//        std::cout << "jpos_cmd | jvelcmd: " << jpos_cmd[i] << " | " << jvel_cmd[i] << "\n";
    }
//    std::cout << "\n";
    return true;
}

/**
 * @brief Build Zero Space Matrix
 * @tparam T
 * @param J Jacobian Matrix
 * @param N Zero Space of the Jacobian
 */
template<typename T>
void KinWBC<T>::BuildProjectMatrix(const DMat<T> &J, DMat<T> &N) {
    DMat<T> J_pinv;
    pseudoInverse(J, threshould_, J_pinv);
    N = identity_matrix_ - J_pinv * J;
}

template
class KinWBC<double>;
