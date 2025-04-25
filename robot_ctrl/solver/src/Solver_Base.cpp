//
// Created by lingwei on 6/3/24.
//
#include "../inc/Solver_Base.h"
#include <cstring>

void Solver_Base::update_problem_data(double const *p, double const *v, double const *q, double const *w,
                                      double const *r, double yaw, double const *weights,
                                      double const *state_trajectory, double alpha, int const *gait, const double *I_body,
                                      double m) {
    // TODO: these size should be parameterized
    memcpy(update_state_.p_, p, sizeof(double) * 3);
    memcpy(update_state_.v_, v, sizeof(double) * 3);
    memcpy(update_state_.q_, q, sizeof(double) * 4);
    memcpy(update_state_.w_, w, sizeof(double) * 3);
    memcpy(update_state_.r_, r, sizeof(double) * 12);
    memcpy(update_state_.weights_, weights, sizeof(double) * 12);
    memcpy(update_state_.traj_, state_trajectory, sizeof(double) * 12 * problem_setup_.horizon_);
    memcpy(update_state_.I_body_, I_body, sizeof(double) * 9);
    update_state_.alpha_ = alpha;
    update_state_.yaw_ = yaw;
    update_state_.m_ = m;
    // std::cout << "Robot mass: " << m << std::endl;
    // std::cout << "real gait\n";
    // for (int i = 0; i < 10; i++) {
    //     for (int j = 0; j < 4; j++) {
    //         std::cout << gait[4 * i + j] << " | ";
    //     }
    // }
    // std::cout << std::endl;
    memcpy(update_state_.gait_, gait, sizeof(int) * 4 * problem_setup_.horizon_);
    solve_problem();
    solved = true;
    //check data.
}

void Solver_Base::update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha,
                                         double terminate) {
    update_state_.max_iterations_ = max_iter;
    update_state_.rho_ = rho;
    update_state_.sigma_ = sigma;
    update_state_.solver_alpha_ = solver_alpha;
    update_state_.terminate_ = terminate;
}

void Solver_Base::update_x_drag(double x_drag) {
    update_state_.x_drag_ = x_drag;
}
