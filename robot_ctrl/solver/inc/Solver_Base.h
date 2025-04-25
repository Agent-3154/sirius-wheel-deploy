//
// Created by lingwei on 6/3/24.
//

#ifndef SOLVER_BASE_H
#define SOLVER_BASE_H

#include <iostream>

#include "../../../utilities/inc/utilities_fun.h"
#include "eigen3/Eigen/Dense"

#define K_MAX_GAIT_SEGMENTS 100
#define BIG_NUMBER 5e10

typedef struct problem_setup {
    double dt_;
    double mu_;
    double f_max_;
    int horizon_;
} problem_setup_t;

typedef struct update_state {
    double p_[3];
    double v_[3];
    double q_[4];
    double w_[3];
    double r_[12];
    double yaw_;
    double weights_[12];
    double traj_[12 * K_MAX_GAIT_SEGMENTS];
    double alpha_;
    int gait_[K_MAX_GAIT_SEGMENTS];
    int max_iterations_;
    double rho_, sigma_, solver_alpha_, terminate_;
    double x_drag_, m_;
    double I_body_[9];
} update_state_t;

typedef struct Eigen_data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double yaw_, m_;
    Eigen::Matrix<double, 3, 1> p_, v_, w_;
    Eigen::Matrix<double, 3, 4> r_feet_;
    Eigen::Matrix<double, 3, 3> R_;
    Eigen::Matrix<double, 3, 3> R_yaw_;
    Eigen::Matrix<double, 3, 3> I_body_;
    Eigen::Matrix<double, 4, 1> q_;

    void set(const double *p, const double *v, const double *q, const double *w, const double *r, double yaw,
             const double *Ib, double m);
} eigen_update_state_t;

inline void Eigen_data::set(const double *p, const double *v, const double *q, const double *w, const double *r,
                            double yaw, const double *Ib, double m) {
    for (int i = 0; i < 3; i++) {
        this->p_(i) = p[i];
        this->v_(i) = v[i];
        this->w_(i) = w[i];
    }
    this->q_(0) = q[0];
    this->q_(1) = q[1];
    this->q_(2) = q[2];
    this->q_(3) = q[3];
    this->yaw_ = yaw;
    this->m_ = m;

    // std::cout << "\nr: \n";
    for (int rs = 0; rs < 3; rs++) {
        for (int c = 0; c < 4; c++) {
            this->r_feet_(rs, c) = r[rs * 4 + c];
        }
    }

    // std::cout << "Quat: ";
    // for(int i = 0; i < 4;i++) {
    //     std::cout << q_[i] << " | ";
    // }
    // std::cout << std::endl;

    this->R_ = ori::quaternionToRotationMatrix(this->q_);

    // Eigen::Matrix<double, 3, 4> r_temp = R_ * r_feet_;
    // std::cout << "\nr_feet ";
    // for(int i = 0; i < 4;i++) {
    //     std::cout << r_temp.col(i).transpose() << "\n ";
    // }
    // std::cout << std::endl;
    // std::cout << "RPY: " << temp_rpy.transpose() << std::endl;
    const double yc = cos(yaw_);
    const double ys = sin(yaw_);
    // std::cout << "yaw: " << yaw_ << std::endl;

    this->R_yaw_ << yc, -ys, 0,
            ys, yc, 0,
            0, 0, 1;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            this->I_body_(i, j) = Ib[3 * i + j];
        }
    }
    // std::cout << "Problem updata state: ////////////////////////////////////////////\n";
    // std::cout << "pos:" << p_.transpose() << std::endl;
    // std::cout << "vel:" << v_.transpose() << std::endl;
    // std::cout << "quat:" << q_.transpose() << std::endl;
    // std::cout << "omega:" << w_.transpose() << std::endl;
    // std::cout << "yaw:" << yaw_ << std::endl;
    // std::cout << "mass:" << m_ << std::endl;
    // std::cout << "Matrix: " << I_body_ << std::endl;
}


class Solver_Base {
public:
    Solver_Base() = default;

    virtual ~Solver_Base() = default;

    virtual void setup_problem(double dt, int horizon, double mu, double f_max) = 0;

    void update_problem_data(double const *p, double const *v, double const *q, double const *w,
                             double const *r, double yaw,
                             double const *weights,
                             double const *state_trajectory, double alpha, int const *gait,
                             const double *I_body, double m);

    void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate);

    void update_x_drag(double x_drag);

    virtual void solve_problem() =0;

    virtual double get_solution(int index) const = 0;

protected:
    problem_setup_t problem_setup_{};
    update_state_t update_state_{};
    eigen_update_state_t eigen_data_{};
    bool first_run{};
    mutable bool solved = false;
};

#endif //SOLVER_BASE_H
