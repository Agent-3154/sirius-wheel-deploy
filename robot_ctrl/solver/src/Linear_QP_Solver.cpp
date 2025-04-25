//
// Created by lingwei on 6/3/24.
//
#include "../inc/Linear_QP_Solver.h"

#include <easylogging++.h>
#include <iomanip>
#include <std_cout_colors.h>

#include "../../../config/Config.h"
#include "../../../utilities/inc/utilities_fun.h"
#include "../../../config/robots_config.h"

void Linear_QP_Solver::setup_problem(double dt, int horizon, double mu, double f_max) {
    problem_setup_.dt_ = dt;
    problem_setup_.horizon_ = horizon;
    problem_setup_.f_max_ = f_max;
    problem_setup_.mu_ = mu;
    resize_qp_mats(horizon);
}

void Linear_QP_Solver::resize_qp_mats(const int horizon) {
    A_qp_.resize(13 * horizon, Eigen::NoChange);
    B_qp_.resize(13 * horizon, 12 * horizon);
    S_.resize(13 * horizon, 13 * horizon);
    X_d_.resize(13 * horizon, Eigen::NoChange);
    U_b_.resize(20 * horizon, Eigen::NoChange);
    fmat_.resize(20 * horizon, 12 * horizon);
    qH_.resize(12 * horizon, 12 * horizon);
    qg_.resize(12 * horizon, Eigen::NoChange);
    eye_12h_.resize(12 * horizon, 12 * horizon);

    A_qp_.setZero();
    B_qp_.setZero();
    S_.setZero();
    X_d_.setZero();
    U_b_.setZero();
    fmat_.setZero();
    qH_.setZero();
    eye_12h_.setIdentity();

    if (re_allocated_) {
        delete [] H_qpoases_;
        delete [] g_qpoases_;
        delete [] A_qpoases_;
        delete [] lb_qpoases_;
        delete [] ub_qpoases_;
        delete [] q_soln_;
        delete [] H_red_;
        delete [] g_red_;
        delete [] A_red_;
        delete [] lb_red_;
        delete [] ub_red_;
        delete [] q_red_;
    }

    H_qpoases_ = new qpOASES::real_t[12 * 12 * horizon * horizon * sizeof(qpOASES::real_t)];
    g_qpoases_ = new qpOASES::real_t[12 * 1 * horizon * sizeof(qpOASES::real_t)];
    A_qpoases_ = new qpOASES::real_t[12 * 20 * horizon * horizon * sizeof(qpOASES::real_t)];
    lb_qpoases_ = new qpOASES::real_t[20 * 1 * horizon * sizeof(qpOASES::real_t)];
    ub_qpoases_ = new qpOASES::real_t[20 * 1 * horizon * sizeof(qpOASES::real_t)];
    q_soln_ = new qpOASES::real_t[12 * horizon * sizeof(qpOASES::real_t)];
    H_red_ = new qpOASES::real_t[12 * 12 * horizon * horizon * sizeof(qpOASES::real_t)];
    g_red_ = new qpOASES::real_t[12 * 1 * horizon * sizeof(qpOASES::real_t)];
    A_red_ = new qpOASES::real_t[12 * 20 * horizon * horizon * sizeof(qpOASES::real_t)];
    lb_red_ = new qpOASES::real_t[20 * 1 * horizon * sizeof(qpOASES::real_t)];
    ub_red_ = new qpOASES::real_t[20 * 1 * horizon * sizeof(qpOASES::real_t)];
    q_red_ = new qpOASES::real_t[12 * horizon * sizeof(qpOASES::real_t)];
    re_allocated_ = true;
}

void Linear_QP_Solver::continius_to_qp(const Eigen::Matrix<double, 13, 13>& Ac, Eigen::Matrix<double, 13, 12> Bc, double dt,
                                       int horizon) {
    ABc_.setZero();
    ABc_.block(0, 0, 13, 13) = Ac;
    ABc_.block(0, 13, 13, 12) = Bc;
    // std::cout << "ABc:\n" << ABc_ << std::endl;
    ABc_ = dt * ABc_;
    expmm_ = ABc_.exp(); //what's this?
    Adt_ = expmm_.block(0, 0, 13, 13);
    Bdt_ = expmm_.block(0, 13, 13, 12);
    // std::cout << "Adt:\n";
    //     std::cout << Adt_ <<"\n";
    //     std::cout << "Bdt:\n";
    //     std::cout << Bdt_ <<"\n";
    // A Matrix and B Matrix recursive.
    Eigen::Matrix<double, 13, 13> powerMats[Config::HatAPowerIndex];
    powerMats[0].setIdentity();
    for (int i = 1; i < horizon + 1; i++) {
        powerMats[i] = Adt_ * powerMats[i - 1];
    }
    for (int r = 0; r < horizon; r++) {
        A_qp_.block(13 * r, 0, 13, 13) = powerMats[r + 1];
        for (int j = 0; j < horizon; j++) {
            if (r >= j) {
                int a_num = r - j;
                B_qp_.block(13 * r, 12 * j, 13, 12) = powerMats[a_num] * Bdt_;
            }
        }
    }
}

void Linear_QP_Solver::continue_ss_mats(const Eigen::Matrix<double, 3, 3> &I_world, Eigen::Matrix<double, 3, 4> r_feets,
                                        double m, Eigen::Matrix<double, 3, 3> R_yaw, Eigen::Matrix<double, 13, 13> &A,
                                        Eigen::Matrix<double, 13, 12> &B,
                                        double x_drag) {
    // AB check ok
    A.setZero();
    A(3, 9) = 1.f;
    A(11, 9) = x_drag;
    A(4, 10) = 1.f;
    A(5, 11) = 1.f;
    A(11, 12) = 1.f;
    A.block(0, 6, 3, 3) = R_yaw.transpose();
    B.setZero();
    Eigen::Matrix<double, 3, 3> I_inv = I_world.inverse();
    for (int b = 0; b < 4; b++) {
        B.block(6, b * 3, 3, 3) = cross_mat(I_inv, r_feets.col(b));
        B.block(9, b * 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() / m;
    }
}

//TODO rewrite here as a thread
double Linear_QP_Solver::get_solution(int index) const {
    if (!solved) {
        std::cout << RED << "[Error: ] " << RESET << "Unsolved\n";
        return 0.;
    } else {
        // solved = false;
        return q_soln_[index];
    }
}

void Linear_QP_Solver::solve_problem() {
    // get state: check ok.
    // std::cout << "******************* problem ****************************\n";
    // std::cout << "f_max: " << problem_setup_.f_max_ << "\nmu: " << problem_setup_.mu_ <<
    //     "\n horizon: " << problem_setup_.horizon_ << "\n dt: " << problem_setup_.dt_ << std::endl;
    eigen_data_.set(update_state_.p_, update_state_.v_, update_state_.q_, update_state_.w_,
                    update_state_.r_, update_state_.yaw_, update_state_.I_body_, update_state_.m_);
    Eigen::Matrix<double, 3, 1> rpy = ori::quatToRPY(eigen_data_.q_);
    // std::cout << "rpy: " << rpy.transpose() << std::endl;
    Eigen::Matrix<double, 13, 1> x_0;
    //TODO check here again
    x_0 << rpy(0), rpy(1), rpy(2), eigen_data_.p_, eigen_data_.w_, eigen_data_.v_, -9.81;
    // std::cout << std::setprecision(6)<< "x_0 state: " << x_0.transpose() << std::endl;
    Eigen::Matrix<double, 3, 3> I_world = eigen_data_.R_yaw_ * eigen_data_.I_body_ * eigen_data_.R_yaw_.transpose();
    Eigen::Matrix<double, 13, 13> A_ct;
    Eigen::Matrix<double, 13, 12> B_ct_r;
    A_ct.setZero();
    B_ct_r.setZero();
    // generate matrix
    continue_ss_mats(I_world, eigen_data_.r_feet_, eigen_data_.m_, eigen_data_.R_yaw_, A_ct, B_ct_r,
                     update_state_.x_drag_);
    // std::cout << "A matrix: \n";
    // std::cout << A_ct << std::endl;
    // std::cout << "B matrix: \n";
    // std::cout << B_ct_r << std::endl;

    continius_to_qp(A_ct, B_ct_r, problem_setup_.dt_, problem_setup_.horizon_);
    // weight
    Eigen::Matrix<double, 13, 1> full_weight;
    for (int i = 0; i < 12; i++)
        full_weight(i) = update_state_.weights_[i];
    full_weight(12) = 0.f;
    S_.diagonal() = full_weight.replicate(problem_setup_.horizon_, 1);

    for (int i = 0; i < problem_setup_.horizon_; i++) {
        for (int j = 0; j < 12; j++) {
            X_d_(13 * i + j, 0) = update_state_.traj_[12 * i + j];
            // std::cout << X_d_(13 * i + j, 0) << " | ";
        }
    }
    // std::cout << "\n";
    // what's this?
    int k = 0;
    // std::cout << "ub:\n";
    for (int i = 0; i < problem_setup_.horizon_; i++) {
        for (int j = 0; j < 4; j++) {
            k = i * 4 + j;
            U_b_(5 * k + 0) = BIG_NUMBER;
            U_b_(5 * k + 1) = BIG_NUMBER;
            U_b_(5 * k + 2) = BIG_NUMBER;
            U_b_(5 * k + 3) = BIG_NUMBER;
            U_b_(5 * k + 4) = update_state_.gait_[i * 4 + j] * problem_setup_.f_max_;
            // std::cout << U_b_(5 * k + 0) << " | " << U_b_(5 * k + 1)
            //         << " | " << U_b_(5 * k + 2) << " | " << U_b_(5 * k + 3) << " | "
            //         << U_b_(5 * k + 4) << " | " << k << "\n";
        }
    }
    // std::cout << std::endl;
    const double mu = 1. / problem_setup_.mu_;
    Eigen::Matrix<double, 5, 3> f_block;
    f_block << mu, 0, 1.,
            -mu, 0, 1.,
            0, mu, 1.,
            0, -mu, 1.,
            0, 0, 1.;
    for (int i = 0; i < problem_setup_.horizon_ * 4; i++) {
        fmat_.block(i * 5, i * 3, 5, 3) = f_block;
    }
    // prepare the qp matrix
    qH_ = 2 * (B_qp_.transpose() * S_ * B_qp_ + update_state_.alpha_ * eye_12h_);
    qg_ = 2 * B_qp_.transpose() * S_ * (A_qp_ * x_0 - X_d_);
    // std::cout << "qH: \n";
    // std::cout << qH_;
    // std::cout << "\n qg_:\n";
    // std::cout << qg_ << "\n";

    int horizon = problem_setup_.horizon_;
    matrix_to_real(H_qpoases_, qH_, horizon * 12, horizon * 12);
    matrix_to_real(g_qpoases_, qg_, horizon * 12, 1);
    matrix_to_real(A_qpoases_, fmat_, horizon * 20, horizon * 12);
    matrix_to_real(ub_qpoases_, U_b_, horizon * 20, 1); // ub check ok.

    // std::cout << "check ub:\n";
    for (int i = 0; i < 20 * horizon; i++) {
        lb_qpoases_[i] = 0.0f;
        // std::cout << ub_qpoases_[i] << " | ";
    }
    // std::cout << "\n";

    int num_constraints = 20 * horizon;
    int num_variables = 12 * horizon;

    qpOASES::int_t nWSR = 100;
    // TODO Figure out how to reduce?
    int new_vars = num_variables;
    int new_cons = num_constraints;

    for (int i = 0; i < num_constraints; i++)
        con_elim[i] = 0;
    for (int i = 0; i < num_variables; i++)
        var_elim[i] = 0;

    for (int i = 0; i < num_constraints; i++) {
        if (!(near_zero(lb_qpoases_[i]) && near_zero(ub_qpoases_[i]))) continue;
        double *c_row = &A_qpoases_[i * num_variables];
        for (int j = 0; j < num_variables; j++) {
            if (near_one(c_row[j])) {
                new_vars -= 3;
                new_cons -= 5;
                int cs = (j * 5) / 3 - 3;
                var_elim[j - 2] = 1;
                var_elim[j - 1] = 1;
                var_elim[j] = 1;
                con_elim[cs] = 1;
                con_elim[cs + 1] = 1;
                con_elim[cs + 2] = 1;
                con_elim[cs + 3] = 1;
                con_elim[cs + 4] = 1;
            }
        }
    }
    //if(new_vars != num_variables)
    int var_ind[new_vars];
    int con_ind[new_cons];
    int vc = 0;
    for (int i = 0; i < num_variables; i++) {
        if (!var_elim[i]) {
            if (!(vc < new_vars)) {
                printf("BAD ERROR 1\n");
            }
            var_ind[vc] = i;
            vc++;
        }
    }
    vc = 0;
    for (int i = 0; i < num_constraints; i++) {
        if (!con_elim[i]) {
            if (!(vc < new_cons)) {
                printf("BAD ERROR 1\n");
            }
            con_ind[vc] = i;
            vc++;
        }
    }
    for (int i = 0; i < new_vars; i++) {
        int olda = var_ind[i];
        g_red_[i] = g_qpoases_[olda];
        for (int j = 0; j < new_vars; j++) {
            int oldb = var_ind[j];
            H_red_[i * new_vars + j] = H_qpoases_[olda * num_variables + oldb];
        }
    }

    for (int con = 0; con < new_cons; con++) {
        for (int st = 0; st < new_vars; st++) {
            double cval = A_qpoases_[(num_variables * con_ind[con]) + var_ind[st]];
            A_red_[con * new_vars + st] = cval;
        }
    }
    for (int i = 0; i < new_cons; i++) {
        int old = con_ind[i];
        ub_red_[i] = ub_qpoases_[old];
        lb_red_[i] = lb_qpoases_[old];
    }

    qpOASES::QProblem problem_red(new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);

    int rval = problem_red.init(H_red_, g_red_, A_red_, nullptr, nullptr, lb_red_, ub_red_, nWSR);
    (void) rval;
    int rval2 = problem_red.getPrimalSolution(q_red_);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");

    // LOG(INFO) << "QP Value: " << problem_red.getObjVal();
    vc = 0;
    for (int i = 0; i < num_variables; i++) {
        if (var_elim[i]) {
            q_soln_[i] = 0.0f;
        } else {
            q_soln_[i] = q_red_[vc];
            vc++;
        }
    }
}
