//
// Created by lingwei on 5/30/24.
//
#include "linear_mpc.h"
#include <iostream>
#include <std_cout_colors.h>
#include "../../config/robots_config.h"
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "../../utilities/inc/LoadData.h"

void Linear_MPC::SetupCommand(const planner_desire<double> &data) {
    desire_data_.pBody_des_ = data.pBody_des_;
    desire_data_.vBody_des_ = data.vBody_des_;
    desire_data_.vBody_Ori_des_ = data.vBody_Ori_des_w_; // mpc use global velocity
    desire_data_.pBody_RPY_des_ = data.pBody_RPY_des_;
    for (int i = 0; i < 4; i++) {
        desire_data_.pFoot_[i] = data.pFoot_[i];
    }
    rpy_comp_ = data.rpy_comp_;
    currentGait_ = data.current_gait_;
    mpc_table_ = data.contact_table_;
    // std::cout << std::endl << desire_data_.pBody_des_.transpose()<<"\n";
    // std::cout << desire_data_.vBody_des_.transpose()<<"\n";
    // std::cout << desire_data_.vBody_Ori_des_.transpose() << "\n";
    // std::cout << desire_data_.pBody_RPY_des_.transpose()<<"\n";
}

Linear_MPC::Linear_MPC(double dt, int iteration_between_mpc) : iterationBetweenMPC_(iteration_between_mpc),
                                                               horizonLength_(Config::horizonLength),
                                                               dt_(dt), mpc_lcm_(getLcmUrl(255)) {
    dtMPC_ = dt * iteration_between_mpc;
    // TODO add the solver setup_problem
    rpy_comp_.setZero();
    desire_data_.setDesireZero();
    body_height_ = Config::mpc_height;
    qp_solver_ = new Linear_QP_Solver();
    qp_solver_->setup_problem(dtMPC_, horizonLength_, Config::mu, Config::f_max);
    this->get_linear_mpc_settings();
    currentGait_ = TROT;
    firstRun_ = true;
}

Linear_MPC::~Linear_MPC() {
    delete qp_solver_;
}

void Linear_MPC::setMPCtime(int iteration_per_mpc) {
    iterationBetweenMPC_ = iteration_per_mpc;
    dtMPC_ = dt_ * iteration_per_mpc;
}

void Linear_MPC::resetLinearMPC() {
    firstRun_ = true;
    iterationCounter_ = 0;
}

/**
 * @brief three phase: 1. get cmd, 2. update mpc 3. give out cmd
 * @param data
 */
void Linear_MPC::run(Control_FSM_Data &data, std::atomic_bool &update) {
    // std::cout << GREEN << "update Enter: " << update.load() << RESET << std::endl;
    if (update.load()) {
        updateMPCIfNeeded(mpc_table_, data);
        update.store(false);
    } else { std::cout << RED << "[MPC Error]: " << RESET << "cmd not fresh! Iter:" << iterationCounter_ << std::endl; }
    iterationCounter_++;
    // std::cout << "update: " << update.load() << std::endl;
}

void Linear_MPC::updateMPCIfNeeded(const int *mpcTable, Control_FSM_Data &data) {
    // std::cout << "iter: " << iterationCounter_ << "\n";
    // loop check ok.
    // if (iterationCounter_ % iterationBetweenMPC_ == 0) {
    // const double *p = data.estimators_->shared_esti_data_.result_->p_w_.data();
    if (currentGait_ == STAND) {
        double trajInitial[12] = {
            desire_data_.pBody_RPY_des_(0),
            desire_data_.pBody_RPY_des_(1),
            desire_data_.pBody_RPY_des_(2),
            desire_data_.pBody_des_(0),
            desire_data_.pBody_des_(1),
            desire_data_.pBody_des_(2),
            0, 0, 0, 0, 0, 0
        };
        for (int i = 0; i < horizonLength_; i++)
            for (int j = 0; j < 12; j++)
                trajectoryAll_[12 * i + j] = trajInitial[j];
    } else {
        double trajInitial[12] = {
            desire_data_.pBody_RPY_des_(0), // 0
            desire_data_.pBody_RPY_des_(1), // 1
            desire_data_.pBody_RPY_des_(2),
            //yawStart,    // 2
            desire_data_.pBody_des_(0), // 3
            desire_data_.pBody_des_(1), // 4
            desire_data_.pBody_des_(2), // 5
            0, // 6
            0, // 7
            desire_data_.vBody_Ori_des_(2), // 8
            desire_data_.vBody_des_(0), // 9
            desire_data_.vBody_des_(1), // 10
            0
        }; // 11

        // std::cout << "Desired: " << trajInitial[0] << " | " << trajInitial[1] << " | " << trajInitial[2] << " | "
        //                     << trajInitial[3] << " | " << trajInitial[4] << " | " << trajInitial[5] << " | "
        //                     << trajInitial[6] << " | " << trajInitial[7] << " | " << trajInitial[8] << " | "
        //                     << trajInitial[9] << " | " << trajInitial[10] << " | " << trajInitial[11] << "\n";

        for (int i = 0; i < horizonLength_; i++) {
            for (int j = 0; j < 12; j++)
                trajectoryAll_[12 * i + j] = trajInitial[j];

            if (i == 0) // start at current position  TODO consider not doing this
            {
                trajectoryAll_[2] = data.estimators_->shared_esti_data_.result_->rpy_[2];
                // std::cout << "here: " << trajectoryAll_[2] << std::endl;
            } else {
                trajectoryAll_[12 * i + 3] =
                        trajectoryAll_[12 * (i - 1) + 3] + dtMPC_ * desire_data_.vBody_des_[0];
                trajectoryAll_[12 * i + 4] =
                        trajectoryAll_[12 * (i - 1) + 4] + dtMPC_ * desire_data_.vBody_des_[1];
                trajectoryAll_[12 * i + 2] =
                        trajectoryAll_[12 * (i - 1) + 2] + dtMPC_ * desire_data_.vBody_Ori_des_(2);
                // std::cout << "vel : " << desire_data_.vBody_Ori_des_.transpose() << std::endl;
            }
        }
        // std::cout << "Desired: " << trajectoryAll_[0] << " | " << trajectoryAll_[1] << " | " << trajectoryAll_[2] <<
        //         " | "
        //         << trajectoryAll_[3] << " | " << trajectoryAll_[4] << " | " << trajectoryAll_[5] << " | "
        //         << trajectoryAll_[6] << " | " << trajectoryAll_[7] << " | " << trajectoryAll_[8] << " | "
        //         << trajectoryAll_[9] << " | " << trajectoryAll_[10] << " | " << trajectoryAll_[11] << "\n";
    }
    solveDenseMPC(mpcTable, data);
    firstRun_ = false;
}

// }

void Linear_MPC::solveDenseMPC(const int *mpcTable, Control_FSM_Data &data) {
    // double Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};
    const double yaw = data.estimators_->shared_esti_data_.result_->rpy_[2];
    double weights[12];
    for (int i = 0; i < 12; i++) {
        weights[i] = Q_[i];
    }

    // check ok
    const double *_p = data.estimators_->shared_esti_data_.result_->p_w_.data();
    const double *_v = data.estimators_->shared_esti_data_.result_->v_w_.data();
    const double *_w = data.estimators_->shared_esti_data_.result_->omega_w_.data();
    const double *_q = data.estimators_->shared_esti_data_.result_->q_ori_.data();

    // Vec3<double> debug_p_w = data.estimators_->shared_esti_data_.result_->p_w_;
    // Vec3<double> debug_v_w = data.estimators_->shared_esti_data_.result_->v_w_;
    // Vec3<double> debug_omega_w = data.estimators_->shared_esti_data_.result_->omega_w_;
    // Vec4<double> debug_q_ori = data.estimators_->shared_esti_data_.result_->q_ori_;
    // Vec3<double> rpy_temp = ori::quatToRPY(data.estimators_->shared_esti_data_.result_->q_ori_);
    // std::cout << "Esitmator RPY:" << rpy_temp.transpose() << std::endl;
    // for(int i = 0; i < 3; i++) {
    //     std::cout << "pos: " << _p[i] << " vel: " << _v[i] << " omega: " << _w[i] <<
    //         " quat: " << _q[i] << "\n";
    // }
    // std::cout << "Quat esti: ";
    // for(int i = 0; i < 4;i++) {
    //     std::cout << _q[i] << " | ";
    // }
    // std::cout << std::endl;

    double r[12];
    for (int i = 0; i < 12; i++)
        r[i] = desire_data_.pFoot_[i % 4][i / 4] - _p[i / 4];

    const double pz_err = _p[2] - body_height_;
    Vec3<double> vxy(_v[0], _v[1], 0);

    dtMPC_ = dt_ * iterationBetweenMPC_;
    qp_solver_->setup_problem(dtMPC_, horizonLength_, Config::mu, Config::f_max);
    qp_solver_->update_x_drag(x_comp_integral_);
    if (vxy[0] > 0.3 || vxy[0] < -0.3) {
        x_comp_integral_ += Config::x_drag * pz_err * dtMPC_ / vxy[0];
    }

    qp_solver_->update_problem_data(_p, _v, _q, _w, r, yaw, weights, trajectoryAll_, alpha_[0],
                                    mpcTable, I_body_, robot_mass_);
    for (int leg = 0; leg < 4; leg++) {
        Vec3<double> f;
        for (int axis = 0; axis < 3; axis++)
            f[axis] = qp_solver_->get_solution(leg * 3 + axis);
        f_ff_[leg] = -data.estimators_->shared_esti_data_.result_->r_b_ * f;
        desire_data_.Fr_des_[leg] = f; //fz should be positive
        // std::cout << "Leg id: " << leg << " | " << f.transpose() << std::endl;
    }

    // if (abs(desire_data_.Fr_des_[2][1]) > 20 || abs(desire_data_.Fr_des_[0][1]) > 20 || abs(desire_data_.Fr_des_[1][1]) > 20)
    // std::cout << "Force Error : pos" << debug_p_w.transpose() << " vel: " << debug_v_w.transpose() <<
    // " omega: " << debug_omega_w.transpose() << " quat: " << debug_q_ori.transpose() <<
    // " \nrx: " << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << "\nry" << r[4] << " " << r[5] << " " << r[6] << " " << r[7]
    // << "\nrz"
    // << r[8] << " " << r[9] << " " << r[10] << " " << r[11] << std::endl;
    mpc_lcm_publish();
}

void Linear_MPC::get_linear_mpc_settings() {
    const std::string filename = Config::path_2_config_directory + "config/Control_Parameters.info";
    const std::string setting_name = "Linear_MPC_Q";
    std::vector<double> mpc_rpy, mpc_pos, mpc_omega, mpc_vel;
    std::vector<int> verbose;
    loadData::loadStdVector(filename, setting_name + ".verbose", verbose, false);
    loadData::loadStdVector(filename, setting_name + ".mpc_rpy", mpc_rpy, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".mpc_p", mpc_pos, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".mpc_omega", mpc_omega, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".mpc_vel", mpc_vel, verbose[0]);
    loadData::loadStdVector(filename, setting_name + ".mpc_alpha", alpha_, verbose[0]);

    for (size_t i = 0; i < mpc_rpy.size(); i++) {
        Q_[i] = mpc_rpy[i];
        Q_[3 + i] = mpc_pos[i];
        Q_[6 + i] = mpc_omega[i];
        Q_[9 + i] = mpc_vel[i];
    }
}

void Linear_MPC::mpc_lcm_publish() {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            mpc_data_.force[i][j] = static_cast<float>(desire_data_.Fr_des_[i][j]);
        }
    }
    mpc_lcm_.publish("MPC_Channel", &mpc_data_);
}
