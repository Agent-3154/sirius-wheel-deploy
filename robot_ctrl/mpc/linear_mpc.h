//
// Created by lingwei on 5/30/24.
//

#ifndef MY_MUJOCO_SIMULATOR_LINEAR_MPC_H
#define MY_MUJOCO_SIMULATOR_LINEAR_MPC_H

#include"mpc_base.h"
#include "../solver/inc/Linear_QP_Solver.h"
#include "../../lcm-types/cpp/mpc_lcmt.hpp"

class Linear_MPC : public MPC_Base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Linear_MPC(double dt_, int iteration_between_mpc);

    ~Linear_MPC() override;

    void run(Control_FSM_Data &data, std::atomic_bool &update) override;

    void setMPCtime(int iteration_per_mpc);

    void resetLinearMPC();

    bool use_wbc_ = true;
    Vec4<double> contact_state_;
    Vec4<double> swingStates;
    Vec3<double> f_ff_[4];

    void SetupCommand(const planner_desire<double> &data) override;

    volatile bool firstRun_{};

private:
    void updateMPCIfNeeded(const int *mpcTable, Control_FSM_Data &data);

    void solveDenseMPC(const int *mpcTable, Control_FSM_Data &data);

    void get_linear_mpc_settings();

    void mpc_lcm_publish() override;

    Solver_Base *qp_solver_;

    double body_height_{}, step_height_{};
    double Q_[12]{};
    Vec3<double> f_last_[4];

    int iterationBetweenMPC_, horizonLength_, iterationCounter_{};

    double dt_;
    //todo add foot trajectory

    gait_number currentGait_;
    int *mpc_table_{};
    Vec3<double> rpy_comp_;
    double x_comp_integral_ = 0;
    //X_drag? Configure the dimension of the vector
    double trajectoryAll_[12 * 36]{};
    double robot_mass_ = Config::mpc_weight;
    // TODO alter this
    const double *I_body_ = Config::mpc_inertia;
    lcm::LCM mpc_lcm_;
    mpc_lcmt mpc_data_{};
    std::vector<double> alpha_{};
};

#endif //MY_MUJOCO_SIMULATOR_LINEAR_MPC_H
