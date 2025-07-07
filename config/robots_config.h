//
// Created by lingwei on 5/30/24.
//

#ifndef MPC_CONFIG_H
#define MPC_CONFIG_H

#include "../../utilities/types/hardware_types.h"

namespace Config {
    // MPC Variables: not adjust often
    // control frequency: 500
    constexpr int horizonLength = 14;
    constexpr int HatAPowerIndex = horizonLength + 1;

    constexpr int trot_horizonLength = 14;
    const Vec4<int> trot_offset = {0, trot_horizonLength / 2, trot_horizonLength / 2, 0};
    const Vec4<int> trot_duration = {
        trot_horizonLength / 2, trot_horizonLength / 2, trot_horizonLength / 2, trot_horizonLength / 2
    };

    const Vec4<int> stand_offset = {0, 0, 0, 0};
    const Vec4<int> stand_duration = {14, 14, 14, 14};

    constexpr int walk_horizonLength = 24;
    const Vec4<int> walk_offset = {0, 12, 6, 18};
    const Vec4<int> walk_duration = {18, 18, 18, 18};

    constexpr int trot_running_horizonLength = 10;
    const Vec4<int> trot_running_offset = {0, 4, 4, 0};
    const Vec4<int> trot_running_duration = {5, 5, 5, 5};

    // **********************************************************************************************************
    // control frequency: 1000
    // constexpr int horizonLength = 12;
    // constexpr int HatAPowerIndex = horizonLength + 1;
    //
    // constexpr int trot_horizonLength = 20;
    // const Vec4<int> trot_offset = {0, trot_horizonLength / 2, trot_horizonLength / 2, 0};
    // const Vec4<int> trot_duration = {
    //     trot_horizonLength / 2, trot_horizonLength / 2, trot_horizonLength / 2, trot_horizonLength / 2
    // };
    //
    // const Vec4<int> stand_offset = {0, 0, 0, 0};
    // const Vec4<int> stand_duration = {horizonLength, horizonLength, horizonLength, horizonLength};
    //
    // constexpr int walk_horizonLength = 36;
    // const Vec4<int> walk_offset = {0, 18, 9, 27};
    // const Vec4<int> walk_duration = {27, 27, 27, 27};
    //
    // constexpr int trot_running_horizonLength = 12;
    // const Vec4<int> trot_running_offset = {0, 6, 6, 0};
    // const Vec4<int> trot_running_duration = {5, 5, 5, 5};

    // *********************************************************************************************************

    constexpr int mpc_thread_fre = 100;
    // dtMPC = mpc_iteration_segment * control_dt
    // total_horizon = horizonLength * dtMPC
    constexpr int mpc_iteration_segment = 25;
    // dtFoot = gait_iteration_segment * control_dt
    // total_time = gait_horizon * dtFoot
    constexpr int gait_iteration_segment = 25; // real time: segment*dt
#if defined BELT
    constexpr double Stand_Up_Height = 0.36;
    constexpr double Sit_Down_Height = 0.07;
    constexpr double mpc_inertia[9] = {
        0.49, 0.000079975911, -0.0878,
        0.000079975911, 1.58, 0.0000038257137,
        -0.0878, 0.0000038257137, 1.56
    };
    constexpr double mpc_height = 0.36;
    constexpr double mpc_weight = 23.7;
#elif defined CHAOJI_GO
    constexpr double Stand_Up_Height = 0.32;
    constexpr double Sit_Down_Height = 0.07;
    constexpr double mpc_inertia[9] = {
        0.14997310, 0.0, -0.0,
        -0.000, 0.51533219, 0.000,
        -0.000, 0.000, 0.60894391
    };
    constexpr double mpc_height = 0.32;
    constexpr double mpc_weight = 16;
#elif defined GO1
    constexpr double Stand_Up_Height = 0.3;
    constexpr double Sit_Down_Height = 0.07;
    constexpr double mpc_inertia[9] = {
        0.07, 0.000079975911, -0.0878,
        0.000079975911, 0.26, 0.0000038257137,
        -0.0878, 0.0000038257137, 0.242
        };
    constexpr double mpc_height = 0.3; //belt 0.38 go1:0.3
    constexpr double mpc_weight = 8; //belt 23.7 go1:8
#elif defined DG_ENGINEER
    constexpr double Stand_Up_Height = 0.45;
    constexpr double Sit_Down_Height = 0.07;
    constexpr double mpc_inertia[9] = {
        0.10767656, 0.00000033, -0.00871520,
        0.00000033, 2.82545726, -0.00000024,
        -0.00871520, -0.00000024, 2.85571634
        };
    constexpr double mpc_height = 0.42; //belt 0.38 go1:0.3
    constexpr double mpc_weight = 27; //belt 23.7 go1:8
    constexpr double HipLinkLength = 0.272;
    constexpr double KneeLinkLength = 0.284;
    constexpr double AbadLinkLength = 0.1408;
#elif defined SIRIUS_WHEEL
    constexpr double Stand_Up_Height = 0.45;
    constexpr double Sit_Down_Height = 0.07;
    constexpr double mpc_inertia[9] = {
        0.10767656, 0.00000033, -0.00871520,
        0.00000033, 2.82545726, -0.00000024,
        -0.00871520, -0.00000024, 2.85571634
        };
    constexpr double mpc_height = 0.42; //belt 0.38 go1:0.3
    constexpr double mpc_weight = 27; //belt 23.7 go1:8
    constexpr double HipLinkLength = 0.272;
    constexpr double KneeLinkLength = 0.284;
    constexpr double AbadLinkLength = 0.1408;
#endif
    constexpr double kp_cartesian = 350;
    constexpr double kp_cartesian_z = 150;
    constexpr double kd_cartesian = 10;

    constexpr double step_height = 0.08;
    constexpr double foot_reference_max = 0.3;
    constexpr double mpc_bonus_swing_x = 0.0;
    constexpr double gain_comp_3 = 0.08; // k_p(v - v_d)
    constexpr double gain_comp_4 = 0.5; // k_p(p[2]/g)*v x w

    //TODO Add these file to info, and add tunnel with simulation.
    constexpr double pf_z = -0.001;
    constexpr double f_max = 2500;
    constexpr double mu = 0.5;
    constexpr double x_drag = 0;
    constexpr double wbc_weight_base = 1000.0;
}

#endif //MPC_CONFIG_H
