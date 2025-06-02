#pragma once

#include <memory>
#include "../../utilities/types/hardware_types.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "onnxruntime_cxx_api.h"
class RLController2 {
public:
    RLController2() = default;
    ~RLController2() = default;

    /**
     * Initialize the RL controller
     * @return true if initialization successful, false otherwise
     */
    bool init();

    /**
     * Start the RL controller
     * @return true if step successful, false otherwise  
     */
    bool step(Vec19<double>* joint_q, Vec18<double>* joint_qd, Vec3<double>* accel, Vec3<double>* desired_vel_xyw);

    /**
     * Stop the RL controller
     * @return true if stop successful, false otherwise
     */
    bool stop();
    /**
     * Load a policy from file
     * @param policy_path Path to the policy file
     * @return true if policy loaded successfully, false otherwise
     */
    bool loadPolicy(const std::string& policy_path);
    Vec12<double> desired_positions;
private:
    bool initialized_ = false;
    bool running_ = false;
    Vec12<double> last_action=Vec12<double>::Zero(); // Stores the last action taken by the controller
    Vec12<double> default_dof_pos = (Vec12<double>() << 
        0.1,  0.4, -1.2,  // FR leg (hip, thigh, calf)
        0.1,  -0.4, 1.2,  // FL leg 
        -0.1,  0.4, -1.2,  // RR leg
        -0.1,  -0.4, 1.2   // RL leg
    ).finished();
    Vec4<double> leg_theta;
    Eigen::Matrix<double, 8, 1, Eigen::DontAlign> leg_xy;
    double period = 0.6;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> vel_commands;
    Eigen::Matrix<double, 4, 1, Eigen::DontAlign> gait_schedule{0,M_PI,M_PI,0};
    int num_history_steps = 5;
    Eigen::Matrix<double, 41, 1, Eigen::DontAlign> observation;
    Eigen::Matrix<double, 205, 1, Eigen::DontAlign> observation_history;
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::unique_ptr<Ort::Session> session_;

    uint64_t step_counter = 0;
};
