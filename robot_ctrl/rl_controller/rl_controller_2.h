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
    bool step(Vec23<double>* joint_q, Vec22<double>* joint_qd, Vec3<double>* accel, Vec3<double>* desired_vel_xyw, int* stand_flag);

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
    Vec16<double> desired_positions;
private:
    bool initialized_ = false;
    bool running_ = false;
    float offset_hip = 0.00;
    Vec16<double> last_action=Vec16<double>::Zero(); // Stores the last action taken by the controller
    Vec16<double> default_dof_pos = (Vec16<double>() << 
        0.0,  (0.7+offset_hip), -1.6, 0.0,  // LF leg (hip, thigh, calf)
        0.0,  -(0.7+offset_hip), 1.6, 0.0,  // RF leg 
        0.0,  (0.7+offset_hip), -1.6, 0.0,  // LH leg
        0.0,  -(0.7+offset_hip), 1.6, 0.0   // RH leg
    ).finished();
    Vec12<double> default_dof_pos_obs = (Vec12<double>() << 
        0.0,  (0.7+offset_hip), -1.6, // LF leg (hip, thigh, calf)
        0.0,  -(0.7+offset_hip), 1.6, // RF leg 
        0.0,  (0.7+offset_hip), -1.6, // LH leg
        0.0,  -(0.7+offset_hip), 1.6 // RH leg
    ).finished();
    Vec4<double> leg_theta;
    Eigen::Matrix<double, 8, 1, Eigen::DontAlign> leg_xy;
    double period = 0.6;
    double motion_time = 0.0;
    double motion_duration = 0.6;

    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> vel_commands;
    Eigen::Matrix<double, 4, 1, Eigen::DontAlign> gait_schedule{0,M_PI,M_PI,0};
    int num_history_steps = 5;
    Eigen::Matrix<double, 42, 1, Eigen::DontAlign> observation;
    Eigen::Matrix<double, 210, 1, Eigen::DontAlign> observation_history;
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::unique_ptr<Ort::Session> session_;

    uint64_t step_counter = 0;
};
