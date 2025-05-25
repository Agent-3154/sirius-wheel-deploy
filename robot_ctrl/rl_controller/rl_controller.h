#pragma once

#include <memory>
#include "../../utilities/types/hardware_types.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
class RLController {
public:
    RLController() = default;
    ~RLController() = default;

    /**
     * Initialize the RL controller
     * @return true if initialization successful, false otherwise
     */
    bool init();

    /**
     * Start the RL controller
     * @return true if step successful, false otherwise  
     */
    bool step(Vec19<double>* joint_q, Vec18<double>* joint_qd, Vec3<double>* accel);

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

private:
    bool initialized_ = false;
    bool running_ = false;
    Vec12<double> last_action=Vec12<double>::Zero(); // Stores the last action taken by the controller
    Vec12<double> default_dof_pos = (Vec12<double>() << 
        0.0,  0.8, -1.6,  // FR leg (hip, thigh, calf)
        0.0,  0.8, -1.6,  // FL leg 
        0.0,  0.8, -1.6,  // RR leg
        0.0,  0.8, -1.6   // RL leg
    ).finished();
    Vec4<double> leg_theta;
    Eigen::Matrix<double, 8, 1, Eigen::DontAlign> leg_xy;
    double period = 0.6;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> vel_commands;
    Eigen::Matrix<double, 4, 1, Eigen::DontAlign> gait_schedule{0,M_PI,M_PI,0};
    int num_history_steps = 5;
    Eigen::Matrix<double, 41, 1, Eigen::DontAlign> observation;
    Eigen::Matrix<double, 205, 1, Eigen::DontAlign> observation_history;

};
