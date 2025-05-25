#include "rl_controller.h"

bool RLController::init() {
    // Initialize default joint positions
    // Initialize leg phase angles based on gait schedule
    for(int i = 0; i < 4; i++) {
        leg_theta[i] = gait_schedule[i];
    }
    initialized_ = true;
    running_ = false;
    return true;
}

bool RLController::step(Vec19<double>* joint_q, Vec18<double>* joint_qd, Vec3<double>* accel) {
    // Extract quaternion from joint_q (indices 3-6 contain q_w, q_x, q_y, q_z)
    Vec4<double> quat;
    quat[0] = (*joint_q)[3]; // w
    quat[1] = (*joint_q)[4]; // x  
    quat[2] = (*joint_q)[5]; // y
    quat[3] = (*joint_q)[6]; // z
    // Extract body angular velocity from joint_qd (first 3 elements)
    Vec3<double> body_ang_vel;
    body_ang_vel[0] = (*joint_qd)[3];
    body_ang_vel[1] = (*joint_qd)[4]; 
    body_ang_vel[2] = (*joint_qd)[5];

    Vec3<double> gravity_vector(0, 0, -9.81);
    Eigen::Quaterniond quat_eigen(quat[0], quat[1], quat[2], quat[3]);
    Vec3<double> projected_gravity = (quat_eigen.inverse() * Eigen::Vector3d(0, 0, -9.81)).cast<double>();
    // Reorder joint angles and velocities to match expected format
    Vec12<double> reordered_angles, reordered_vels;
    
    // Front legs (indices 3-6 and 9-12 in original)
    reordered_angles.segment<3>(0) = joint_q->segment<3>(7+3);   // FR leg
    reordered_angles.segment<3>(3) = joint_q->segment<3>(7+9);   // FL leg
    
    // Rear legs (indices 0-3 and 6-9 in original) 
    reordered_angles.segment<3>(6) = joint_q->segment<3>(7+0);   // RR leg
    reordered_angles.segment<3>(9) = joint_q->segment<3>(7+6);   // RL leg

    // // Front legs velocities
    // reordered_vels.segment<3>(0) = joint_qd->segment<3>(3+3);    // FR leg
    // reordered_vels.segment<3>(3) = joint_qd->segment<3>(3+9);    // FL leg
    
    // // Rear legs velocities
    // reordered_vels.segment<3>(6) = joint_qd->segment<3>(3+9);    // RR leg  
    // reordered_vels.segment<3>(9) = joint_qd->segment<3>(3+0);    // RL leg
    
    // update commands, period and gait schedule:
    vel_commands = Vec3<double>(0, 0, 0);
    period = 0.6;
    gait_schedule = Vec4<double>(0,M_PI,M_PI,0);
    // Update leg phase angles
    double time_step = 0.001; // 1ms timestep
    double delta_theta = time_step/period * 2*M_PI;
    // Update phase for each leg
    for(int i = 0; i < 4; i++) {
        leg_theta[i] += delta_theta;
        // Convert polar to cartesian coordinates
        leg_xy[2*i] = cos(leg_theta[i]);
        leg_xy[2*i+1] = sin(leg_theta[i]);
        // Update phase angle based on cartesian coordinates
        leg_theta[i] = atan2(leg_xy[2*i+1], leg_xy[2*i]);
    }
    // calculate current observation by concatenating:
    // 1. Base angular velocity (3)
    // 2. Projected gravity vector (3) 
    // 3. Velocity commands (3)
    // 4. Joint angle difference from default pose (12)
    // 5. Last actions taken (12)
    // 6. Leg phase angles in x-y coordinates (8)
    observation.segment<3>(0) = body_ang_vel;
    observation.segment<3>(3) = projected_gravity;
    observation.segment<3>(6) = vel_commands;
    observation.segment<12>(9) = reordered_angles - default_dof_pos;
    observation.segment<12>(21) = last_action;
    // Convert leg phases to x-y coordinates
    for(int i = 0; i < 4; i++) {
        observation[33 + i*2] = leg_xy[2*i]*0.1; // x coordinate
        observation[34 + i*2] = leg_xy[2*i+1]*0.1; // y coordinate
    }
    // Store current observation in history buffer
    // Shift old observations left and add new observation at end

    // We maintain 5 steps of history
    // Shift history left by one step
    for(int j = 0; j < num_history_steps-1; j++) {
        for(int i = 0; i < observation.size(); i++) {
            observation_history[j*observation.size() + i] = observation_history[(j+1)*observation.size() + i];
        }
    }
    // Add new observation at the end
    for(int i = 0; i < observation.size(); i++) {
        observation_history[(num_history_steps-1)*observation.size() + i] = observation[i];
    }
    // std::cout << "Observation: " << observation.transpose() << std::endl;
    // std::cout << "Observation History: " << observation_history.transpose() << std::endl;
    return true;
}

bool RLController::stop() {
    return true;
}

bool RLController::loadPolicy(const std::string& policy_path) {
    return true;
}

