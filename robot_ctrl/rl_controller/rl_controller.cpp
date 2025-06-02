#include "rl_controller.h"

bool RLController::init() {
    // Initialize default joint positions
    // Initialize leg phase angles based on gait schedule
    // Clear all vectors to zero
    vel_commands.setZero();
    leg_theta.setZero();
    last_action.setZero();
    step_counter = 0;

    for(int i = 0; i < 4; i++) {
        leg_theta[i] = gait_schedule[i];
    }
    initialized_ = true;
    running_ = false;
    loadPolicy("../models/policy.onnx");
    std::cout << "Policy Loaded" << std::endl;

    return true;
}

bool RLController::step(Vec19<double>* joint_q, Vec18<double>* joint_qd, Vec3<double>* accel, Vec3<double>* desired_vel_xyw) {
    
if (step_counter % 10 == 0) {

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
    Vec3<double> projected_gravity = (quat_eigen.inverse() * Eigen::Vector3d(0, 0, -1)).cast<double>();
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
    vel_commands = *desired_vel_xyw;
    period = 0.6;
    gait_schedule = Vec4<double>(0,M_PI,M_PI,0);
        // Update leg phase angles
        double time_step = 0.02;
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
    observation.segment<3>(0) = body_ang_vel*0.25;
    observation.segment<3>(3) = projected_gravity;
    observation.segment<3>(6) = vel_commands.cwiseProduct(Vec3<double>(2.0, 2.0, 0.25));
    observation.segment<12>(9) = (reordered_angles - default_dof_pos)*1.0;
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
    if(step_counter <= num_history_steps) {
        // Fill all history steps with current observation during initialization
        for(int j = 0; j < num_history_steps; j++) {
            for(int i = 0; i < observation.size(); i++) {
                observation_history[j*observation.size() + i] = observation[i];
            }
        }
    }
    // Add new observation at the end
    for(int i = 0; i < observation.size(); i++) {
        observation_history[(num_history_steps-1)*observation.size() + i] = observation[i];
    }
    // std::cout << "Observation: " << observation.transpose() << std::endl;
    // std::cout << "Observation History: " << observation_history.transpose() << std::endl;

    // Create input tensor
    std::vector<float> input_tensor_values(observation_history.size());
    // std::cout << "Observation History: " << observation_history.transpose() << std::endl;
    for (int i = 0; i < observation_history.size(); i++) {
        input_tensor_values[i] = static_cast<float>(observation_history[i]);
    }
    
    // Define input shape and create input tensor
    std::vector<int64_t> input_shape = {static_cast<int64_t>(observation_history.size())};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_tensor_values.data(), input_tensor_values.size(), &input_shape[0], 1);

    // Define input/output names
    const char* input_names[] = {"input"};
    const char* output_names[] = {"output"};

    // Run inference
    auto output_tensors = session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    // Get output data
    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    // std::cout << "Output: " << output_data[0] << std::endl;
    // Update last_action with policy output
    for (int i = 0; i < 12; i++) {
        last_action[i] = output_data[i];
    }
    //calculate the desire pos:
    // Scale actions and add default positions to get full joint commands
    Vec12<double> scaled_actions;
    for (int i = 0; i < 12; i++) {
        scaled_actions[i] = last_action[i] * 0.5 + default_dof_pos[i]; // Assuming action_scale is 0.5, adjust if needed
    }
    // tweak order:

    desired_positions.segment<3>(0) = scaled_actions.segment<3>(6);  // FR leg
    desired_positions.segment<3>(3) = scaled_actions.segment<3>(0);  // FL leg
    desired_positions.segment<3>(6) = scaled_actions.segment<3>(9);  // RR leg
    desired_positions.segment<3>(9) = scaled_actions.segment<3>(3);  // RL leg
}
    step_counter++;
    return true;
}

bool RLController::stop() {
    return true;
}

bool RLController::loadPolicy(const std::string& policy_path) {
    // 1. Initialize ONNX Runtime environment 
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "PolicyInference");
    session_options_ = std::make_unique<Ort::SessionOptions>();
    
    // (Optional) Enable CUDA if available
    // Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(*session_options_, 0));

    // 2. Load the ONNX model
    session_ = std::make_unique<Ort::Session>(*env_, policy_path.c_str(), *session_options_);
    return true;
}

