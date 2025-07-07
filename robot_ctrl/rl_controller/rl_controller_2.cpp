#include "rl_controller_2.h"

bool RLController2::init() {
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
    loadPolicy("../models/policy_whl.onnx");
    std::cout << "Policy Loaded" << std::endl;

    return true;
}

bool RLController2::step(Vec23<double>* joint_q, Vec22<double>* joint_qd, Vec3<double>* accel, Vec3<double>* desired_vel_xyw, int* stand_flag) {
    
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
        Vec12<double> reordered_angles;
        Vec4<double> reordered_vels;
        
        // Front legs (indices 3-6 and 9-12 in original)
        reordered_angles.segment<3>(0) = joint_q->segment<3>(11);   // LF leg 10
        reordered_angles.segment<3>(3) = joint_q->segment<3>(19);   // LH leg
        
        // Rear legs (indices 0-3 and 6-9 in original) 
        reordered_angles.segment<3>(6) = joint_q->segment<3>(7);   // RF leg
        reordered_angles.segment<3>(9) = joint_q->segment<3>(15);   // RH leg

        // whl velocities
        reordered_vels[0] = (*joint_qd)[13];  
        reordered_vels[1] = (*joint_qd)[21];   
        reordered_vels[2] = (*joint_qd)[9];
        reordered_vels[3] = (*joint_qd)[17];
        
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
        observation[9] = *stand_flag;
        observation.segment<12>(10) = (reordered_angles - default_dof_pos_obs)*1.0;
        observation.segment<4>(22) = reordered_vels*0.05;
        observation.segment<16>(26) = last_action;
    

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

        // Prepare input tensors for ONNX model
        std::vector<float> obs_tensor_values(42);
        std::vector<float> obs_history_tensor_values(210);
        
        // Current observation (42 values)
        for (int i = 0; i < 42; i++) {
            obs_tensor_values[i] = static_cast<float>(observation[i]);
        }
        
        // Observation history (210 values)
        for (int i = 0; i < 210; i++) {
            obs_history_tensor_values[i] = static_cast<float>(observation_history[i]);
        }
        
        // Define input shapes
        std::vector<int64_t> obs_shape = {1, 42};
        std::vector<int64_t> obs_history_shape = {1, 210};
        
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        
        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, obs_tensor_values.data(), obs_tensor_values.size(), 
            obs_shape.data(), obs_shape.size()));
            
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, obs_history_tensor_values.data(), obs_history_tensor_values.size(),
            obs_history_shape.data(), obs_history_shape.size()));

        // Define input/output names
        const char* input_names[] = {"obs", "obs_history"};
        const char* output_names[] = {"actions"};

        // Run inference
        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr}, 
            input_names, 
            input_tensors.data(), 
            input_tensors.size(), 
            output_names, 
            1);
        // Print the input tensors
        // std::cout << "===== Input Tensors =====" << std::endl;
        // std::cout << "obs tensor (shape: [1,42]):" << std::endl;
        // for (size_t i = 0; i < obs_tensor_values.size(); i++) {
        //     std::cout << obs_tensor_values[i] << " ";
        //     if ((i+1) % 6 == 0) std::cout << std::endl;  // Print 6 values per line
        // }
        // Get output data
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        
        // Update last_action with policy output
        for (int i = 0; i < 16; i++) {
            last_action[i] = output_data[i];
        }
        
        //calculate the desire pos:
        // Scale actions and add default positions to get full joint commands
        Vec16<double> scaled_actions;
        for (int i = 0; i < 16; i++) {
            if (i == 3 || i == 7 || i == 11 || i == 15) {
                scaled_actions[i] = last_action[i] * 10.0 + default_dof_pos[i]; // Assuming action_scale is 10.0, adjust if needed
            }
            else {
                scaled_actions[i] = last_action[i] * 0.5 + default_dof_pos[i]; // Assuming action_scale is 0.5, adjust if needed
            }
        }

        // tweak order:
        desired_positions.segment<3>(0) = scaled_actions.segment<3>(8);  // FR leg
        desired_positions.segment<3>(3) = scaled_actions.segment<3>(0);  // FL leg
        desired_positions.segment<3>(6) = scaled_actions.segment<3>(12);  // RR leg
        desired_positions.segment<3>(9) = scaled_actions.segment<3>(4);  // RL leg
        desired_positions[12] = scaled_actions[11];  // whl
        desired_positions[13] = scaled_actions[3];
        desired_positions[14] = scaled_actions[15];
        desired_positions[15] = scaled_actions[7];
        // desired_positions.segment<3>(0) = default_dof_pos.segment<3>(8);  // FR leg
        // desired_positions.segment<3>(3) = default_dof_pos.segment<3>(0);  // FL leg
        // desired_positions.segment<3>(6) = default_dof_pos.segment<3>(12);  // RR leg
        // desired_positions.segment<3>(9) = default_dof_pos.segment<3>(4);  // RL leg
        // desired_positions[12] = 0.0;  // whl
        // desired_positions[13] = 0.0;
        // desired_positions[14] = 0.0;
        // desired_positions[15] = 0.0;
    }
    step_counter++;
    return true;
}

bool RLController2::stop() {
    return true;
}

bool RLController2::loadPolicy(const std::string& policy_path) {
    // 1. Initialize ONNX Runtime environment 
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "PolicyInference");
    session_options_ = std::make_unique<Ort::SessionOptions>();
    
    // (Optional) Enable CUDA if available
    // Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(*session_options_, 0));

    // 2. Load the ONNX model
    session_ = std::make_unique<Ort::Session>(*env_, policy_path.c_str(), *session_options_);
    return true;
}