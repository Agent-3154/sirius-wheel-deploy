#include "FSM_State_RL.h"
#include <cmath>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <iostream>
#include "../mdp/observation.h"
#include "../mdp/observation.cpp"
#include "./filters.h"
#include <yaml-cpp/yaml.h>


Eigen::Quaternionf yaw_quat(Eigen::Quaternionf quat) {
    auto qw = quat.w();
    auto qx = quat.x();
    auto qy = quat.y();
    auto qz = quat.z();
    auto yaw = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
    Eigen::Quaternionf quat_yaw(std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2));
    quat_yaw.normalize();
    return quat_yaw;
}

float wrap_to_pi(float angle) {
    auto wrapped_angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle + M_PI < 0) {
        wrapped_angle = wrapped_angle + 2 * M_PI;
    }
    return wrapped_angle - M_PI;
}

float clamp_norm(float x, float max_norm) {
    return std::clamp(x, -max_norm, max_norm);
}


ONNXPolicy::ONNXPolicy(const std::string &model_path) {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXInference");
    Ort::SessionOptions session_options;
    session_ = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);
    memory_info_ = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
    run_options_ = std::make_unique<Ort::RunOptions>(Ort::RunOptions(nullptr));

    auto input_names_vector = session_->GetInputNames();
    for (const auto &name : input_names_vector)
    {
        std::cout << GREEN << name << RESET << std::endl;
    }

    auto output_names_vector = session_->GetOutputNames();
    for (const auto &name : output_names_vector)
    {
        std::cout << GREEN << name << RESET << std::endl;
    }
}

void ONNXPolicy::runInference(std::vector<Ort::Value> &input_tensors) {
    // auto output_tensors = session_->Run(
    //     *run_options_,
    //     input_names,
    //     input_tensors.data(),
    //     input_tensors.size(),
    //     output_names,
    //     output_tensors.size());
}


FSM_State_RL::FSM_State_RL(
    Control_FSM_Data_t *controlfsmdata,
    Control_Parameters_t *control_para) : FSM_State(controlfsmdata, control_para, RL),
                                          jvel_filter_1(0.01f, 0.001f),
                                          jvel_filter_2(0.01f, 0.002f)
{
    std::cout << GREEN << "[FSM State RL]: Ort version: " << ORT_API_VERSION << RESET << std::endl;
}

void FSM_State_RL::load_policy(const std::string &policy_path) {
    std::string config_path = policy_path;
    size_t pos = config_path.rfind(".onnx");
    if (pos != std::string::npos) {
        config_path.replace(pos, 5, ".yaml");
    }
    std::cout << GREEN << "[FSM State RL]: Ort version: " << ORT_API_VERSION << RESET << std::endl;
    std::cout << GREEN << "[FSM State RL]: Policy path: " << policy_path << RESET << std::endl;
    std::cout << GREEN << "[FSM State RL]: Config path: " << config_path << RESET << std::endl;

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXInference");
    Ort::SessionOptions session_options;
    // Initialize ONNX Runtime objects
    this->session_ = std::make_unique<Ort::Session>(env, policy_path.c_str(), session_options);
    this->run_options_ = std::make_unique<Ort::RunOptions>(Ort::RunOptions(nullptr));
    this->memory_info_ = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    // load yaml config
    YAML::Node config = YAML::LoadFile(config_path);
    this->default_leg_jpos_ = Eigen::Map<Eigen::VectorXf>(config["default_joint_pos"].as<std::vector<float>>().data(), 12);
    this->joint_stiffness_ = Eigen::Map<Eigen::VectorXf>(config["stiffness"].as<std::vector<float>>().data(), 12);
    this->joint_damping_ = Eigen::Map<Eigen::VectorXf>(config["damping"].as<std::vector<float>>().data(), 12);
    
    // Print out the loaded configs
    std::cout << GREEN << "[FSM State RL]: default_leg_jpos_: " << this->default_leg_jpos_.transpose() << RESET << std::endl;
    std::cout << GREEN << "[FSM State RL]: joint_stiffness_: " << this->joint_stiffness_.transpose() << RESET << std::endl;
    std::cout << GREEN << "[FSM State RL]: joint_damping_: " << this->joint_damping_.transpose() << RESET << std::endl;

    this->is_init[0] = false; // Initialize the bool array
    this->hx.resize(HIDDEN_STATE_DIM, 0.0f);
    this->rpy_.setZero();

    this->cmd_lin_vel_w_.setZero();
    this->cmd_lin_vel_b_.setZero();
    this->cmd_ang_vel_.setZero();
    this->prev_actions_.setZero();
    
    // prepare observations
    this->observations_.push_back(std::make_unique<ProjectedGravity>(1, 1));
    this->observations_.push_back(std::make_unique<JointPosMultistep>(4, 2));
    this->observations_.push_back(std::make_unique<JointVelMultistep>(4, 2));
    this->observations_.push_back(std::make_unique<PrevActions>(3));
    
    int policy_dim = 0;
    for (auto &obs : this->observations_) {
        int obs_size = obs->get_size();
        std::cout << GREEN << "[FSM State RL]: Observation size: " << obs_size << RESET << std::endl;
        policy_dim += obs_size;
    }

    this->obs_command_ = Eigen::VectorXf(4);
    this->obs_command_.setZero();
    this->obs_command_shape = {1, 4};
    this->obs_policy_ = Eigen::VectorXf(policy_dim);
    this->obs_policy_.setZero();
    this->obs_policy_shape = {1, policy_dim};
    std::cout << GREEN << "[FSM State RL]: Policy Loaded" << RESET << std::endl;

    this->input_names_ = this->session_->GetInputNames();
    this->output_names_ = this->session_->GetOutputNames();
    
    // Convert to C-strings once for ONNX Runtime API
    this->input_names_cstr_.clear();
    this->output_names_cstr_.clear();
    for (const auto& name : this->input_names_) {
        this->input_names_cstr_.push_back(name.c_str());
    }
    for (const auto& name : this->output_names_) {
        this->output_names_cstr_.push_back(name.c_str());
    }
    
    // Print input and output names in a single line each
    std::cout << GREEN << "[FSM State RL]: ONNX Model Inputs: " << RESET;
    for (size_t i = 0; i < this->input_names_.size(); ++i) {
        std::cout << (i > 0 ? ", " : "") << this->input_names_[i];
    }
    std::cout << std::endl;

    std::cout << GREEN << "[FSM State RL]: ONNX Model Outputs: " << RESET;
    for (size_t i = 0; i < this->output_names_.size(); ++i) {
        std::cout << (i > 0 ? ", " : "") << this->output_names_[i];
    }
    std::cout << std::endl;
    
    this->computeObservation();
    this->runInference(false);
}

bool FSM_State_RL::state_on_enter()
{
    std::cout << "[FSM State RL]: state_on_enter" << std::endl;
    
    this->quat_ = fsm_data_->estimators_->get_result_quat();
    this->gyro_ = fsm_data_->estimators_->get_result_angular_body();
    this->rpy_ = fsm_data_->estimators_->shared_esti_data_.result_->rpy_;

    this->hx.resize(HIDDEN_STATE_DIM, 0.0f);
    this->cmd_ang_vel_.setZero();

    Eigen::VectorXf desired_leg_jpos = this->default_leg_jpos_;
    this->desired_leg_jpos_ = Eigen::Map<Eigen::Matrix<float, 4, 3>>(desired_leg_jpos.data());
    this->desired_leg_jpos_filtered_ = this->desired_leg_jpos_;
    
    // wait for the observation buffers to be filled
    this->loop_step_count_ = 0;
    this->ctrl_step_count_ = 0;
    this->apply_action = false;
    return true;
};

void FSM_State_RL::state_on_exit()
{
    return;
};

void FSM_State_RL::computeObservation() {
    for (auto &obs : this->observations_) {
        obs->update(this);
    }
    this->obs_policy_.setZero();
    int current_idx = 0;
    for (auto &obs : this->observations_) {
        Eigen::VectorXf obs_result = obs->compute();
        int obs_size = obs_result.size();
        this->obs_policy_.segment(current_idx, obs_size) = obs_result;
        current_idx += obs_size;
    }
}

void FSM_State_RL::runInference(bool apply_action) {
    std::vector<Ort::Value> input_tensors;
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        *this->memory_info_,
        this->obs_command_.data(),
        this->obs_command_.size(),
        this->obs_command_shape.data(),
        this->obs_command_shape.size()));
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        *this->memory_info_,
        this->obs_policy_.data(),
        this->obs_policy_.size(),
        this->obs_policy_shape.data(),
        this->obs_policy_shape.size()));
    input_tensors.push_back(Ort::Value::CreateTensor<bool>(
        *this->memory_info_,
        this->is_init,
        1,
        this->is_init_shape.data(),
        this->is_init_shape.size()));
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        *this->memory_info_,
        hx.data(),
        this->hx.size(),
        this->hx_shape.data(),
        this->hx_shape.size()));

    auto output_tensors = this->session_->Run(
        *this->run_options_,
        this->input_names_cstr_.data(),
        input_tensors.data(),
        input_tensors.size(),
        this->output_names_cstr_.data(),
        this->output_names_cstr_.size());

    // // get action and convert to Eigen
    auto action = output_tensors[2].GetTensorMutableData<float>();
    Eigen::Map<const Eigen::VectorXf> action_eigen(action, ACTION_DIM);
    auto next_hx = output_tensors[4].GetTensorMutableData<float>();

    if (apply_action) {
        for (int i = 3; i > 0; i--) {
            this->prev_actions_.col(i) = this->prev_actions_.col(i - 1);
        }
        this->prev_actions_.col(0) = action_eigen;
    
        Eigen::VectorXf desired_leg_jpos = action_eigen.head(12) * LEG_ACTION_SCALE + this->default_leg_jpos_;
    
        this->desired_leg_jpos_ = Eigen::Map<Eigen::Matrix<float, 4, 3>>(desired_leg_jpos.data());
        // this->desired_whl_jvel_ = action_eigen.tail(4) * 10.0;
    
        // get next_hx and copy to hx
        std::copy(next_hx, next_hx + HIDDEN_STATE_DIM, this->hx.begin());
    }
}

void FSM_State_RL::run_state()
{
    this->loop_step_count_++;

    this->quat_ = fsm_data_->estimators_->get_result_quat();
    this->gyro_ = fsm_data_->estimators_->get_result_angular_body();
    this->rpy_ = fsm_data_->estimators_->shared_esti_data_.result_->rpy_;

    Eigen::Matrix<float, 4, 3> jpos_leg; // joint position in ISAAC order
    Eigen::Matrix<float, 4, 3> jvel_leg; // joint velocity in ISAAC order

    jpos_leg.row(0) = fsm_data_->leg_controller_->leg_data[1].q.cast<float>();  // LF
    jpos_leg.row(1) = fsm_data_->leg_controller_->leg_data[3].q.cast<float>();  // LH
    jpos_leg.row(2) = fsm_data_->leg_controller_->leg_data[0].q.cast<float>();  // RF
    jpos_leg.row(3) = fsm_data_->leg_controller_->leg_data[2].q.cast<float>();  // RG

    jvel_leg.row(0) = fsm_data_->leg_controller_->leg_data[1].qd.cast<float>(); // LF
    jvel_leg.row(1) = fsm_data_->leg_controller_->leg_data[3].qd.cast<float>(); // LH
    jvel_leg.row(2) = fsm_data_->leg_controller_->leg_data[0].qd.cast<float>(); // RF
    jvel_leg.row(3) = fsm_data_->leg_controller_->leg_data[2].qd.cast<float>(); // RG

    auto jpos_leg_flat = Eigen::Map<Eigen::VectorXf>(jpos_leg.data(), 12);
    auto jvel_leg_flat = Eigen::VectorXf(16);
    jvel_leg_flat << Eigen::Map<Eigen::VectorXf>(jvel_leg.data(), 12),
        float(fsm_data_->leg_controller_->leg_data[1].whl_qd),
        float(fsm_data_->leg_controller_->leg_data[3].whl_qd),
        float(fsm_data_->leg_controller_->leg_data[0].whl_qd),
        float(fsm_data_->leg_controller_->leg_data[2].whl_qd);

    raw_jpos_buffer_.col(loop_step_count_ % 10) = jpos_leg_flat;
    raw_jvel_buffer_.col(loop_step_count_ % 10) = jvel_leg_flat;

    if ((loop_step_count_+1) % 10 == 0)
    {
        this->ctrl_step_count_++;
        if (this->ctrl_step_count_ > 4) {
            this->apply_action = true;
        }
        this->computeObservation();
        this->runInference(true);
    }
    // only update if desired_leg_jpos does not contain nan
    if (!this->desired_leg_jpos_.hasNaN())
    {
        this->desired_leg_jpos_filtered_ = 0.8 * this->desired_leg_jpos_ + 0.2 * this->desired_leg_jpos_filtered_;
    }

    if (this->apply_action)
    {
        fsm_data_->leg_controller_->leg_command[0].q_des = desired_leg_jpos_filtered_.row(2).cast<double>();
        fsm_data_->leg_controller_->leg_command[1].q_des = desired_leg_jpos_filtered_.row(0).cast<double>();
        fsm_data_->leg_controller_->leg_command[2].q_des = desired_leg_jpos_filtered_.row(3).cast<double>();
        fsm_data_->leg_controller_->leg_command[3].q_des = desired_leg_jpos_filtered_.row(1).cast<double>();

        // fsm_data_->leg_controller_->leg_command[0].whl_qd_des = double(this->desired_whl_jvel_(2));
        // fsm_data_->leg_controller_->leg_command[1].whl_qd_des = double(this->desired_whl_jvel_(0));
        // fsm_data_->leg_controller_->leg_command[2].whl_qd_des = double(this->desired_whl_jvel_(3));
        // fsm_data_->leg_controller_->leg_command[3].whl_qd_des = double(this->desired_whl_jvel_(1));
    
        for (auto &leg : fsm_data_->leg_controller_->leg_command)
        {
            leg.kp_joint = Vec3<double>(LEG_KP, LEG_KP, LEG_KP).asDiagonal();
            leg.kd_joint = Vec3<double>(LEG_KD, LEG_KD, LEG_KD).asDiagonal();
            // leg.whl_kp_joint = 0;
            // leg.whl_kd_joint = WHEEL_KD;
        }
    }
};

bool FSM_State_RL::is_busy()
{
    return false;
};
