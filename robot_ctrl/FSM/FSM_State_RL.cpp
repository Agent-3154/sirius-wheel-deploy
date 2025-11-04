#include "FSM_State_RL.h"
#include "./filters.h"
#include <cmath>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <iostream>
#include <filesystem>
#include "../mdp/observation.h"
#include "../mdp/observation.cpp"


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

    // Find the latest .onnx file in the ../models directory
    std::string policy_path;
    std::filesystem::file_time_type latest_time;
    bool found = false;
    for (const auto& entry : std::filesystem::directory_iterator("../models")) {
        if (entry.path().extension() == ".onnx") {
            auto ftime = std::filesystem::last_write_time(entry.path());
            if (!found || ftime > latest_time) {
                latest_time = ftime;
                policy_path = entry.path().string();
                found = true;
            }
        }
    }
    if (!found) {
        throw std::runtime_error("No .onnx policy file found in ../models");
    }
    std::cout << GREEN << "[FSM State RL]: Policy path: " << policy_path << RESET << std::endl;

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXInference");
    Ort::SessionOptions session_options;
    session = std::make_unique<Ort::Session>(env, policy_path.c_str(), session_options);

    // Initialize ONNX Runtime objects
    run_options = std::make_unique<Ort::RunOptions>(Ort::RunOptions(nullptr));
    memory_info = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
    
    this->is_init[0] = false; // Initialize the bool array
    this->hx.resize(HIDDEN_STATE_DIM, 0.0f);
    this->rpy.setZero();
    this->ref_rpy_.setZero();

    this->cmd_lin_vel_w_.setZero();
    this->cmd_lin_vel_b_.setZero();
    this->cmd_ang_vel_.setZero();
    this->ref_ang_vel_.setZero();

    this->prev_actions_.setZero();
    
    // prepare observations
    this->observations_.push_back(std::make_unique<ProjectedGravity>(1, 1));
    this->observations_.push_back(std::make_unique<JointPosMultistep>(HISTORY_STEPS, 2));
    // this->observations_.push_back(std::make_unique<JointVelMultistep>(2, 1));
    this->observations_.push_back(std::make_unique<PrevActions>());
    
    int policy_dim = 0;
    for (auto &obs : this->observations_) {
        int obs_size = obs->get_size();
        std::cout << GREEN << "[FSM State RL]: Observation size: " << obs_size << RESET << std::endl;
        policy_dim += obs_size;
    }

    this->obs_command_ = Eigen::VectorXf(COMMAND_DIM);
    this->obs_command_.setZero();
    this->obs_policy_ = Eigen::VectorXf(policy_dim);
    this->obs_policy_.setZero();
    this->cum_hip_deviation_.setZero();
    this->obs_policy_shape = {1, policy_dim};

    this->compute_observation();

    std::cout << GREEN << "[FSM State RL]: Policy Loaded" << RESET << std::endl;

    auto output_names_vector = session->GetOutputNames();
    for (const auto &name : output_names_vector)
    {
        std::cout << GREEN << name << RESET << std::endl;
    }

    this->run_inference(false);
}

bool FSM_State_RL::state_on_enter()
{
    std::cout << "[FSM State RL]: state_on_enter" << std::endl;
    
    this->quat = fsm_data_->estimators_->get_result_quat();
    this->gyro = fsm_data_->estimators_->get_result_angular_body();
    this->rpy = fsm_data_->estimators_->shared_esti_data_.result_->rpy_;

    this->is_jumping = false;
    this->cmd_time_ = 0.0;
    this->hx.resize(HIDDEN_STATE_DIM, 0.0f);
    this->ref_rpy_ << 0.0, 0.0, this->rpy(2);
    this->des_rpy_ << 0.0, 0.0, this->rpy(2);
    this->cmd_ang_vel_.setZero();

    Eigen::VectorXf desired_leg_jpos = this->DEFAULT_LEG_JOINT_POS;
    this->desired_leg_jpos_ = Eigen::Map<Eigen::Matrix<float, 4, 3>>(desired_leg_jpos.data());
    this->desired_leg_jpos_filtered_ = this->desired_leg_jpos_;
    
    // wait for the observation buffers to be filled
    this->obs_jpos_buffer_.setZero();
    this->obs_jvel_buffer_.setZero();
    this->cum_hip_deviation_.setZero();
    this->loop_step_count_ = 0;
    this->ctrl_step_count_ = 0;
    this->apply_action = false;
    return true;
};

void FSM_State_RL::state_on_exit()
{
    return;
};

void FSM_State_RL::step_command()
{
    float multiplier = 1.2;
    if ((fsm_data_->rc_->rc_map_.lt > 0) && (fsm_data_->rc_->rc_map_.rt > 0)) {
        multiplier = 2.2;
    }
    float v_des_x = fsm_data_->rc_->rc_control_.v_des[0] * multiplier;
    float v_des_y = fsm_data_->rc_->rc_control_.v_des[1] * 0.60;
    v_des_y = (abs(v_des_y) > 0.15) ? v_des_y : 0.0; // discard lateral velocity less than 0.1 m/s
    
    float v_des_yaw = fsm_data_->rc_->rc_control_.v_des[2] * M_PI / 2.0;
    
    Eigen::Vector3f v_des_xy = Eigen::Vector3f(v_des_x, v_des_y, 0.0);

    this->cmd_lin_vel_b_(0) = this->cmd_lin_vel_b_(0) + clamp_norm(0.2 * (v_des_x - this->cmd_lin_vel_b_(0)), 0.05);
    this->cmd_lin_vel_b_(1) = this->cmd_lin_vel_b_(1) + clamp_norm(0.2 * (v_des_y - this->cmd_lin_vel_b_(1)), 0.05);
    this->cmd_lin_vel_b_(2) = 0.0;

    if (this->is_jumping)
    {
        this->cmd_lin_vel_b_(1) = 0.0; // lateral velocity is zero
        this->cmd_mode_ << 0.0, 1.0;
        if (this->cmd_time_ < JUMP_PREP_TIME)
        {
            this->des_contact_  = Eigen::Vector4f::Ones() * 0.25;
            this->ref_vel_ = 0.0;
            this->ref_hei_ = 0.40;
            this->ref_ang_vel_(2) = 0.0;
        }
        else if (this->cmd_time_ < JUMP_PREP_TIME + JUMP_TAKEOFF_TIME)
        {
            double ref_acc = 0.1 + 30.0 * (this->cmd_time_ - JUMP_PREP_TIME);
            ref_acc = std::clamp(ref_acc, 0.0, 10.0);
            this->ref_vel_ = this->ref_vel_ + ref_acc * 0.02;
            this->ref_hei_ = this->ref_hei_ + this->ref_vel_ * 0.02;

            this->des_contact_ = Eigen::Vector4f::Zero();
            this->ref_ang_vel_(2) = this->jump_turn_ / this->jump_air_time_;
        }
        else if (this->cmd_time_ < JUMP_PREP_TIME + jump_air_time_)
        {
            double ref_acc = -9.81;
            if (this->ref_hei_ < 0.5) {
                ref_acc = ref_acc * 0.2 + 100.0 * (0.5 - this->ref_hei_) - 20.0 * this->ref_vel_;
            }
            this->ref_vel_ = this->ref_vel_ + ref_acc * 0.02;
            this->ref_hei_ = this->ref_hei_ + this->ref_vel_ * 0.02;

            this->des_contact_ = -Eigen::Vector4f::Ones();
            this->ref_ang_vel_(2) = this->jump_turn_ / this->jump_air_time_;
        }
        else if (this->cmd_time_ < this->cmd_duration_) {
            this->des_contact_ = Eigen::Vector4f::Zero();
            this->ref_ang_vel_(2) = 0.0;
        } else {
            this->is_jumping = false;
            this->cmd_time_ = 0.0;
            this->ref_rpy_(2) = this->rpy(2);
        }
        this->cmd_lin_vel_w_(2) = this->ref_vel_;
        this->cmd_ang_vel_(2) = this->ref_ang_vel_(2);
    }
    else
    {
        this->cmd_mode_ << 1.0, 0.0;
        for (int i = 0; i < 4; i++) {
            auto cond = (this->cum_hip_deviation_(i) > 0.6);
            this->des_contact_(i) = cond ? -1.0 : 0.0;
        }
        this->ref_ang_vel_(2) = v_des_yaw;
        this->cmd_ang_vel_(2) = this->ref_ang_vel_(2);
    }
    this->ref_rpy_ += this->ref_ang_vel_ * 0.02;
    this->ref_rpy_(2) = std::fmod(this->ref_rpy_(2), 2 * M_PI);
    this->cmd_time_ += 0.02;
}

void FSM_State_RL::compute_command() {
    Eigen::Vector3f cmd_lin_vel;
    Eigen::Vector3f ref_rpy_b = Eigen::Vector3f::Zero();
    
    Eigen::Vector2f timing;
    if (this->is_jumping) {
        Eigen::Quaternionf quat_eigen(quat(0), quat(1), quat(2), quat(3));
        cmd_lin_vel = yaw_quat(quat_eigen).inverse() * this->cmd_lin_vel_w_;
        timing << this->cmd_time_, this->cmd_duration_ - this->cmd_time_;
    } else {
        cmd_lin_vel = this->cmd_lin_vel_b_;
        timing << 0.0, 0.0;
    }

    ref_rpy_b(2) = wrap_to_pi(this->ref_rpy_(2) - this->rpy(2));

    this->obs_command_ << 
        cmd_lin_vel, // 3
        this->cmd_ang_vel_, // 3
        ref_rpy_b, // 3
        timing, // 2
        this->cmd_mode_,
        abs(cmd_lin_vel(1)) < 0.1,
        abs(this->cmd_ang_vel_(2)) < 0.1; // 2
        // this->des_contact_; // 4
    // std::cout << this->rpy(2) << " " << this->ref_rpy_(2) << " " << ref_rpy_b(2) << std::endl;
}

void FSM_State_RL::compute_observation() {
    Eigen::Quaterniond quat_eigen(quat[0], quat[1], quat[2], quat[3]);
    this->projected_gravity_ = (quat_eigen.inverse() * Eigen::Vector3d(0, 0, -1));

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

void FSM_State_RL::run_inference(bool apply_action) {
    std::vector<Ort::Value> input_tensors;
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        *this->memory_info,
        this->obs_command_.data(),
        this->obs_command_.size(),
        this->obs_command_shape.data(),
        this->obs_command_shape.size()));
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        *this->memory_info,
        this->obs_policy_.data(),
        this->obs_policy_.size(),
        this->obs_policy_shape.data(),
        this->obs_policy_shape.size()));
    input_tensors.push_back(Ort::Value::CreateTensor<bool>(
        *this->memory_info,
        this->is_init,
        1,
        this->is_init_shape.data(),
        this->is_init_shape.size()));
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        *this->memory_info,
        hx.data(),
        this->hx.size(),
        this->hx_shape.data(),
        this->hx_shape.size()));

    const char *input_names[] = {"command", "policy", "is_init", "hx"};
    const char *output_names[] = {"div", "div_1", "linear_8", "sum_1", "add_3", "mish_4"};
    auto output_tensors = session->Run(
        *run_options,
        input_names,
        input_tensors.data(),
        session->GetInputCount(),
        output_names,
        session->GetOutputCount());

    // // get action and convert to Eigen
    auto action = output_tensors[2].GetTensorMutableData<float>();
    Eigen::Map<const Eigen::VectorXf> action_eigen(action, ACTION_DIM);
    auto next_hx = output_tensors[4].GetTensorMutableData<float>();

    if (apply_action) {
        this->prev_actions_.col(1) = this->prev_actions_.col(0);
        this->prev_actions_.col(0) = action_eigen;
    
        Eigen::VectorXf desired_leg_jpos = action_eigen.head(12) * LEG_ACTION_SCALE + this->DEFAULT_LEG_JOINT_POS;
    
        this->desired_leg_jpos_ = Eigen::Map<Eigen::Matrix<float, 4, 3>>(desired_leg_jpos.data());
        this->desired_whl_jvel_ = action_eigen.tail(4) * 10.0;
    
        // get next_hx and copy to hx
        std::copy(next_hx, next_hx + HIDDEN_STATE_DIM, this->hx.begin());
    }
}

void FSM_State_RL::run_state()
{
    this->loop_step_count_++;

    this->quat = fsm_data_->estimators_->get_result_quat();
    this->gyro = fsm_data_->estimators_->get_result_angular_body();
    this->rpy = fsm_data_->estimators_->shared_esti_data_.result_->rpy_;
    // std::cout << "rpy: " << rpy.transpose() << std::endl;

    if (fsm_data_->rc_->rc_map_.b && !fsm_data_->rc_->rc_map_.lb && !this->is_jumping)
    {
        this->is_jumping = true;
        this->jump_turn_ = M_PI;
        this->jump_air_time_ = 1.0;
        this->cmd_time_ = 0.0;
        this->cmd_duration_ = JUMP_PREP_TIME + this->jump_air_time_ + JUMP_LAND_TIME;

        Eigen::Quaternionf quat_eigen(quat(0), quat(1), quat(2), quat(3));
        this->cmd_lin_vel_w_ = (yaw_quat(quat_eigen) * this->cmd_lin_vel_b_);
        this->ref_rpy_ << 0.0, 0.0, this->rpy(2);
        this->des_rpy_ << 0.0, 0.0, this->rpy(2) + this->jump_turn_;
    }

    Eigen::Matrix<float, 4, 3> jpos_leg; // joint position in ISAAC order
    Eigen::Matrix<float, 4, 3> jvel_leg; // joint velocity in ISAAC order

    jpos_leg.row(0) = fsm_data_->leg_controller_->leg_data[1].q.cast<float>();   // RF
    jpos_leg.row(1) = fsm_data_->leg_controller_->leg_data[3].q.cast<float>();   // LF
    jpos_leg.row(2) = fsm_data_->leg_controller_->leg_data[0].q.cast<float>();   // RH
    jpos_leg.row(3) = fsm_data_->leg_controller_->leg_data[2].q.cast<float>();   // LH
    jvel_leg.row(0) = fsm_data_->leg_controller_->leg_data[1].qd.cast<float>(); // RF
    jvel_leg.row(1) = fsm_data_->leg_controller_->leg_data[3].qd.cast<float>(); // LF
    jvel_leg.row(2) = fsm_data_->leg_controller_->leg_data[0].qd.cast<float>(); // RH
    jvel_leg.row(3) = fsm_data_->leg_controller_->leg_data[2].qd.cast<float>(); // LH

    auto jpos_leg_flat = Eigen::Map<Eigen::VectorXf>(jpos_leg.data(), 12);
    auto jvel_leg_flat = Eigen::VectorXf(16);
    jvel_leg_flat << Eigen::Map<Eigen::VectorXf>(jvel_leg.data(), 12),
        float(fsm_data_->leg_controller_->leg_data[1].whl_qd),
        float(fsm_data_->leg_controller_->leg_data[3].whl_qd),
        float(fsm_data_->leg_controller_->leg_data[0].whl_qd),
        float(fsm_data_->leg_controller_->leg_data[2].whl_qd);

    raw_jpos_buffer_.col(loop_step_count_ % 10) = jpos_leg_flat;
    raw_jvel_buffer_.col(loop_step_count_ % 10) = jvel_leg_flat;

    Eigen::VectorXf jpos_leg_filtered = raw_jpos_buffer_.rowwise().mean().eval();
    Eigen::VectorXf jvel_leg_filtered = raw_jvel_buffer_.rowwise().mean().eval();

    if ((loop_step_count_+1) % 10 == 0)
    {
        this->ctrl_step_count_++;
        if (this->ctrl_step_count_ > HISTORY_STEPS) {
            this->apply_action = true;
        }

        // auto jpos_leg = raw_jpos_buffer_.rowwise().mean();
        auto jpos_leg = jpos_leg_flat;
        
        // shift history
        for (int i = HISTORY_STEPS - 1; i > 0; i--)
        {
            this->obs_jpos_buffer_.col(i) = this->obs_jpos_buffer_.col(i - 1);
            this->obs_jvel_buffer_.col(i) = this->obs_jvel_buffer_.col(i - 1);
        }

        this->obs_jpos_buffer_.col(0) = jpos_leg;
        this->obs_jvel_buffer_.col(0) << 
            float(fsm_data_->leg_controller_->leg_data[1].whl_qd),
            float(fsm_data_->leg_controller_->leg_data[3].whl_qd),
            float(fsm_data_->leg_controller_->leg_data[0].whl_qd),
            float(fsm_data_->leg_controller_->leg_data[2].whl_qd);

        this->step_command();
        this->compute_command();
        this->compute_observation();
        this->run_inference(true);
        
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

        fsm_data_->leg_controller_->leg_command[0].whl_qd_des = double(this->desired_whl_jvel_(2));
        fsm_data_->leg_controller_->leg_command[1].whl_qd_des = double(this->desired_whl_jvel_(0));
        fsm_data_->leg_controller_->leg_command[2].whl_qd_des = double(this->desired_whl_jvel_(3));
        fsm_data_->leg_controller_->leg_command[3].whl_qd_des = double(this->desired_whl_jvel_(1));
    
        for (auto &leg : fsm_data_->leg_controller_->leg_command)
        {
            leg.kp_joint = Vec3<double>(LEG_KP, LEG_KP, LEG_KP).asDiagonal();
            leg.kd_joint = Vec3<double>(LEG_KD, LEG_KD, LEG_KD).asDiagonal();
            leg.whl_kp_joint = 0;
            leg.whl_kd_joint = WHEEL_KD;
        }
    }
};

bool FSM_State_RL::is_busy()
{
    // wait until the jump is finished
    return this->is_jumping;
};
