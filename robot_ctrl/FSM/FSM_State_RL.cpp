#include "FSM_State_RL.h"
#include "./filters.h"
#include <iostream>
#include <iomanip>
// #include <onnxruntime_cxx_api.h>
// #include <eigen3/Eigen/Dense>
// #include "../../utilities/types/std_cout_colors.h"

FSM_State_RL::FSM_State_RL(
    Control_FSM_Data_t *controlfsmdata,
    Control_Parameters_t *control_para) : FSM_State(controlfsmdata, control_para, RL),
                                          lcm_logger_("udpm://239.255.76.67:7667?ttl=255"),
                                          jvel_filter_1(0.01f, 0.001f),
                                          jvel_filter_2(0.01f, 0.002f)
{
    std::cout << GREEN << "[FSM State RL]: Ort version: " << ORT_API_VERSION << RESET << std::endl;

    const std::string policy_path = "../models/policy-08-27_15-42.onnx";
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXInference");
    Ort::SessionOptions session_options;
    session = std::make_unique<Ort::Session>(env, policy_path.c_str(), session_options);

    // Initialize ONNX Runtime objects
    run_options = std::make_unique<Ort::RunOptions>(Ort::RunOptions(nullptr));
    memory_info = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    // Initialize observation vector with 49 zeros
    policy.resize(POLICY_DIM, 0.0f);
    
    this->is_init[0] = false; // Initialize the bool array
    this->hx.resize(HIDDEN_STATE_DIM, 0.0f);
    this->rpy.setZero();
    this->cmd_rpy_.setZero();
    // rpy_init = fsm_data_->estimators_->shared_esti_data_.result_->rpy_;
    // std::cout << "rpy_init: " << rpy_init.transpose() << std::endl;

    this->cmd_lin_vel_.setZero();
    this->cmd_ang_vel_.setZero();

    this->prev_actions.setZero();
    this->command_ = Eigen::VectorXf(COMMAND_DIM);
    this->command_.setZero();
    this->cum_hip_deviation_.setZero();

    std::cout << GREEN << "[FSM State RL]: Policy Loaded" << RESET << std::endl;

    auto output_names_vector = session->GetOutputNames();
    for (const auto &name : output_names_vector)
    {
        std::cout << GREEN << name << RESET << std::endl;
    }
}

bool FSM_State_RL::state_on_enter()
{
    std::cout << "[FSM State RL]: state_on_enter" << std::endl;
    for (auto &leg : fsm_data_->leg_controller_->leg_command)
    {
        leg.kp_joint = Vec3<double>(LEG_KP, LEG_KP, LEG_KP).asDiagonal();
        leg.kd_joint = Vec3<double>(LEG_KD, LEG_KD, LEG_KD).asDiagonal();
        leg.whl_kp_joint = 0;
        leg.whl_kd_joint = WHEEL_KD;
    }
    this->is_jumping = false;
    this->cmd_time_ = 0.0;
    this->hx.resize(HIDDEN_STATE_DIM, 0.0f);
    this->cmd_ang_vel_.setZero();

    Eigen::VectorXf desired_leg_jpos = this->DEFAULT_LEG_JOINT_POS;
    this->desired_leg_jpos_ = Eigen::Map<Eigen::Matrix<float, 4, 3>>(desired_leg_jpos.data());
    this->desired_leg_jpos_filtered_ = this->desired_leg_jpos_;
    return true;
};

void FSM_State_RL::state_on_exit()
{
    return;
};

void FSM_State_RL::step_command()
{
    float v_des_x = fsm_data_->rc_->rc_control_.v_des[0] * 1.2;
    float v_des_y = fsm_data_->rc_->rc_control_.v_des[1] * 0.8;
    float v_des_z = fsm_data_->rc_->rc_control_.v_des[2];

    Eigen::Vector3f v_des_xy = Eigen::Vector3f(v_des_x, v_des_y, 0.0);
    this->cmd_lin_vel_ = this->cmd_lin_vel_ * 0.5 + v_des_xy * 0.5;

    if (this->is_jumping)
    {
        this->cmd_mode_ << 0.0, 1.0;
        if (this->cmd_time_ < JUMP_PREP_TIME)
        {
            this->des_contact_  = - Eigen::Vector4f::Ones() * 0.25;
        }
        else if (this->cmd_time_ < JUMP_PREP_TIME + 0.2)
        {
            this->des_contact_ << 0.0, 0.0, 0.0, 0.0;
            this->cmd_ang_vel_ << 0.0, 0.0, 0.0;
        }
        else if (this->cmd_time_ < this->cmd_duration_ - JUMP_LAND_TIME)
        {
            this->des_contact_ << Eigen::Vector4f::Ones();
        } else if (this->cmd_time_ < this->cmd_duration_) {
            
        } else {
            this->is_jumping = false;
            this->cmd_time_ = 0.0;
            this->cmd_rpy_(2) = this->rpy(2);
        }
    }
    else
    {
        this->cmd_mode_ << 1.0, 0.0;
        for (int i = 0; i < 4; i++) {
            auto cond = (this->cum_hip_deviation_(i) > 0.6);
            this->des_contact_(i) = cond ? -1.0 : 0.0;
        }
        this->cmd_ang_vel_ << 0.0, 0.0, v_des_z;
    }
    this->cmd_rpy_ += this->cmd_ang_vel_ * 0.02;
    this->cmd_time_ += 0.02;
}

void FSM_State_RL::compute_command() {
    Eigen::Vector3f cmd_rpy_b;
    cmd_rpy_b(2) = this->cmd_rpy_(2) - this->rpy(2);
    cmd_rpy_b(2) = std::fmod(cmd_rpy_b(2) + M_PI, 2 * M_PI) - M_PI;
    auto des_yaw_b = 0.0;

    Eigen::Vector2f timing;
    if (this->is_jumping) {
        timing << this->cmd_time_, this->cmd_duration_ - this->cmd_time_;
    } else {
        timing << 0.0, 0.0;
    }
    this->command_ << 
        this->cmd_lin_vel_,
        this->cmd_ang_vel_,
        cmd_rpy_b,
        des_yaw_b,
        timing,
        this->cmd_mode_,
        this->des_contact_;
}

void FSM_State_RL::run_state()
{
    step_count++;

    this->quat = fsm_data_->estimators_->get_result_quat();
    this->gyro = fsm_data_->estimators_->get_result_angular_body();
    this->rpy = fsm_data_->estimators_->shared_esti_data_.result_->rpy_;
    // std::cout << "rpy: " << rpy.transpose() << std::endl;

    if (fsm_data_->rc_->rc_map_.a && !this->is_jumping)
    {
        this->is_jumping = true;
        float angle = 0.0;
        float air_time = 0.5;
        this->cmd_duration_ = JUMP_PREP_TIME + air_time + JUMP_LAND_TIME;

        // if (fsm_data_->rc_->rc_map_.lt > 0)
        // {
        //     angle = M_PI / 2;
        //     std::cout << "[FSM State RL]: jump left" << std::endl;
        // }
        // else if (fsm_data_->rc_->rc_map_.rt > 0)
        // {
        //     angle = -M_PI / 2;
        //     std::cout << "[FSM State RL]: jump right" << std::endl;
        // }
        this->cmd_rpy_ << 0.0, 0.0, this->rpy(2);
        this->des_rpy_ << 0.0, 0.0, this->rpy(2) + angle;
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

    raw_jpos_buffer_.col(step_count % 10) = jpos_leg_flat;
    // raw_jvel_buffer_.col(step_count % 10) = jvel_leg_flat;

    std::memcpy(lcm_leg_raw_data.q, jpos_leg_flat.data(), 12 * sizeof(float));
    std::memcpy(lcm_leg_raw_data.qd, jvel_leg_flat.data(), 16 * sizeof(float));
    lcm_logger_.publish("RAW_DATA_CHANNEL", &lcm_leg_raw_data);
    
    auto jvel_leg_filtered_1 = this->jvel_filter_1.update(jvel_leg_flat);
    std::memcpy(lcm_leg_filtered_data_1.qd, jvel_leg_filtered_1.data(), 16 * sizeof(float));
    lcm_logger_.publish("FILTERED_DATA_CHANNEL_1", &lcm_leg_filtered_data_1);

    auto jvel_leg_filtered_2 = this->jvel_filter_2.update(jvel_leg_filtered_1);
    std::memcpy(lcm_leg_filtered_data_2.qd, jvel_leg_filtered_2.data(), 16 * sizeof(float));
    lcm_logger_.publish("FILTERED_DATA_CHANNEL_2", &lcm_leg_filtered_data_2);

    if ((step_count+1) % 10 == 0)
    {
        // shift history
        for (int i = HISTORY_STEPS - 1; i > 0; i--)
        {
            this->obs_jpos_buffer_.col(i) = this->obs_jpos_buffer_.col(i - 1);
            this->obs_jvel_buffer_.col(i) = this->obs_jvel_buffer_.col(i - 1);
        }

        // auto jpos_leg_mean_prev = raw_jpos_buffer_.leftCols(5).rowwise().mean();
        auto jpos_leg_mean_curr = raw_jpos_buffer_.rowwise().mean();
        auto jvel_leg_approx = (jpos_leg_mean_curr - this->obs_jpos_buffer_.col(1)) / 0.02;

        for (int i = 0; i < 4; i++)
        {
            auto hip_deviation = abs(jpos_leg_mean_curr(i));
            if (hip_deviation < 0.1)
            {
                this->cum_hip_deviation_(i) = 0.0;
            }
            else
            {
                this->cum_hip_deviation_(i) += hip_deviation * 0.02;
            }
        }

        this->obs_jpos_buffer_.col(0) = jpos_leg_mean_curr;
        this->obs_jvel_buffer_.col(0) << jvel_leg_approx,
            float(fsm_data_->leg_controller_->leg_data[1].whl_qd),
            float(fsm_data_->leg_controller_->leg_data[3].whl_qd),
            float(fsm_data_->leg_controller_->leg_data[0].whl_qd),
            float(fsm_data_->leg_controller_->leg_data[2].whl_qd);

        Eigen::Quaterniond quat_eigen(quat[0], quat[1], quat[2], quat[3]);
        this->projected_gravity = (quat_eigen.inverse() * Eigen::Vector3d(0, 0, -1));

        // observation "policy":
        // concat [projected_gravity(3), q_buffer.flatten(48), qd_buffer.flatten(64), prev_actions.flatten(32)] = 147 total
        Eigen::VectorXf _policy(POLICY_DIM);
        _policy.setZero();
        
        auto whl_qd = obs_jvel_buffer_.bottomRows(4);
        _policy << projected_gravity.cast<float>(),
            Eigen::Map<Eigen::VectorXf>(obs_jpos_buffer_.data(), 48),
            Eigen::Map<Eigen::VectorXf>(whl_qd.data(), 16),
            Eigen::Map<Eigen::VectorXf>(prev_actions.data(), 32),
            this->cum_hip_deviation_;

        // Copy to policy vector
        std::copy(_policy.data(), _policy.data() + POLICY_DIM, this->policy.begin());

        step_command();
        compute_command();

        // std::cout << "command: " << std::fixed << std::setprecision(2) << cmd_rpy_.transpose() << std::endl;

        std::vector<Ort::Value> input_tensors;
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            *this->memory_info,
            this->command_.data(),
            this->command_.size(),
            this->command_shape.data(),
            this->command_shape.size()));
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            *this->memory_info,
            policy.data(),
            policy.size(),
            policy_shape.data(),
            policy_shape.size()));
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

        this->prev_actions.col(1) = this->prev_actions.col(0);
        this->prev_actions.col(0) = action_eigen;

        Eigen::VectorXf desired_leg_jpos = action_eigen.head(12) * 0.75 + this->DEFAULT_LEG_JOINT_POS;

        this->desired_leg_jpos_ = Eigen::Map<Eigen::Matrix<float, 4, 3>>(desired_leg_jpos.data());
        this->desired_whl_jvel_ = action_eigen.tail(4) * 10.0;

        // get next_hx and copy to hx
        auto next_hx = output_tensors[4].GetTensorMutableData<float>();
        std::copy(next_hx, next_hx + HIDDEN_STATE_DIM, this->hx.begin());

        std::memcpy(lcm_leg_obs_data.q, obs_jpos_buffer_.col(0).data(), 12 * sizeof(float));
        std::memcpy(lcm_leg_obs_data.qd, obs_jvel_buffer_.col(0).data(), 16 * sizeof(float));
        lcm_logger_.publish("POLICY_DATA_CHANNEL", &lcm_leg_obs_data);
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
    }

    for (auto &leg : fsm_data_->leg_controller_->leg_command)
    {
        leg.kp_joint = Vec3<double>(LEG_KP, LEG_KP, LEG_KP).asDiagonal();
        leg.kd_joint = Vec3<double>(LEG_KD, LEG_KD, LEG_KD).asDiagonal();
        leg.whl_kp_joint = 0;
        leg.whl_kd_joint = WHEEL_KD;
    }
};

bool FSM_State_RL::is_busy()
{
    // wait until the jump is finished
    return this->is_jumping;
};