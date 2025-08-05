#include "FSM_State_RL.h"
#include <iostream>
// #include <onnxruntime_cxx_api.h>
// #include <eigen3/Eigen/Dense>
// #include "../../utilities/types/std_cout_colors.h"

FSM_State_RL::FSM_State_RL(
    Control_FSM_Data_t *controlfsmdata,
    Control_Parameters_t *control_para) : FSM_State(controlfsmdata, control_para, RL)
{

    std::cout << GREEN << "[FSM State RL]: Ort version: " << ORT_API_VERSION << RESET << std::endl;

    const std::string policy_path = "../models/policy-06-24_14-34.onnx";
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXInference");
    Ort::SessionOptions session_options;
    session = std::make_unique<Ort::Session>(env, policy_path.c_str(), session_options);

    // Initialize ONNX Runtime objects
    run_options = std::make_unique<Ort::RunOptions>(Ort::RunOptions(nullptr));
    memory_info = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    // Initialize observation vector with 49 zeros
    policy.resize(POLICY_DIM, 0.0f);
    is_init[0] = false; // Initialize the bool array
    hx.resize(HIDDEN_STATE_DIM, 0.0f);

    cmd_lin_vel.setZero();
    cmd_ang_vel.setZero();

    prev_actions.setZero();
    desired_leg_jpos.setZero();
    desired_leg_jpos_filtered.setZero();

    // [Debug] fill q with arange(history * 12)
    for (int i = 0; i < HISTORY_STEPS; i++)
    {
        for (int j = 0; j < 12; j++)
        {
            q_buffer(i, j) = i * 12 + j;
        }
    }

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
        leg.kp_joint = Vec3<double>(40, 40, 40).asDiagonal();
        leg.kd_joint = Vec3<double>(1, 1, 1).asDiagonal();
        leg.whl_kp_joint = 0;
        leg.whl_kd_joint = 5.0;
    }
    this->is_jumping = false;
    this->cmd_jump_time = 0.0;
    this->hx.resize(HIDDEN_STATE_DIM, 0.0f);
    return true;
};

void FSM_State_RL::state_on_exit()
{
    return;
};

void FSM_State_RL::run_state()
{
    step_count++;

    quat = fsm_data_->estimators_->get_result_quat();
    gyro = fsm_data_->estimators_->get_result_angular_body();

    float v_des_x = fsm_data_->rc_->rc_control_.v_des[0] * 1.2;
    float v_des_y = fsm_data_->rc_->rc_control_.v_des[1] * 0.8;
    float v_des_z = fsm_data_->rc_->rc_control_.v_des[2];

    
    if (fsm_data_->rc_->rc_map_.a && !this->is_jumping)
    {
        this->is_jumping = true;
    }

    if (step_count % 10 == 0)
    {
        Eigen::Matrix<float, 4, 3> q_leg;
        q_leg.setZero(); // joint position in ISAAC order
        Eigen::Matrix<float, 4, 3> qd_leg;
        qd_leg.setZero(); // joint velocity in ISAAC order

        q_leg.row(0) = fsm_data_->leg_controller_->leg_data[1].q.cast<float>();   // RF
        q_leg.row(1) = fsm_data_->leg_controller_->leg_data[3].q.cast<float>();   // LF
        q_leg.row(2) = fsm_data_->leg_controller_->leg_data[0].q.cast<float>();   // RH
        q_leg.row(3) = fsm_data_->leg_controller_->leg_data[2].q.cast<float>();   // LH
        qd_leg.row(0) = fsm_data_->leg_controller_->leg_data[1].qd.cast<float>(); // RF
        qd_leg.row(1) = fsm_data_->leg_controller_->leg_data[3].qd.cast<float>(); // LF
        qd_leg.row(2) = fsm_data_->leg_controller_->leg_data[0].qd.cast<float>(); // RH
        qd_leg.row(3) = fsm_data_->leg_controller_->leg_data[2].qd.cast<float>(); // LH

        // shift history
        for (int i = HISTORY_STEPS - 1; i > 0; i--)
        {
            this->q_buffer.col(i) = this->q_buffer.col(i - 1);
            this->qd_buffer.col(i) = this->qd_buffer.col(i - 1);
        }
        auto q_leg_flat = Eigen::Map<Eigen::VectorXf>(q_leg.data(), 12);
        auto qd_leg_flat = Eigen::Map<Eigen::VectorXf>(qd_leg.data(), 12);

        this->q_buffer.col(0) = q_leg_flat;
        this->qd_buffer.col(0) << qd_leg_flat,
            float(fsm_data_->leg_controller_->leg_data[1].whl_qd),
            float(fsm_data_->leg_controller_->leg_data[3].whl_qd),
            float(fsm_data_->leg_controller_->leg_data[0].whl_qd),
            float(fsm_data_->leg_controller_->leg_data[2].whl_qd);

        Eigen::Quaterniond quat_eigen(quat[0], quat[1], quat[2], quat[3]);
        this->projected_gravity = (quat_eigen.inverse() * Eigen::Vector3d(0, 0, -1));

        // observation "policy":
        // concat [projected_gravity(3), q_buffer.flatten(48), qd_buffer.flatten(64), prev_actions.flatten(32)] = 147 total
        Eigen::VectorXf _policy(147);
        _policy.setZero();

        _policy << projected_gravity.cast<float>(),
            Eigen::Map<Eigen::VectorXf>(q_buffer.data(), 48),
            Eigen::Map<Eigen::VectorXf>(qd_buffer.data(), 64),
            Eigen::Map<Eigen::VectorXf>(prev_actions.data(), 32);

        // Copy to policy vector
        std::copy(_policy.data(), _policy.data() + POLICY_DIM, this->policy.begin());

        Eigen::VectorXf _command(COMMAND_DIM); _command.setZero();
        
        Eigen::Vector3f v_des_xy = Eigen::Vector3f(v_des_x, v_des_y, 0.0);
        this->cmd_lin_vel = this->cmd_lin_vel * 0.5 + v_des_xy * 0.5;
        this->cmd_ang_vel << 0.0, 0.0, v_des_z;

        Eigen::Vector2f cmd_roll_pitch; cmd_roll_pitch.setZero();
        Eigen::Vector2f timing;
        Eigen::Vector4f cmd_mode;
        Eigen::Vector4f des_contact;

        if (this->is_jumping) {
            timing << this->cmd_jump_time, 1.0 - this->cmd_jump_time;
            cmd_mode << 0.0, 0.0, 1.0, 0.0;
            bool in_air = (this->cmd_jump_time > 0.4) && (this->cmd_jump_time < 0.7);
            if (in_air) {
                des_contact << -1.0, -1.0, -1.0, -1.0;
            } else {
                des_contact << 0.0, 0.0, 0.0, 0.0;
            }
            this->cmd_jump_time += 0.02;
            if (this->cmd_jump_time > 1.0) {
                this->is_jumping = false;
                this->cmd_jump_time = 0.0;
            }
        } else {
            timing << 0.0, 0.0;
            cmd_mode << 1.0, 0.0, 0.0, 0.0;
            des_contact << 0.0, 0.0, 0.0, 0.0;
        }

        _command << cmd_lin_vel.head(2), cmd_ang_vel, cmd_roll_pitch, timing, cmd_mode, des_contact;

        std::vector<Ort::Value> input_tensors;
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            *this->memory_info,
            _command.data(),
            _command.size(),
            command_shape.data(),
            command_shape.size()));
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
            is_init_shape.data(),
            is_init_shape.size()));
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            *this->memory_info,
            hx.data(),
            hx.size(),
            hx_shape.data(),
            hx_shape.size()));

        const char *input_names[] = {"command", "policy", "is_init", "hx"};
        const char *output_names[] = {"div", "div_1", "linear_4", "add_3", "mish_4", "linear_8", "mul_2", "linear_8", "sum_1"};
        auto output_tensors = session->Run(
            *run_options,
            input_names,
            input_tensors.data(),
            session->GetInputCount(),
            output_names,
            session->GetOutputCount());

        // get action and convert to Eigen
        auto action = output_tensors[7].GetTensorMutableData<float>();
        Eigen::Map<const Eigen::VectorXf> action_eigen(action, ACTION_DIM);
        
        this->prev_actions.col(1) = this->prev_actions.col(0);
        this->prev_actions.col(0) = action_eigen;

        Eigen::VectorXf desired_leg_jpos_ = action_eigen.head(12) * 0.5 + DEFAULT_JOINT_POS.head(12);
        
        this->desired_leg_jpos = Eigen::Map<Eigen::Matrix<float, 4, 3>>(desired_leg_jpos_.data());
        this->desired_whl_jvel = action_eigen.tail(4) * 10.0;

        // get next_hx and copy to hx
        auto next_hx = output_tensors[3].GetTensorMutableData<float>();
        std::copy(next_hx, next_hx + HIDDEN_STATE_DIM, this->hx.begin());
    }
    // only update if desired_leg_jpos does not contain nan
    if (!desired_leg_jpos.hasNaN()) {
        desired_leg_jpos_filtered = desired_leg_jpos_filtered * 0.2 + desired_leg_jpos * 0.8;
    }
    
    if (this -> apply_action)
    {
        fsm_data_->leg_controller_->leg_command[0].q_des = desired_leg_jpos_filtered.row(2).cast<double>();
        fsm_data_->leg_controller_->leg_command[1].q_des = desired_leg_jpos_filtered.row(0).cast<double>();
        fsm_data_->leg_controller_->leg_command[2].q_des = desired_leg_jpos_filtered.row(3).cast<double>();
        fsm_data_->leg_controller_->leg_command[3].q_des = desired_leg_jpos_filtered.row(1).cast<double>();

        fsm_data_->leg_controller_->leg_command[0].whl_qd_des = double(desired_whl_jvel(2));
        fsm_data_->leg_controller_->leg_command[1].whl_qd_des = double(desired_whl_jvel(0));
        fsm_data_->leg_controller_->leg_command[2].whl_qd_des = double(desired_whl_jvel(3));
        fsm_data_->leg_controller_->leg_command[3].whl_qd_des = double(desired_whl_jvel(1));
    }

    for (auto &leg : fsm_data_->leg_controller_->leg_command)
    {
        leg.kp_joint = Vec3<double>(40, 40, 40).asDiagonal();
        leg.kd_joint = Vec3<double>(1, 1, 1).asDiagonal();
        leg.whl_kp_joint = 0;
        leg.whl_kd_joint = 5.0;
    }
};

bool FSM_State_RL::is_busy()
{
    // wait until the jump is finished
    return this->is_jumping;
};