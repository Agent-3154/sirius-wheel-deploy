#include <iostream>
#include "FSM_State.h"
#include <onnxruntime_cxx_api.h>
#include "../../utilities/types/std_cout_colors.h"

// constants for tensor dimensions
const int64_t COMMAND_DIM = 17;
const int64_t POLICY_DIM = 147; // Updated to match JSON configuration
const int64_t ACTION_DIM = 16;
const int64_t HIDDEN_STATE_DIM = 128; // for GRU
const int64_t HISTORY_STEPS = 4;

class FSM_State_RL final : public FSM_State
{
private:
    std::unique_ptr<Ort::Session> session;
    Ort::RunOptions run_options = Ort::RunOptions{nullptr};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Eigen::Vector4d quat;
    Eigen::Vector3d gyro;
    Eigen::Vector3d projected_gravity;

    std::vector<float> command;
    std::vector<float> policy;
    bool is_init[1]; // Use bool array instead of std::vector<bool>
    std::vector<float> hx;

    Eigen::Matrix<float, 12, HISTORY_STEPS> q_buffer;  // joint position history in ISAAC order
    Eigen::Matrix<float, 16, HISTORY_STEPS> qd_buffer; // joint velocity history in ISAAC order
    Eigen::Matrix<float, 16, 2> prev_actions;          // previous actions
    Eigen::Matrix<float, 4, 3> desired_leg_jpos;
    Eigen::Matrix<float, 4, 3> desired_leg_jpos_filtered;
    Eigen::Vector<float, 4> desired_whl_jvel;

    // Eigen::Vector<float, 12> q_leg;
    // Eigen::Vector<float, 12> qd_leg;
    // Eigen::Vector<float, 12> q_leg_prev;  // Previous joint positions for finite differencing
    // Eigen::Vector<float, 12> qd_leg_fd;
    const float dt = 0.002f;  // Time step in seconds (assuming 1kHz control loop)

    const Eigen::VectorXf DEFAULT_JOINT_POS = (Eigen::VectorXf(16) << 0.0, 0.0, 0.0, 0.0,
                                               0.40, -0.40, 0.40, -0.40,
                                               -1.20, 1.20, -1.20, 1.20,
                                               0.0, 0.0, 0.0, 0.0)
                                                  .finished();

    const std::vector<int64_t> command_shape = {1, COMMAND_DIM};
    const std::vector<int64_t> policy_shape = {1, POLICY_DIM};
    const std::vector<int64_t> is_init_shape = {1};
    const std::vector<int64_t> hx_shape = {1, HIDDEN_STATE_DIM};
    int64_t step_count = 0;

    const std::array<std::string, 16> ISAAC_JORDER = {
        "LF_HAA", "LH_HAA", "RF_HAA", "RH_HAA",
        "LF_HFE", "LH_HFE", "RF_HFE", "RH_HFE",
        "LF_KFE", "LH_KFE", "RF_KFE", "RH_KFE",
        "LF_WHEEL", "LH_WHEEL", "RF_WHEEL", "RH_WHEEL"
    };

    const std::array<std::string, 16> REAL_JORDER = {
        "RF_HAA", "RF_HFE", "RF_KFE",
        "LF_HAA", "LF_HFE", "LF_KFE",
        "RH_HAA", "RH_HFE", "RH_KFE",
        "LH_HAA", "LH_HFE", "LH_KFE",
        "RF_WHEEL", "LF_WHEEL", "RH_WHEEL", "LH_WHEEL"
    };

    std::array<int, 16> isaac2real;
    std::array<int, 16> real2isaac;

public:
    FSM_State_RL(
        Control_FSM_Data_t *controlfsmdata,
        Control_Parameters_t *control_para) : FSM_State(controlfsmdata, control_para, RL)
    {

        std::cout << GREEN << "[FSM State RL]: Ort version: " << ORT_API_VERSION << RESET << std::endl;

        const std::string policy_path = "/home/btx0424/lab45/sirius_deploy/checkpoints/policy-06-24_14-34.onnx";
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXInference");
        Ort::SessionOptions session_options;
        session = std::make_unique<Ort::Session>(env, policy_path.c_str(), session_options);

        // Initialize observation vector with 49 zeros
        command.resize(COMMAND_DIM, 0.0f);
        policy.resize(POLICY_DIM, 0.0f);
        is_init[0] = false; // Initialize the bool array
        hx.resize(HIDDEN_STATE_DIM, 0.0f);
        
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

    ~FSM_State_RL() override = default;

    bool state_on_enter() override
    {
        std::cout << "[FSM State RL]: state_on_enter" << std::endl;
        for (auto &leg: fsm_data_->leg_controller_->leg_command)
        {
            leg.kp_joint = Vec3<double>(40, 40, 40).asDiagonal();
            leg.kd_joint = Vec3<double>(1, 1, 1).asDiagonal();
            leg.whl_kp_joint = 0;
            leg.whl_kd_joint = 5.0;
        }
        return true;
    };

    void state_on_exit() override
    {
        return;
    };

    void run_state() override
    {
        step_count++;

        quat = fsm_data_->estimators_->get_result_quat();
        gyro = fsm_data_->estimators_->get_result_angular_body();

        float v_des_x = fsm_data_->rc_->rc_control_.v_des[0] * 1.6;
        float v_des_y = fsm_data_->rc_->rc_control_.v_des[1] * 0.8;
        float v_des_z = fsm_data_->rc_->rc_control_.v_des[2];



        if (step_count % 10 == 0)
        {
            Eigen::Matrix<float, 4, 3> q_leg;  q_leg.setZero();// joint position in ISAAC order
            Eigen::Matrix<float, 4, 3> qd_leg; qd_leg.setZero(); // joint velocity in ISAAC order

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
            auto q_leg_flat = Eigen::Map<Eigen::Vector<float, 12>>(q_leg.data());
            auto qd_leg_flat = Eigen::Map<Eigen::Vector<float, 12>>(qd_leg.data());

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
            Eigen::Vector<float, 147> _policy; _policy.setZero();

            _policy << projected_gravity.cast<float>(),
                Eigen::Map<Eigen::Vector<float, 48>>(q_buffer.data()),
                Eigen::Map<Eigen::Vector<float, 64>>(qd_buffer.data()),
                Eigen::Map<Eigen::Vector<float, 32>>(prev_actions.data());
                        
            // Copy to policy vector
            std::copy(_policy.data(), _policy.data() + POLICY_DIM, this->policy.begin());

            Eigen::Vector<float, COMMAND_DIM> _command; _command.setZero();
            Eigen::Vector<float, 2> cmd_lin_vel; cmd_lin_vel << v_des_x, v_des_y;
            Eigen::Vector<float, 3> cmd_ang_vel; cmd_ang_vel << 0.0, 0.0, v_des_z;
            Eigen::Vector<float, 1> cmd_roll; cmd_roll.setZero();
            Eigen::Vector<float, 1> cmd_pitch; cmd_pitch.setZero();
            Eigen::Vector<float, 2> timing; timing.setZero();
            Eigen::Vector<float, 4> cmd_mode; cmd_mode << 1.0, 0.0, 0.0, 0.0;
            Eigen::Vector<float, 4> des_contact; des_contact.setZero();

            _command << cmd_lin_vel, cmd_ang_vel, cmd_roll, cmd_pitch, timing, cmd_mode, des_contact;
            std::copy(_command.data(), _command.data() + COMMAND_DIM, this->command.begin());

            std::vector<Ort::Value> input_tensors;
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info,
                command.data(),
                command.size(),
                command_shape.data(),
                command_shape.size()));
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info,
                policy.data(),
                policy.size(),
                policy_shape.data(),
                policy_shape.size()));
            input_tensors.push_back(Ort::Value::CreateTensor<bool>(
                memory_info,
                is_init,
                1,
                is_init_shape.data(),
                is_init_shape.size()));
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info,
                hx.data(),
                hx.size(),
                hx_shape.data(),
                hx_shape.size()));

            const char *input_names[] = {"command", "policy", "is_init", "hx"};
            const char *output_names[] = {"div", "div_1", "linear_4", "add_3", "mish_4", "linear_8", "mul_2", "linear_8", "sum_1"};
            auto output_tensors = session->Run(
                run_options,
                input_names,
                input_tensors.data(),
                session->GetInputCount(),
                output_names,
                session->GetOutputCount());

            // get action and convert to Eigen
            auto action = output_tensors[7].GetTensorData<float>();
            Eigen::Map<const Eigen::VectorXf> action_eigen(action, ACTION_DIM);
            
            this->prev_actions.col(1) = this->prev_actions.col(0);
            this->prev_actions.col(0) = action_eigen;

            Eigen::Vector<float, 12> desired_leg_jpos_ = action_eigen.head(12) * 0.5 + DEFAULT_JOINT_POS.head(12);
            
            this->desired_leg_jpos = Eigen::Map<Eigen::Matrix<float, 4, 3>>(desired_leg_jpos_.data());
            this->desired_whl_jvel = action_eigen.tail(4) * 10.0;

            // get next_hx and copy to hx
            auto next_hx = output_tensors[3].GetTensorData<float>();
            std::copy(next_hx, next_hx + HIDDEN_STATE_DIM, this->hx.begin());
        }
        desired_leg_jpos_filtered = desired_leg_jpos_filtered * 0.2 + desired_leg_jpos * 0.8;
        fsm_data_->leg_controller_->leg_command[0].q_des = desired_leg_jpos_filtered.row(2).cast<double>();
        fsm_data_->leg_controller_->leg_command[1].q_des = desired_leg_jpos_filtered.row(0).cast<double>();
        fsm_data_->leg_controller_->leg_command[2].q_des = desired_leg_jpos_filtered.row(3).cast<double>();
        fsm_data_->leg_controller_->leg_command[3].q_des = desired_leg_jpos_filtered.row(1).cast<double>();
        
        fsm_data_->leg_controller_->leg_command[0].whl_qd_des = double(desired_whl_jvel(2));
        fsm_data_->leg_controller_->leg_command[1].whl_qd_des = double(desired_whl_jvel(0));
        fsm_data_->leg_controller_->leg_command[2].whl_qd_des = double(desired_whl_jvel(3));
        fsm_data_->leg_controller_->leg_command[3].whl_qd_des = double(desired_whl_jvel(1));

        for (auto &leg: fsm_data_->leg_controller_->leg_command)
        {
            leg.kp_joint = Vec3<double>(40, 40, 40).asDiagonal();
            leg.kd_joint = Vec3<double>(1, 1, 1).asDiagonal();
            leg.whl_kp_joint = 0;
            leg.whl_kd_joint = 5.0;
        }
    };

    bool is_busy() override
    {
        return false;
    };
};