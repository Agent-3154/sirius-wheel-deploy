#include <iostream>
#include "FSM_State.h"
#include <onnxruntime_cxx_api.h>
#include "../../utilities/types/std_cout_colors.h"

// constants for tensor dimensions
const int64_t command_dim = 17;
const int64_t policy_dim = 147;  // Updated to match JSON configuration
const int64_t action_dim = 16;
const int64_t hidden_state_dim = 128; // for GRU

class FSM_State_RL final : public FSM_State
{
private:

    std::unique_ptr<Ort::Session> session;
    Ort::RunOptions run_options = Ort::RunOptions{nullptr};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    
    std::vector<float> command;
    std::vector<float> policy;
    bool is_init[1];  // Use bool array instead of std::vector<bool>
    std::vector<float> hx;
    
    const std::vector<int64_t> command_shape = {1, command_dim};
    const std::vector<int64_t> policy_shape = {1, policy_dim};
    const std::vector<int64_t> is_init_shape = {1};
    const std::vector<int64_t> hx_shape = {1, hidden_state_dim};

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
        command.resize(command_dim, 0.0f);
        policy.resize(policy_dim, 0.0f);
        is_init[0] = false; // Initialize the bool array
        hx.resize(hidden_state_dim, 0.0f);

        std::cout << GREEN << "[FSM State RL]: Policy Loaded" << RESET << std::endl;

        auto output_names_vector = session->GetOutputNames();
        for (const auto& name : output_names_vector) {
            std::cout << GREEN << name << RESET << std::endl;
        }
    }

    ~FSM_State_RL() override = default;

    bool state_on_enter() override {
        return true;
    };

    void state_on_exit() override {
        return;
    };

    void run_state() override {

        auto quat = fsm_data_->estimators_->get_result_quat();
        auto angular_body = fsm_data_->estimators_->get_result_angular_body();
        
        Eigen::Matrix<double, 4, 3> q; // joint position
        Eigen::Matrix<double, 4, 3> qd; // joint velocity
        for (int i = 0; i < 4; i++) {
            q.row(i) = fsm_data_->leg_controller_->leg_data[i].q;
            qd.row(i) = fsm_data_->leg_controller_->leg_data[i].qd;
        }

        // // print q
        // std::cout << "q: " << q << std::endl;

        std::vector<Ort::Value> input_tensors;
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info,
            command.data(),
            command.size(),
            command_shape.data(),
            command_shape.size()
        ));
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info,
            policy.data(),
            policy.size(),
            policy_shape.data(),
            policy_shape.size()
        ));
        input_tensors.push_back(Ort::Value::CreateTensor<bool>(
            memory_info,
            is_init,
            1,
            is_init_shape.data(),
            is_init_shape.size()
        ));
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info,
            hx.data(),
            hx.size(),
            hx_shape.data(),
            hx_shape.size()
        ));

        const char* input_names[] = {"command", "policy", "is_init", "hx"};
        const char* output_names[] = {"div", "div_1", "linear_4", "add_3", "mish_4", "linear_8", "mul_2", "linear_8", "sum_1"};
        auto output_tensors = session->Run(
            run_options,
            input_names,
            input_tensors.data(),
            session->GetInputCount(),
            output_names,
            session->GetOutputCount()
        );
    };

    bool is_busy() override {
        return false;
    };
};