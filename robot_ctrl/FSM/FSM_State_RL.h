#ifndef FSM_STATE_RL_H
#define FSM_STATE_RL_H

#include "FSM_State.h"
#include <memory>
#include <vector>
#include <array>
#include <string>
#include <onnxruntime_cxx_api.h>
#include "./filters.h"

// Forward declaration
class Observation;

class ONNXPolicy {
    private:
        std::unique_ptr<Ort::Session> session_;
        std::unique_ptr<Ort::RunOptions> run_options_;
        std::unique_ptr<Ort::MemoryInfo> memory_info_;

    public:
        ONNXPolicy(const std::string &model_path);
        ~ONNXPolicy() = default;

        void runInference(std::vector<Ort::Value> &input_tensors);
};

class FSM_State_RL final : public FSM_State
{
private:
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::RunOptions> run_options_;
    std::unique_ptr<Ort::MemoryInfo> memory_info_;

    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<const char*> input_names_cstr_;
    std::vector<const char*> output_names_cstr_;

    std::vector<float> policy;
    bool is_init[1]; // Use bool array instead of std::vector<bool>
    std::vector<float> hx;
    
    // joint velocity history in ISAAC order
    Eigen::Matrix<float, 4, 3> desired_leg_jpos_;
    Eigen::Matrix<float, 4, 3> desired_leg_jpos_filtered_;

    Eigen::VectorXf default_leg_jpos_;
    Eigen::VectorXf joint_stiffness_;
    Eigen::VectorXf joint_damping_;
    Eigen::Vector3f cmd_lin_vel_b_;
    Eigen::Vector3f cmd_lin_vel_w_;
    Eigen::Vector3f cmd_ang_vel_;
    
    const float dt = 0.002f; // Time step in seconds (assuming 1kHz control loop)
    SecondOrderLowPassFilter jvel_filter_1;
    SecondOrderLowPassFilter jvel_filter_2;
    
    Eigen::VectorXf obs_command_;
    Eigen::VectorXf obs_policy_;
    std::vector<int64_t> obs_command_shape;
    std::vector<int64_t> obs_policy_shape;
    const std::vector<int64_t> is_init_shape = {1};
    const std::vector<int64_t> hx_shape = {1, HIDDEN_STATE_DIM};
    int64_t loop_step_count_ = 0;
    int64_t ctrl_step_count_ = 0;

    const std::array<std::string, 16> ISAAC_JORDER = {
        "LF_HAA", "LH_HAA", "RF_HAA", "RH_HAA",
        "LF_HFE", "LH_HFE", "RF_HFE", "RH_HFE",
        "LF_KFE", "LH_KFE", "RF_KFE", "RH_KFE",
        "LF_WHEEL", "LH_WHEEL", "RF_WHEEL", "RH_WHEEL"};

    const std::array<std::string, 16> REAL_JORDER = {
        "RF_HAA", "RF_HFE", "RF_KFE",
        "LF_HAA", "LF_HFE", "LF_KFE",
        "RH_HAA", "RH_HFE", "RH_KFE",
        "LH_HAA", "LH_HFE", "LH_KFE",
        "RF_WHEEL", "LF_WHEEL", "RH_WHEEL", "LH_WHEEL"};

    bool apply_action = true; // set to false for dry-run
    
    void step_command();
    void computeObservation();
    void runInference(bool apply_action);

    std::vector<std::unique_ptr<Observation>> observations_;
public:
    static constexpr int64_t ACTION_DIM = 16;
    static constexpr int64_t HIDDEN_STATE_DIM = 128; // for GRU
    
    static constexpr float LEG_ACTION_SCALE = 1.0;
    static constexpr float WHEEL_ACTION_SCALE = 10.0;
    static constexpr float LEG_KP = 52.0;
    static constexpr float LEG_KD = 2.4;
    static constexpr float WHEEL_KD = 10.0;

    // IMU states
    Eigen::Vector4d quat_;
    Eigen::Vector4d quat_init_;
    Eigen::Vector3d gyro_;
    Eigen::Vector3d rpy_;
    Eigen::Vector3d rpy_init_;

    Eigen::Matrix<float, 12, 10> raw_jpos_buffer_;
    Eigen::Matrix<float, 16, 10> raw_jvel_buffer_;
    Eigen::Matrix<float, 16, 3> prev_actions_; // previous actions

    FSM_State_RL(
        Control_FSM_Data_t *controlfsmdata,
        Control_Parameters_t *control_para);

    ~FSM_State_RL() override = default;

    void load_policy(const std::string &policy_path);
    bool state_on_enter() override;
    void state_on_exit() override;
    void run_state() override;
    bool is_busy() override;
};


#endif // FSM_STATE_RL_H