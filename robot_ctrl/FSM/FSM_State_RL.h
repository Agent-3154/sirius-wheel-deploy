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
    std::unique_ptr<Ort::Session> session;
    std::unique_ptr<Ort::RunOptions> run_options;
    std::unique_ptr<Ort::MemoryInfo> memory_info;

    // IMU states
    Eigen::Vector4d quat;
    Eigen::Vector3d gyro;
    Eigen::Vector3d rpy;
    Eigen::Vector3d rpy_init;

    std::vector<float> policy;
    bool is_init[1]; // Use bool array instead of std::vector<bool>
    std::vector<float> hx;

    Eigen::Matrix<float, 12, 10> raw_jpos_buffer_;
    Eigen::Matrix<float, 16, 10> raw_jvel_buffer_;
    
    // joint velocity history in ISAAC order
    Eigen::Matrix<float, 4, 3> desired_leg_jpos_;
    Eigen::Matrix<float, 4, 3> desired_leg_jpos_filtered_;
    Eigen::Vector4f desired_whl_jvel_;
    const Eigen::VectorXf DEFAULT_LEG_JOINT_POS = 
    (
        Eigen::VectorXf(12) << 0.1, 0.1, -0.1, -0.1,
                            0.95, -0.95, 0.95, -0.95,
                            -1.60, 1.60, -1.60, 1.60
    ).finished();
    // (
    //     Eigen::VectorXf(12) << 0.0, 0.0, 0.0, 0.0,
    //                         0.40, -0.40, 0.40, -0.40,
    //                         -1.20, 1.20, -1.20, 1.20
    // ).finished();
    
    Eigen::Vector3f cmd_lin_vel_b_;
    Eigen::Vector3f cmd_lin_vel_w_;
    Eigen::Vector3f des_rpy_; // global target rpy
    Eigen::Vector3f ref_rpy_;
    Eigen::Vector3f cmd_ang_vel_;
    Eigen::Vector3f ref_ang_vel_;
    Eigen::Vector4f des_contact_;
    Eigen::Vector2f cmd_mode_;
    float ref_vel_;
    float ref_hei_;
    
    const float dt = 0.002f; // Time step in seconds (assuming 1kHz control loop)
    SecondOrderLowPassFilter jvel_filter_1;
    SecondOrderLowPassFilter jvel_filter_2;
    
    Eigen::VectorXf obs_command_;
    Eigen::VectorXf obs_policy_;
    const std::vector<int64_t> obs_command_shape = {1, COMMAND_DIM};
    const std::vector<int64_t> is_init_shape = {1};
    const std::vector<int64_t> hx_shape = {1, HIDDEN_STATE_DIM};
    std::vector<int64_t> obs_policy_shape;
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
    
    bool is_jumping = false;
    float jump_turn_ = 0.0;
    float jump_air_time_ = 0.0;
    float cmd_time_ = 0.0;
    float cmd_duration_ = 0.0;
    
    void step_command();
    void compute_command();
    void compute_observation();
    void run_inference(bool apply_action);

    std::vector<std::unique_ptr<Observation>> observations_;
public:
    static constexpr int64_t COMMAND_DIM = 15;
    static constexpr int64_t ACTION_DIM = 16;
    static constexpr int64_t HIDDEN_STATE_DIM = 128; // for GRU
    static constexpr int64_t HISTORY_STEPS = 6;

    static constexpr float JUMP_PREP_TIME = 0.6;
    static constexpr float JUMP_TAKEOFF_TIME = 0.38;
    static constexpr float JUMP_LAND_TIME = 0.8;
    
    static constexpr float LEG_ACTION_SCALE = 1.0;
    static constexpr float WHEEL_ACTION_SCALE = 10.0;
    static constexpr float LEG_KP = 52.0;
    static constexpr float LEG_KD = 2.4;
    static constexpr float WHEEL_KD = 10.0;
    
    Eigen::Matrix<float, 16, 2> prev_actions_; // previous actions
    Eigen::Matrix<float, 12, HISTORY_STEPS> obs_jpos_buffer_;  // joint position history in ISAAC order
    Eigen::Matrix<float, 4, 4> obs_jvel_buffer_;   // wheels only
    Eigen::Vector3d projected_gravity_;
    Eigen::Vector4f cum_hip_deviation_;

    FSM_State_RL(
        Control_FSM_Data_t *controlfsmdata,
        Control_Parameters_t *control_para);

    ~FSM_State_RL() override = default;

    bool state_on_enter() override;
    void state_on_exit() override;
    void run_state() override;
    bool is_busy() override;
};


#endif // FSM_STATE_RL_H