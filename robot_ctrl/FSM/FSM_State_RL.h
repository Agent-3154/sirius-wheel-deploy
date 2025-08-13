#ifndef FSM_STATE_RL_H
#define FSM_STATE_RL_H

#include "FSM_State.h"
#include <memory>
#include <vector>
#include <array>
#include <string>
#include <onnxruntime_cxx_api.h>
#include "../../lcm-types/cpp/leg_control_data_lcmt.hpp"
#include "../../lcm-types/cpp/leg_control_command_lcmt.hpp"

class FSM_State_RL final : public FSM_State
{
private:
    std::unique_ptr<Ort::Session> session;
    std::unique_ptr<Ort::RunOptions> run_options;
    std::unique_ptr<Ort::MemoryInfo> memory_info;

    Eigen::Vector4d quat;
    Eigen::Vector3d gyro;
    Eigen::Vector3d rpy;
    Eigen::Vector3d rpy_init;
    Eigen::Vector3d projected_gravity;

    std::vector<float> policy;
    bool is_init[1]; // Use bool array instead of std::vector<bool>
    std::vector<float> hx;

    Eigen::Matrix<float, 12, 4> q_buffer;     // joint position history in ISAAC order
    Eigen::Matrix<float, 16, 4> qd_buffer;    // joint velocity history in ISAAC order
    Eigen::Matrix<float, 16, 2> prev_actions; // previous actions
    Eigen::Matrix<float, 4, 3> desired_leg_jpos;
    Eigen::Matrix<float, 4, 3> desired_leg_jpos_filtered;
    Eigen::Vector4f desired_whl_jvel;
    const Eigen::VectorXf DEFAULT_LEG_JOINT_POS = (
        Eigen::VectorXf(12) << 0.0, 0.0, 0.0, 0.0,
                            0.40, -0.40, 0.40, -0.40,
                            -1.20, 1.20, -1.20, 1.20
                            ).finished();
    
    Eigen::Vector3f cmd_lin_vel;
    Eigen::Vector3f cmd_rpy;
    Eigen::Vector3f cmd_ang_vel;
    Eigen::Vector3f des_ang_vel;
    Eigen::Vector4f des_contact;
    Eigen::Vector4f cmd_mode;

    const float dt = 0.002f; // Time step in seconds (assuming 1kHz control loop)

    const std::vector<int64_t> command_shape = {1, COMMAND_DIM};
    const std::vector<int64_t> policy_shape = {1, POLICY_DIM};
    const std::vector<int64_t> is_init_shape = {1};
    const std::vector<int64_t> hx_shape = {1, HIDDEN_STATE_DIM};
    int64_t step_count = 0;

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
    float cmd_jump_time = 0.0;
    
    // logging with lcm
    lcm::LCM lcm_logger_;
    leg_control_data_lcmt lcm_leg_control_data{};
    leg_control_command_lcmt lcm_leg_control_cmd{};
public:
    static constexpr int64_t COMMAND_DIM = 18;
    static constexpr int64_t POLICY_DIM = 147; // Updated to match JSON configuration
    static constexpr int64_t ACTION_DIM = 16;
    static constexpr int64_t HIDDEN_STATE_DIM = 128; // for GRU
    static constexpr int64_t HISTORY_STEPS = 4;

    static constexpr float JUMP_PREP_TIME = 0.5;
    static constexpr float JUMP_LAND_TIME = 0.4;

    static constexpr float LEG_KP = 40.0;
    static constexpr float LEG_KD = 1.0;
    static constexpr float WHEEL_KD = 10.0;

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