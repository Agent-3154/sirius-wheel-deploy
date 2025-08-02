#ifndef RL_CONSTANTS_H_
#define RL_CONSTANTS_H_

#include <cstdint>

namespace RLConstants {
    // Tensor dimensions for ONNX model inference
    constexpr int64_t OBSERVATION_DIM = 49;      // Dimension of observation vector
    constexpr int64_t ACTION_DIM = 16;           // Dimension of action vector
    constexpr int64_t OBSERVATION_HISTORY_DIM = 245;  // 49 * 5 history steps
    
    // Model file path
    constexpr const char* POLICY_PATH = "../models/policy_whl_1.onnx";
    
    // ONNX input/output tensor names
    constexpr const char* INPUT_NAME_OBS = "obs";
    constexpr const char* INPUT_NAME_OBS_HISTORY = "obs_history";
    constexpr const char* OUTPUT_NAME_ACTIONS = "actions";
    
    // History configuration
    constexpr int NUM_HISTORY_STEPS = 5;  // Number of observation history steps to maintain
    
    // Action scaling factors
    constexpr double WHEEL_ACTION_SCALE = 10.0;  // Scaling factor for wheel actions
    constexpr double LEG_ACTION_SCALE = 0.5;     // Scaling factor for leg actions
    
    // Observation scaling factors
    constexpr double BODY_ANG_VEL_SCALE = 0.25;
    constexpr double VEL_COMMAND_SCALE_X = 2.0;
    constexpr double VEL_COMMAND_SCALE_Y = 2.0;
    constexpr double VEL_COMMAND_SCALE_W = 0.25;
    constexpr double JOINT_VEL_SCALE = 0.05;
    constexpr double LEG_PHASE_SCALE = 0.1;
}

#endif  // RL_CONSTANTS_H_ 