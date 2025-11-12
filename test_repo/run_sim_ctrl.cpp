//
// Created by lingwei on 4/29/24.
//
#include "../robot/HardwareBridge.h"
#include <iostream>
#include <cxxopts.hpp>

int main(int argc, char **argv) {
    // Parse command-line arguments using cxxopts
    cxxopts::Options options("sim_ctrl", "Simulation control with RL policy");
    
    options.add_options()
        ("p,policy", "Path to the policy file (required)", cxxopts::value<std::string>())
        ("r,rerun-logging", "Enable rerun logging", cxxopts::value<bool>()->implicit_value("true")->default_value("false"))
        ("h,help", "Print usage");
    
    cxxopts::ParseResult result;
    try {
        result = options.parse(argc, argv);
    } catch (const cxxopts::exceptions::exception& e) {
        std::cerr << "Error parsing arguments: " << e.what() << std::endl;
        std::cerr << options.help() << std::endl;
        return 1;
    }
    
    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }
    
    // Check if policy path is provided
    if (!result.count("policy")) {
        std::cerr << "Error: Policy path is required. Use --policy or -p to specify it." << std::endl;
        std::cerr << std::endl;
        std::cerr << options.help() << std::endl;
        return 1;
    }
    
    std::string policy_path = result["policy"].as<std::string>();
    bool enable_rerun_logging = result["rerun-logging"].as<bool>();
    
    Config::run_type type_ = Config::sim_mj;
    Eigen::setNbThreads(1);
#if defined (SIMULATOR)
    iox::runtime::PoshRuntime::initRuntime("Sim_Ctrl_Node");
#endif
    
    const char* env_model = std::getenv("SIRIUS_MODEL");
    std::string model_name;
    
    if (env_model != nullptr && std::string(env_model) == "wheel") {
        model_name = "../robot/robot_model/sirius_wheel_new/scene.xml";
        std::cout << "Using Wheel model: " << model_name << std::endl;
    } else {
        model_name = "../robot/robot_model/ly-mid-p-0916/scene.xml";
        std::cout << "Using Point-Foot model: " << model_name << std::endl;
    }

    HardwareBridge::My_HardwareBridge sim_ctrl(model_name, type_);
    sim_ctrl.setup_HardwareBridge(false, false);
    // sim_ctrl.setup_rc("../robot/hardwares/usb/config/BTP-KP20.yaml");
    sim_ctrl.setup_rc("");
    sim_ctrl.setup_runner();
    std::cout << "Loading policy from: " << policy_path << std::endl;
    sim_ctrl.robot_runner_->fsm_->state_list_.s_rl->load_policy(policy_path.c_str());
    if (enable_rerun_logging) {
        std::cout << "Rerun logging: enabled" << std::endl;
        // TODO: Configure rerun logging when implemented
    }
    sim_ctrl.run();
    return 0;
}
