//
// Created by lingwei on 4/29/24.
//
#include "../robot/HardwareBridge.h"
#include <iostream>
#include <cxxopts.hpp>

int main(int argc, char **argv) {
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
    
    const char* env_model = std::getenv("SIRIUS_MODEL");
    std::string model_name;
    
    if (env_model != nullptr && std::string(env_model) == "wheel") {
        model_name = "../robot/robot_model/sirius_wheel_new/scene.xml";
        std::cout << "Using Wheel model: " << model_name << std::endl;
    } else {
        model_name = "../robot/robot_model/ly-mid-p-0916/scene.xml";
        std::cout << "Using Point-Foot model: " << model_name << std::endl;
    }
    bool launch_imu = true;
    bool launch_usb2can = true;
    Config::run_type type_ = Config::real_usb;

    Eigen::setNbThreads(1);
    iox::runtime::PoshRuntime::initRuntime("Robot_Ctrl_Node");
    HardwareBridge::My_HardwareBridge test_hardware(model_name, type_);
    test_hardware.setup_HardwareBridge(launch_imu, launch_usb2can);
    test_hardware.setup_rc("");
    test_hardware.setup_runner();
    std::cout << "Loading policy from: " << policy_path << std::endl;
    test_hardware.robot_runner_->fsm_->state_list_.s_rl->load_policy(policy_path.c_str());
    test_hardware.run();
    return 0;
}
