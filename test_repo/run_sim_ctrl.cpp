//
// Created by lingwei on 4/29/24.
//
#include "../robot/HardwareBridge.h"
#include <iostream>

int main(int argc, char **argv) {
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
    sim_ctrl.robot_runner_->fsm_->state_list_.s_rl->load_policy("/home/btx0424/lab50/active-adaptation/scripts/exports/SiriusATEC/policy-11-11_19-06.onnx");
    sim_ctrl.run();
    return 0;
}
