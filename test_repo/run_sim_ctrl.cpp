//
// Created by lingwei on 4/29/24.
//
#include "../robot/HardwareBridge.h"
#include <iostream>
#include "../robot_ctrl/my_controller.h"

int main(int argc, char **argv) {
    bool launch_imu = false;
    bool launch_usb2can = false;
    Config::run_type type_ = Config::sim_mj;
    auto *robot_ctrl = new My_Controller();
    Eigen::setNbThreads(1);
#if defined (SIMULATOR)
    iox::runtime::PoshRuntime::initRuntime("Sim_Ctrl_Node");
#endif
    std::string model_name = "../robot/robot_model/sirius_wheel_new/scene.xml";
    HardwareBridge::My_HardwareBridge sim_ctrl(model_name, robot_ctrl, type_);
    sim_ctrl.setup_HardwareBridge(launch_imu, launch_usb2can);
    sim_ctrl.setup_rc("../robot/hardwares/usb/config/BTP-KP20.yaml");
    sim_ctrl.setup_runner();
    sim_ctrl.run();
    return 0;
}
