//
// Created by lingwei on 4/29/24.
//
#include "../robot/HardwareBridge.h"
#include <iostream>
#include "../robot_ctrl/my_controller.h"

int main(int argc, char **argv) {
#if defined BELT
    std::string model_name = "../robot/robot_model/belt/scene.xml";
    bool launch_imu = true;
    bool launch_usb2can = true;
    bool launch_rc = true;
    bool unitree = false;
    run_type type_ = real_usb;
#elif defined CHAOJI_GO
    std::string model_name = "../robot/robot_model/chaojigou/scene.xml";
#elif defined GO1
    std::string model_name = "../robot/robot_model/unitree_go1/scene.xml";
#elif defined DG_ENGINEER
    std::string model_name = "../robot/robot_model/dg_engineer/scene.xml";
#elif defined SIRIUS_WHEEL
    std::string model_name = "../robot/robot_model/sirius_wheel/scene.xml";
#endif
    bool launch_imu = false;
    bool launch_usb2can = false;
    bool launch_rc = true;
    bool unitree = false;
    Config::run_type type_ = Config::sim_mj;
    auto *robot_ctrl = new My_Controller();
    Eigen::setNbThreads(1);
#if defined (SIMULATOR)
    iox::runtime::PoshRuntime::initRuntime("Sim_Ctrl_Node");
#endif
    HardwareBridge::My_HardwareBridge sim_ctrl(model_name, robot_ctrl, type_);
    sim_ctrl.setup_HardwareBridge(launch_imu, launch_usb2can, launch_rc, unitree);
    return 0;
}
