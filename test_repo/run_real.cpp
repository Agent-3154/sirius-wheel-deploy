//
// Created by lingwei on 4/29/24.
//
#include "../robot/HardwareBridge.h"
#include <iostream>

int main(int argc, char **argv) {
#if defined DG_ENGINEER
    std::string model_name = "../robot/robot_model/dg_engineer/scene.xml";
    bool launch_imu = true;
    bool launch_usb2can = true;
    Config::run_type type_ = Config::real_usb;
#elif defined SIRIUS_WHEEL
    std::string model_name = "../robot/robot_model/sirius_wheel/scene.xml";
    bool launch_imu = true;
    bool launch_usb2can = true;
    Config::run_type type_ = Config::real_usb;
#endif

    Eigen::setNbThreads(1);
    iox::runtime::PoshRuntime::initRuntime("Robot_Ctrl_Node");
    HardwareBridge::My_HardwareBridge test_hardware(model_name, type_);
    test_hardware.setup_HardwareBridge(launch_imu, launch_usb2can);
    test_hardware.setup_rc("");
    test_hardware.setup_runner();
    test_hardware.run();
    return 0;
}
