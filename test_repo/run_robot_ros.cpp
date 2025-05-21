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
    bool launch_imu = true;
    bool launch_usb2can = true;
    bool launch_rc = false;
    bool unitree = false;
    Config::run_type type_ = Config::real_ros_ctrl;
#elif defined GO1
    std::string model_name = "../robot/robot_model/unitree_go1/scene.xml";
    bool launch_imu = false;
    bool launch_usb2can = false;
    bool launch_rc = true;
    bool unitree = true;
    run_type type_ = real_unitree;
#endif
    auto *robot_ctrl = new My_Controller();
    Eigen::setNbThreads(1);

    HardwareBridge::My_HardwareBridge test_hardware(model_name, robot_ctrl, type_);
    test_hardware.setup_HardwareBridge(launch_imu, launch_usb2can, launch_rc, unitree);
    return 0;
}
