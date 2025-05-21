//
// Created by lingwei on 12/6/24.
//
#include <new>
#include <string>
#include "mujoco/mujoco.h"
#include "../simulator/my_simulator.h"
#include "../simulator/SimulationBridge.h"
#include "../robot_ctrl/my_controller.h"
#include "../config/Config.h"

int main(int argc, char **argv) {
    // print version, check compatibility

#if defined BELT
    std::string model_name = "../robot/robot_model/belt/scene.xml";
#elif defined CHAOJI_GO
    std::string model_name = "../robot/robot_model/chaojigou/scene.xml";
#elif defined GO1
    std::string model_name = "../robot/robot_model/unitree_go1/scene.xml";
#endif
    auto *robot_ctrl = new My_Controller();
    bool b_sub_real_imu = true;
    bool b_sub_usb2can = true;

    // Simulation::SimulationBridge real_robot_show("Thread Show Robot", Config::sim_task_fre, model_name, robot_ctrl, Config::sim_show);
    // start simulation UI loop (blocking call)
    // real_robot_show.setup_simulation_bridge(b_sub_real_imu, b_sub_usb2can);

    return 0;
}
