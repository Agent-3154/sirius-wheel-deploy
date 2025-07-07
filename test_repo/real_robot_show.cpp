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
#elif defined DG_ENGINEER
    std::string model_name = "../robot/robot_model/dg_engineer/scene.xml";
#elif defined SIRIUS_WHEEL
    std::string model_name = "../robot/robot_model/sirius_wheel/scene.xml";

    Config::run_type type_ = Config::sim_show;
#endif
    bool launch_imu = true;
    bool launch_usb2can = false;
    bool launch_rc = false;
    bool unitree = false;
#if defined (SIMULATOR)
    iox::runtime::PoshRuntime::initRuntime("Sim_Show_Node");
#endif
    Simulation::SimulationBridge real_robot_show("Thread Show Robot", Config::sim_task_fre, model_name, Config::sim_show);
    // start simulation UI loop (blocking call)
    real_robot_show.setup_simulation_bridge(launch_imu, launch_usb2can);

    return 0;
}
