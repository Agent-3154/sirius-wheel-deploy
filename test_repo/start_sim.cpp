#include <new>
#include <string>
#include "../simulator/SimulationBridge.h"
#include "../robot_ctrl/my_controller.h"
#include "../config/Config.h"
#include "../utilities/inc/easylogging++.h"

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
#endif
    auto *robot_ctrl = new My_Controller();
    bool use_rc = true;
    bool b_sub_real_imu = false;
    bool b_sub_usb2can = false;

    // el::Configurations conf("my_conf.conf");
    // el::Loggers::reconfigureAllLoggers(conf);
    el::Loggers::addFlag(el::LoggingFlag::ColoredTerminalOutput);

    Simulation::SimulationBridge sim_test("Thread Sim", Config::sim_task_fre, model_name, robot_ctrl, sim_mj);
    // start simulation UI loop (blocking call)
    sim_test.setup_simulation_bridge(use_rc, b_sub_real_imu, b_sub_usb2can);

    return 0;
}
