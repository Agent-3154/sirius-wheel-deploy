#include <new>
#include <string>
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
#endif
    bool b_sub_real_imu = false;
    bool b_sub_usb2can = false;

    iox::runtime::PoshRuntime::initRuntime("Simulation_Node");
    Simulation::SimulationBridge sim_test("Thread Sim", Config::sim_task_fre, model_name, Config::sim_mj);
    // start simulation UI loop (blocking call)
    std::cout << "run here\n";
    sim_test.setup_simulation_bridge(b_sub_real_imu, b_sub_usb2can);

    return 0;
}
