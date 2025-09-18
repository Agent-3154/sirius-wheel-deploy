#include <new>
#include <string>
#include "../simulator/SimulationBridge.h"
#include "../robot_ctrl/my_controller.h"
#include "../config/Config.h"

int main(int argc, char **argv) {
    // print version, check compatibility
    std::string model_name = "../robot/robot_model/sirius_wheel_new/scene.xml";
    bool b_sub_real_imu = false;
    bool b_sub_usb2can = false;

    iox::runtime::PoshRuntime::initRuntime("Simulation_Node");
    Simulation::SimulationBridge sim_test("Thread Sim", Config::sim_task_fre, model_name, Config::sim_mj);
    // start simulation UI loop (blocking call)
    std::cout << "run here\n";
    sim_test.setup_simulation_bridge(b_sub_real_imu, b_sub_usb2can);

    return 0;
}
