#include <string>
#include <cstdlib>
#include <iostream>
#include "../simulator/SimulationBridge.h"
#include "../config/Config.h"

int main(int argc, char **argv) {
    // Select model based on environment variable
    // Usage: export SIRIUS_MODEL=new (or leave unset for default)
    const char* env_model = std::getenv("SIRIUS_MODEL");
    std::string model_name;
    
    if (env_model != nullptr && std::string(env_model) == "wheel") {
        model_name = "../robot/robot_model/sirius_wheel_new/scene.xml";
        std::cout << "Using Wheel model: " << model_name << std::endl;
    } else {
        model_name = "../robot/robot_model/ly-mid-p-0916/scene.xml";
        std::cout << "Using Point-Foot model: " << model_name << std::endl;
    }
    
    bool b_sub_real_imu = false;
    bool b_sub_usb2can = false;

    iox::runtime::PoshRuntime::initRuntime("Simulation_Node");
    Simulation::SimulationBridge sim_test("Thread Sim", Config::sim_task_fre, model_name, Config::sim_mj);
    // start simulation UI loop (blocking call)
    std::cout << "run here\n";
    sim_test.setup_simulation_bridge(b_sub_real_imu, b_sub_usb2can);

    return 0;
}
