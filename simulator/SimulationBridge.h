//
// Created by lingwei on 5/10/24.
//

#ifndef MY_MUJOCO_SIMULATOR_SIMULATIONBRIDGE_H
#define MY_MUJOCO_SIMULATOR_SIMULATIONBRIDGE_H

#include "../simulator/my_simulator.h"
#include "include/array_safety.h"
#include "../utilities/types/hardware_types.h"
#include "../utilities/inc/thread_timer.h"
#include "motor_control/my_motor_model.h"
#include "lcm/lcm-cpp.hpp"
#include "../lcm-types/cpp/sim_ground_truth_lcmt.hpp"
#include <random>

#include "../lcm-types/cpp/mpc_lcmt.hpp"
#include "../lcm-types/cpp/planner_lcmt.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iox/signal_watcher.hpp"
#include "../quadruped_share_data/robot_state_protocols.h"

namespace Simulation {
    namespace mj = ::mujoco;
    namespace mju = ::mujoco::utils;
    // constants
    const double syncMisalign = 0.1; // maximum mis-alignment before re-sync (simulation seconds)
    const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
    const int kErrorLength = 1024; // load error string length

    using Seconds = std::chrono::duration<double>;

    class SimulationBridge : public Thread::thread_timer {
    public:
        SimulationBridge(const std::string &task_name, int task_frequency, std::string &model_name,
                         Config::run_type _sim_type);

        ~SimulationBridge() override;

        void setup_simulation_bridge(bool real_imu, bool real_control);

        mjModel *LoadModel();

        void PhysicsThread();

        void PhysicsLoop();

        void sim_step();

        void sim_control();

        void sim_show_step();

    private:
        void set_lcm();

        // model and data
        mjModel *m_ = nullptr;
        mjData *d_ = nullptr;
        // control noise variables
        mjtNum *ctrlnoise = nullptr;
        std::unique_ptr<mj::Simulate> sim_handle_;

        USB_Command_t *motor_cmd_;
        USB_Data_t *motor_data_;
        USB_Imu_t *usb_imu_;
        Vec4<double> noise_quat;
        Vec3<double> gyro_;
        Vec3<double> acc_;

        // parameter for sim control
        bool runner_started = false;
        bool dynamic_printed = false;
        bool real_control_ = false;

        // sim thread starts with std::thread.
        std::thread thread_robot_runner_;
        std::thread thread_rc_;

        // std::shared_ptr<Utilities::ThreadPool> tp_rc_;
        // std::shared_ptr<Utilities::ThreadPool> tp_robot_runner_;
        std::thread physics_handle_;
        std::string model_name_;

        Motor_Control::Motor_Model *motors_{};

        lcm::LCM lcm_;
        sim_ground_truth_lcmt sim_ground_truth{};

        // add gaussian noise;
        std::default_random_engine generator;
        std::normal_distribution<double> dist_acc_;
        std::normal_distribution<double> dist_gyro_;
        std::normal_distribution<double> dist_quat_;
        std::normal_distribution<double> dist_encoder_;
        std::normal_distribution<double> dist_encoder_vel_;

        // mocap parameter; Note: this order should be the same as in model.xml
        int mocap_esti_indicator_id_ = 0;
        // foot trajectory drawing
        mjtNum foot_pos_des_[4][3]{};
        mjtNum foot_pos_last_des_[4][3]{};
        mjtNum foot_pos_[4][3]{};
        mjtNum foot_pos_last_[4][3]{};
        mjtNum pos_des_[3]{};
        mjtNum pos_last_des_[3]{};
        mjtNum pos_[3]{};
        mjtNum pos_last_[3]{};
        mjtNum pf_init_[4][3]{};


        void foot_traject_lcm_handle(const lcm::ReceiveBuffer *rbuf,
                                     const std::string &chan,
                                     const planner_lcmt *msg);

        void mpc_force_lcm_handle(const lcm::ReceiveBuffer *rbuf,
                                  const std::string &chan,
                                  const mpc_lcmt *msg);

        lcm::LCM usb_2_can_LCM_;
        usb_data_t sim_local_usbdata{};

        void USB_DATA_LCM_HANDLE(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const usb_data_t *msg);

        lcm::LCM usb_imu_LCM_;
        imu_lcmt sim_local_imudata_{};

        void USB_IMU_LCM_HANDLE(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const imu_lcmt *msg);

        Config::run_type sim_;

        iox::popo::Subscriber<Robot_Control_Motor_Cmd> subscriber;
        iox::popo::Publisher<Robot_State> publisher;
        iox::popo::Subscriber<Sim_Plot> plot_subscriber_;
    };
}

#endif //MY_MUJOCO_SIMULATOR_SIMULATIONBRIDGE_H
