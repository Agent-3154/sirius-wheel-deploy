//
// Created by lingwei on 4/30/24.
//

#ifndef MY_MUJOCO_SIMULATOR_ROBOT_RUNNER_H
#define MY_MUJOCO_SIMULATOR_ROBOT_RUNNER_H

#include "../hardwares/usb/include/rt_usb_imu.h"
#include "../hardwares/usb/include/rt_usb2can.h"
#include "../hardwares/usb/include/rt_remote_controller.h"
#include "../leg_controller/leg_control.h"
#include "../estimators/Estimator_Base.h"
#include "../../lcm-types/cpp/leg_control_command_lcmt.hpp"
#include "../../lcm-types/cpp/leg_control_data_lcmt.hpp"
#include "../../lcm-types/cpp/state_estimator_lcmt.hpp"
#include "../../lcm-types/cpp/ros_lowcmd_lcmt.hpp"
#include "../../lcm-types/cpp/ros_lowstate_lcmt.hpp"
#include "../../robot_ctrl/robot_ctrl_base.h"
#include <atomic>
#include "../../config/Config.h"

#include "../../utilities/inc/debug_tools.h"

#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iox/signal_watcher.hpp"

#include "../../utilities/inc/thread_timer.h"
#include "../../quadruped_share_data/robot_state_protocols.h"

#include <mujoco/mujoco.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjdata.h>

class RobotRunner
{
public:
    explicit RobotRunner(std::string &model_name, Robot_Controller_Base *control_base, Config::run_type sim_real);

    ~RobotRunner() = default;

    usb_controller::LogicRemoteController *runner_rc_ = nullptr;
    USB_HARDWARE::Beast_USB2CAN *runner_usb2can_ = nullptr;
    USB_HARDWARE::USB_IMU *runner_imu_ = nullptr;
    USB_Data_t *runner_usbdata_ = nullptr;
    USB_Command_t *runner_usbcmd_ = nullptr;
    USB_Imu_t *runner_imudata_ = nullptr;

    virtual void init_robotrunner();
    void run_step(int step_count);
    virtual void setupStep();
    virtual void finalStep();

    std::mutex sim_mtx; // for sim

    Robot_Controller_Base *robot_ctrl_ = nullptr;
    mjModel *model_ = nullptr;
    mjData *data_ = nullptr;

    Leg_Controller<double> *leg_controller_ = nullptr;
    StateEstimateOutput<double> state_esti_ouput_;
    StateEstimatorContainer<double> *estimators_ = nullptr;

    // lcm types
    state_estimator_lcmt lcm_state_estimate{};
    leg_control_command_lcmt lcm_leg_control_cmd{};
    leg_control_data_lcmt lcm_leg_control_data{};
    Config::run_type sim_;
    lcm::LCM lcm_leg_cmd_;
    lcm::LCM lcm_leg_data_;
    lcm::LCM lcm_leg_esti_;
    std::atomic<bool> syn_bool_{}; // used for syning with sim loop.
    Debugging::test_timer runner_timer_;
    std::unique_ptr<std::thread> thread_ptr;

    lcm::LCM lcm_cmd_receive_;
    lcm::LCM lcm_data_publish_;
    ros_lowstate_lcmt low_state_data_{};
    ros_lowcmd_lcmt low_cmd_{};

    void handleRosCMD(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const ros_lowcmd_lcmt *msg);

#if defined(SIMULATOR)
    
#endif

private:
    // Helper methods for data handling
    void updateDataFromUSB();
    void updateDataFromSimulation();
    void runEstimatorsWithLock();
    
    // Helper methods for command handling
    void sendCommandsToUSB();
    void sendCommandsToSimulation();
    void sendCommandsToLCM();
    void publishData();
};

class MujocoRunner : public RobotRunner {
public:
    explicit MujocoRunner(std::string &model_name, Robot_Controller_Base *control_base, Config::run_type sim);
    ~MujocoRunner() = default;

    void init_robotrunner() override;
    void finalStep() override;
    void setupStep() override;

    iox::popo::Subscriber<Robot_State> sim_state_subscriber;
    iox::popo::Publisher<Robot_Control_Motor_Cmd> sim_motor_publisher;
    std::thread thread_subscriber_;
    std::shared_ptr<Thread::thread_timer> robot_runner_timer_;
    void thread_subscriber_function();
    void publishMotorCommands();
    void copyLegMotorCommands(auto &sample);
    void copyWheelMotorCommands(auto &sample);
    void copyImuData(auto &sample);
    void copyLegData(auto &sample);
    void copyWheelData(auto &sample);
};

class RealRobotRunner : public RobotRunner {
public:
    explicit RealRobotRunner(std::string &model_name, Robot_Controller_Base *control_base, Config::run_type sim);
    ~RealRobotRunner() = default;
};

#endif // MY_MUJOCO_SIMULATOR_ROBOT_RUNNER_H
