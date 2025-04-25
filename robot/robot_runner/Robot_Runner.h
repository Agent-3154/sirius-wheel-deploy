//
// Created by lingwei on 4/30/24.
//

#ifndef MY_MUJOCO_SIMULATOR_ROBOT_RUNNER_H
#define MY_MUJOCO_SIMULATOR_ROBOT_RUNNER_H

#include "../hardwares/usb/include/rt_usb_imu.h"
#include "../hardwares/usb/include/rt_usb2can.h"
#include "../hardwares/usb/include/rt_remote_controller.h"
#include "../robot_model/Quadruped_Model_Base.h"
#include "../leg_controller/leg_control.h"
#include "../estimators/Estimator_Base.h"
#include "../../lcm-types/cpp/leg_control_command_lcmt.hpp"
#include "../../lcm-types/cpp/leg_control_data_lcmt.hpp"
#include "../../lcm-types/cpp/state_estimator_lcmt.hpp"
#include "../../lcm-types/cpp/ros_lowcmd_lcmt.hpp"
#include "../../lcm-types/cpp/ros_lowstate_lcmt.hpp"
#include "../../robot_ctrl/robot_ctrl_base.h"
#include <atomic>

#include "../../utilities/inc/debug_tools.h"
#include "../hardwares/fdsc_utils/my_fdsc.h"

enum run_type {
    real_usb = 0, // real_ctrl_byusb
    real_unitree, //real_ctrl_go1
    real_ros_ctrl,
    sim_show, // syn imu and motor datas with real robot
    sim_mj, // sim in mujoco
    sim_lcm,
    sim_embedded_in_other// sim in ros or cheetah
};

class RobotRunner {
public:
    explicit RobotRunner(std::string &model_name, Robot_Controller_Base *control_base, run_type sim_real);

    void lcm_handle_func();

    ~RobotRunner() = default;

    usb_controller::logic_remote_controller *runner_rc_ = nullptr;
    USB_HARDWARE::Motor_Control_Board *runner_usb2can_ = nullptr;
    USB_HARDWARE::USB_IMU *runner_imu_ = nullptr;
    My_FDSC *runner_fdsc_ = nullptr;
    USB_Data_t *runner_usbdata_ = nullptr;
    USB_Command_t *runner_usbcmd_ = nullptr;
    USB_Imu_t *runner_imudata_ = nullptr;

    void init_robotrunner();

    void run();

    void Load_Model(std::string &model_name);

    std::mutex sim_mtx; //for sim

    //    std::atomic_bool ato_print_data_ = false;
    std::array<double, 7> groud_truth_q{};
    std::array<double, 6> ground_truth_qd_{};
    Quadruped_Base *quadruped_model_ = nullptr;

    Robot_Controller_Base *robot_ctrl_ = nullptr;

    Leg_Controller<double> *leg_controller_ = nullptr;
    StateEstimateOutput<double> state_esti_ouput_;
    StateEstimatorContainer<double> *estimators_ = nullptr;
    // this mjModel is used for initiate
    mjModel *mnew = nullptr;

    void setupStep();

    void finalStep();

    // lcm types
    state_estimator_lcmt lcm_state_estimate{};
    leg_control_command_lcmt lcm_leg_control_cmd{};
    leg_control_data_lcmt lcm_leg_control_data{};
    run_type sim_;
    lcm::LCM lcm_leg_cmd_;
    lcm::LCM lcm_leg_data_;
    lcm::LCM lcm_leg_esti_;
    std::atomic<bool> syn_bool_{}; // used for syning with sim loop.
    Debugging::test_timer runner_timer_;
    std::unique_ptr<std::thread> thread_ptr;

    lcm::LCM lcm_cmd_receive_;
    lcm::LCM lcm_data_publish_;
    ros_lowstate_lcmt low_state_data_;
    ros_lowcmd_lcmt low_cmd_;

    void handleRosCMD(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const ros_lowcmd_lcmt *msg);
};

#endif //MY_MUJOCO_SIMULATOR_ROBOT_RUNNER_H
