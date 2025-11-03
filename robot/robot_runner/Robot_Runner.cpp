//
// Created by lingwei on 4/30/24.
//
#include "Robot_Runner.h"
#include <memory>
#include "../estimators/OrientationEstimator.h"
#include "../../utilities/inc/utilities_fun.h"
#include "../../utilities/inc/debug_tools.h"

RobotRunner::RobotRunner(std::string &model_name, Robot_Controller_Base *control_base, Config::run_type sim)
    : robot_ctrl_(control_base), sim_(sim), lcm_leg_cmd_(getLcmUrl(255)), lcm_leg_data_(getLcmUrl(255)),
      lcm_leg_esti_(getLcmUrl(255)), runner_timer_(0, 2000), lcm_cmd_receive_(getLcmUrl(255)),
      lcm_data_publish_(getLcmUrl(255))
{
    syn_bool_.store(false);
}

void RobotRunner::init_robotrunner() {
    leg_controller_ = new Leg_Controller<double>();
    estimators_ = new StateEstimatorContainer<double>(&state_esti_ouput_, runner_imudata_, leg_controller_->leg_data);
    // TODO Add Contact Estimator

    // important: set contact phase
    Vec4<double> init_contact_phase;
    init_contact_phase << 0.5, 0.5, 0.5, 0.5;
    estimators_->setContactPhase(init_contact_phase);
    // this file path is related with script
    estimators_->addEstimator<Estimators::UsbImuOrientationEstimator<double> >(
        Config::path_2_config_directory + "config/Estimators.info");

    // assign address to robot ctrl
    robot_ctrl_->leg_controller_ = leg_controller_;
    robot_ctrl_->estimators_ = estimators_;
    robot_ctrl_->state_esti_ouput_ = &state_esti_ouput_;
    robot_ctrl_->ctrl_rc_ = runner_rc_;

    robot_ctrl_->Controller_Init();
}

void RobotRunner::setupStep() {
    std::shared_lock<std::shared_mutex> usb2can_in_read_lk(runner_usb2can_->usb_shared_in_mutex);
    leg_controller_->Update_Data(runner_usbdata_);
    usb2can_in_read_lk.unlock();
}

void RobotRunner::run_step(int step_count) {
    if (sim_ == Config::real_usb) {
        std::lock_guard<std::mutex> lk(runner_imu_->imu_mtx);
        estimators_->run_estimators();
    } else if (sim_ == Config::sim_mj) {
        estimators_->run_estimators();
    }
    setupStep();
    robot_ctrl_->run();
    finalStep();
}

void RobotRunner::finalStep() {
    // runner_timer_.timer_record();
    if (sim_ == Config::real_usb) {
        std::unique_lock<std::shared_mutex> lk(runner_usb2can_->usb_shared_out_mutex);
        leg_controller_->Setup_Command(runner_usbcmd_);
        lk.unlock();
    }
    if (sim_ == Config::real_ros_ctrl) {
        // std::lock_guard<std::mutex> lk(runner_usb2can_->usb_out_mutex);
        // leg_controller_->Setup_Command(runner_usbcmd_);
    } else {
        syn_bool_.store(true);
        leg_controller_->setLcm(&lcm_leg_control_data, &lcm_leg_control_cmd);
        state_esti_ouput_.setLcm(lcm_state_estimate);
        lcm_leg_cmd_.publish("LEG_COMMAND_CHANNEL", &lcm_leg_control_cmd);
        lcm_leg_data_.publish("LEG_DATA_CHANNEL", &lcm_leg_control_data);
        lcm_leg_esti_.publish("STATE_ESTI_CHANNEL", &lcm_state_estimate);
    }
    // runner_timer_.timer_exit(5);
}
