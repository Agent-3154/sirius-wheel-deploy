//
// Created by lingwei on 4/30/24.
//
#include "Robot_Runner.h"
#include "../simulator/include/array_safety.h"
#include "../utilities/types/std_cout_colors.h"
#include "../estimators/KalmanFilterEstimator.h"
#include "../estimators/OrientationEstimator.h"
#include "../../utilities/inc/utilities_fun.h"
#include "../../utilities/inc/debug_tools.h"

RobotRunner::RobotRunner(std::string &model_name, Robot_Controller_Base *control_base, run_type sim)
    : robot_ctrl_(control_base), sim_(sim), lcm_leg_cmd_(getLcmUrl(255)), lcm_leg_data_(getLcmUrl(255)),
      lcm_leg_esti_(getLcmUrl(255)), runner_timer_(0, 2000), lcm_cmd_receive_(getLcmUrl(255)),
      lcm_data_publish_(getLcmUrl(255)) {
    Load_Model(model_name);
    syn_bool_.store(false);
}

void RobotRunner::lcm_handle_func() {
    while (true) {
        lcm_cmd_receive_.handle();
    }
}

/**
 * @note Call this after constructed in hardwarebridge
 */
void RobotRunner::init_robotrunner() {
    quadruped_model_ = new Quadruped_Base(mnew);
    leg_controller_ = new Leg_Controller<double>(quadruped_model_);
    estimators_ = new StateEstimatorContainer<double>(&state_esti_ouput_, runner_imudata_, leg_controller_->leg_data,
                                                      quadruped_model_);
    // TODO Add Contact Estimator

    // important: set contact phase
    Vec4<double> init_contact_phase;
    init_contact_phase << 0.5, 0.5, 0.5, 0.5;
    estimators_->setContactPhase(init_contact_phase);
    // this file path is related with script
    estimators_->addEstimator<Estimators::UsbImuOrientationEstimator<double> >(
        Config::path_2_config_directory + "config/Estimators.info");
    estimators_->addEstimator<Estimators::LinearKFPositionVelocityEsitmator<double> >(
        Config::path_2_config_directory + "config/Estimators.info");

    // assign address to robot ctrl
    robot_ctrl_->quadruped_model_ = quadruped_model_;
    robot_ctrl_->leg_controller_ = leg_controller_;
    robot_ctrl_->estimators_ = estimators_;
    robot_ctrl_->state_esti_ouput_ = &state_esti_ouput_;
    robot_ctrl_->ctrl_rc_ = runner_rc_;

    robot_ctrl_->Controller_Init();

    std::cout << GREEN << "[LCM SUCCESS]: " << RESET << " Start subcribe upper cmd!\n";
    lcm_cmd_receive_.subscribe("ROS_CTRL", &RobotRunner::handleRosCMD, this);
    thread_ptr = std::make_unique<std::thread>(&RobotRunner::lcm_handle_func, this);
}

void RobotRunner::setupStep() {
    if (runner_rc_->rc_control_.mode != usb_controller::RC_MODE::EXTERNAL) {
        leg_controller_->Update_Data(runner_usbdata_);
    } else {
        // std::lock_guard<std::mutex> lk(runner_usb2can_->usb_in_mutex);
        // for (int i = 0; i < 4; i++) {
        //     low_state_data_.q[3 * i] = runner_usbdata_->q_abad[i];
        //     low_state_data_.q[3 * i + 1] = runner_usbdata_->q_hip[i];
        //     low_state_data_.q[3 * i + 2] = runner_usbdata_->q_knee[i];
        //     low_state_data_.qd[3 * i] = runner_usbdata_->qd_abad[i];
        //     low_state_data_.qd[3 * i + 1] = runner_usbdata_->qd_hip[i];
        //     low_state_data_.qd[3 * i + 2] = runner_usbdata_->qd_knee[i];
        //     low_state_data_.tauIq[3 * i] = runner_usbdata_->tau_abad[i];
        //     low_state_data_.tauIq[3 * i + 1] = runner_usbdata_->tau_hip[i];
        //     low_state_data_.tauIq[3 * i + 2] = runner_usbdata_->tau_knee[i];
        // }
        // lcm_data_publish_.publish("CTRL_DATA", &low_state_data_);
    }
}

void RobotRunner::run() {
    // if (runner_rc_->rc_control_.mode != usb_controller::RC_MODE::EXTERNAL) {
    estimators_->run_estimators();
    // } else {
    // }
    setupStep();
    // TODO run controller here
    robot_ctrl_->run();
    finalStep();
    // }
    // runner_timer_.timer_exit(3);
}

void RobotRunner::finalStep() {
    // runner_timer_.timer_record();
    if (sim_ == real_usb) {
        std::lock_guard<std::mutex> lk(runner_usb2can_->usb_out_mutex);
        leg_controller_->Setup_Command(runner_usbcmd_);
    } else if (sim_ == sim_mj || sim_ == sim_embedded_in_other) {
        std::lock_guard<std::mutex> lk(sim_mtx);
        leg_controller_->Setup_Command(runner_usbcmd_);
    }
    leg_controller_->Setup_Command(runner_usbcmd_);
    syn_bool_.store(true);
    leg_controller_->setLcm(&lcm_leg_control_data, &lcm_leg_control_cmd);
    state_esti_ouput_.setLcm(lcm_state_estimate);
    lcm_leg_cmd_.publish("LEG_COMMAND_CHANNEL", &lcm_leg_control_cmd);
    lcm_leg_data_.publish("LEG_DATA_CHANNEL", &lcm_leg_control_data);
    lcm_leg_esti_.publish("STATE_ESTI_CHANNEL", &lcm_state_estimate);
    // runner_timer_.timer_exit(5);
}

void RobotRunner::handleRosCMD(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                               const ros_lowcmd_lcmt *msg) {
    (void) rbuf;
    (void) chan;
    if (runner_rc_->rc_control_.mode == usb_controller::RC_MODE::EXTERNAL) {
        memcpy(&low_cmd_, msg, sizeof(low_cmd_));
        for (int i = 0; i < 4; i++) {
            leg_controller_->leg_command[i].q_des(0) = static_cast<double>(low_cmd_.q_des[3 * i]);
            leg_controller_->leg_command[i].q_des(1) = static_cast<double>(low_cmd_.q_des[3 * i + 1]);
            leg_controller_->leg_command[i].q_des(2) = static_cast<double>(low_cmd_.q_des[3 * i + 2]);

            leg_controller_->leg_command[i].qd_des(0) = static_cast<double>(low_cmd_.qd_des[3 * i]);
            leg_controller_->leg_command[i].qd_des(1) = static_cast<double>(low_cmd_.qd_des[3 * i + 1]);
            leg_controller_->leg_command[i].qd_des(2) = static_cast<double>(low_cmd_.qd_des[3 * i + 2]);

            leg_controller_->leg_command[i].tau_ff(0) = static_cast<double>(low_cmd_.tau_ff[3 * i]);
            leg_controller_->leg_command[i].tau_ff(1) = static_cast<double>(low_cmd_.tau_ff[3 * i + 1]);
            leg_controller_->leg_command[i].tau_ff(2) = static_cast<double>(low_cmd_.tau_ff[3 * i + 2]);

            leg_controller_->leg_command[i].kp_joint(0, 0) = static_cast<double>(low_cmd_.kp_joint[3 * i]);
            leg_controller_->leg_command[i].kp_joint(1, 1) = static_cast<double>(low_cmd_.kp_joint[3 * i + 1]);
            leg_controller_->leg_command[i].kp_joint(2, 2) = static_cast<double>(low_cmd_.kp_joint[3 * i + 2]);

            leg_controller_->leg_command[i].kd_joint(0, 0) = static_cast<double>(low_cmd_.kd_joint[3 * i]);
            leg_controller_->leg_command[i].kd_joint(1, 1) = static_cast<double>(low_cmd_.kd_joint[3 * i + 1]);
            leg_controller_->leg_command[i].kd_joint(2, 2) = static_cast<double>(low_cmd_.kd_joint[3 * i + 2]);
        }
    }
}

void RobotRunner::Load_Model(std::string &model_name) {
    // load mujoco model
    char filename[1000];
    mujoco::utils::strcpy_arr(filename, model_name.c_str());
    // make sure filename is not empty
    if (!filename[0]) {
        std::cout << RED << "[Error]: " << RESET << "Robot runner file empty!\n";
        std::abort();
    }
    // load and compile
    char loadError[512] = "";
    if (mujoco::utils::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mujoco::utils::strlen_arr(filename) - 4, ".mjb",
                      mujoco::utils::sizeof_arr(filename) - mujoco::utils::strlen_arr(filename) + 4)) {
        mnew = mj_loadModel(filename, nullptr);
        if (!mnew) {
            mujoco::utils::strcpy_arr(loadError, "could not load binary model");
        }
    } else {
        mnew = mj_loadXML(filename, nullptr, loadError, 1000);
        // remove trailing newline character from loadError
        if (loadError[0]) {
            int error_length = mujoco::utils::strlen_arr(loadError);
            if (loadError[error_length - 1] == '\n') {
                loadError[error_length - 1] = '\0';
            }
        }
        // add build algorithm model
    }
    if (!mnew) {
        std::printf("%s\n", loadError);
        std::abort();
    }
}
