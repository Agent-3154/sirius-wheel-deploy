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
#if defined(SIMULATOR)
      , sim_state_subscriber({"Robot", "SIM", "State"}),
      sim_motor_publisher({"Robot", "SIM", "Motor"})
#endif
{
    syn_bool_.store(false);
#if defined(SIMULATOR)
    robot_runner_timer_ = std::make_shared<Thread::thread_timer>("Robot Runner", 2000);
#endif

}

// void RobotRunner::lcm_handle_func() {
// while (true) {
// lcm_cmd_receive_.handle();
// }
// }

/**
 * @note Call this after constructed in hardwarebridge
 */
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

    // if (sim_ == Config::real_ros_ctrl) {
    // std::cout << GREEN << "[LCM SUCCESS]: " << RESET << " Start subcribe upper cmd!\n";
    // lcm_cmd_receive_.subscribe("ROS_CTRL", &RobotRunner::handleRosCMD, this);
    // thread_ptr = std::make_unique<std::thread>(&RobotRunner::lcm_handle_func, this);
#if defined(SIMULATOR)
    if (sim_ == Config::sim_mj) {
        thread_subscriber_ = std::thread(&RobotRunner::thread_subscriber_function, this);
    }
#endif
}

void RobotRunner::setupStep() {
    if (sim_ == Config::real_usb) {
        std::shared_lock<std::shared_mutex> usb2can_in_read_lk(runner_usb2can_->usb_shared_in_mutex);
        leg_controller_->Update_Data(runner_usbdata_);
        usb2can_in_read_lk.unlock();
    } else if (sim_ == Config::sim_mj) {
        std::lock_guard<std::mutex> lk(sim_mtx);
        leg_controller_->Update_Data(runner_usbdata_);
    } else {
        // lcm_state_.handleTimeout(0);
        // for (int i = 0; i < 4; i++) {
        //     runner_usbdata_->q_abad[i] = lowstate_data_.q[3 * i];
        //     runner_usbdata_->q_hip[i] = lowstate_data_.q[3 * i + 1];
        //     runner_usbdata_->q_knee[i] = lowstate_data_.q[3 * i + 2];
        //     runner_usbdata_->qd_abad[i] = lowstate_data_.qd[3 * i];
        //     runner_usbdata_->qd_hip[i] = lowstate_data_.qd[3 * i + 1];
        //     runner_usbdata_->qd_knee[i] = lowstate_data_.qd[3 * i + 2];
        //     runner_imudata_->q[i] = lowstate_data_.quat[i];
        // }
        // for (int j = 0; j < 3; j++) {
        //     runner_imudata_->gyro[j] = lowstate_data_.omegaBody[j];
        //     runner_imudata_->accel[j] = lowstate_data_.aBody[j];
        // }
        std::lock_guard<std::mutex> lk(sim_mtx);
        leg_controller_->Update_Data(runner_usbdata_);
    }
}

void RobotRunner::run() {
    // run estimators
    // runner_timer_.timer_record();
    // if (sim_ != sim_embedded_in_other) {
    if (sim_ == Config::real_usb) {
        std::lock_guard<std::mutex> lk(runner_imu_->imu_mtx);
        estimators_->run_estimators();
    } else if (sim_ == Config::sim_mj) {
        estimators_->run_estimators();
        // for (int i = 0; i < 3; i++) {
        // this->estimators_->shared_esti_data_.result_->p_w_(i) = groud_truth_q[i];
        // this->estimators_->shared_esti_data_.result_->v_w_(i) = ground_truth_qd_[i];
        // this->estimators_->shared_esti_data_.result_->omega_w_(i) = ground_truth_qd_[i + 3];
        // this->estimators_->shared_esti_data_.result_->q_ori_(i) = groud_truth_q[i + 3];
        // }
        // this->estimators_->shared_esti_data_.result_->q_ori_(3) = groud_truth_q[6];
        // this->estimators_->shared_esti_data_.result_->r_b_ = ori::quaternionToRotationMatrix(
        // this->estimators_->shared_esti_data_.result_->q_ori_);
    }
    setupStep();
    // runner_timer_.timer_exit(1);
    // TODO run controller here
    // runner_timer_.timer_record();
    robot_ctrl_->run();
    // runner_timer_.timer_exit(2);
    // runner_timer_.timer_record();
    finalStep();
    // }
    // runner_timer_.timer_exit(3);
}

void RobotRunner::finalStep() {
    // runner_timer_.timer_record();
    if (sim_ == Config::real_usb) {
        std::unique_lock<std::shared_mutex> lk(runner_usb2can_->usb_shared_out_mutex);
        leg_controller_->Setup_Command(runner_usbcmd_);
        lk.unlock();
    } else if (sim_ == Config::sim_mj) {
        std::lock_guard<std::mutex> lk(sim_mtx);
        leg_controller_->Setup_Command(runner_usbcmd_);
#if defined(SIMULATOR)
        sim_motor_publisher.loan().and_then([this](auto &sample) {
            for (int i = 0; i < 4; i++) {
                const int index = i / 2;
                const int index_shift = index * 2;
                // std::cout << "index: " << index << " | index_shift: " << index_shift << std::endl;
                sample->q[3 * i] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift)].q_des;
                sample->q[3 * i + 1] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].q_des;
                sample->q[3 * i + 2] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].q_des;
                sample->qd[3 * i] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift)].qd_des;
                sample->qd[3 * i + 1] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].qd_des;
                sample->qd[3 * i + 2] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].qd_des;
                sample->tau_ff[3 * i] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift)].tau_ff;
                sample->tau_ff[3 * i + 1] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].tau_ff;
                sample->tau_ff[3 * i + 2] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].tau_ff;
                sample->kp[3 * i] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift)].kp;
                sample->kp[3 * i + 1] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].kp;
                sample->kp[3 * i + 2] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].kp;
                sample->kd[3 * i] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift)].kd;
                sample->kd[3 * i + 1] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].kd;
                sample->kd[3 * i + 2] = runner_usbcmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].kd;
            }
            for (int i = 0; i < 4; i++) {
                    const int index = 2;
                    const int index_shift = i / 2;
                    sample->q[12 + i] =  runner_usbcmd_->chip_cmds[index].motor_cmds[i + index_shift].q_des;
                    sample->qd[12 + i] =  runner_usbcmd_->chip_cmds[index].motor_cmds[i + index_shift].qd_des;
                    sample->tau_ff[12 + i] =  runner_usbcmd_->chip_cmds[index].motor_cmds[i + index_shift].tau_ff;
                    sample->kp[12 + i] =  runner_usbcmd_->chip_cmds[index].motor_cmds[i + index_shift].kp;
                    sample->kd[12 + i] =  runner_usbcmd_->chip_cmds[index].motor_cmds[i + index_shift].kd;
                    // std::cout <<sample->kd[12 + i]<<std::endl;
            }
            sample.publish();
        }).or_else([](auto &result) {
            std::cerr << "Unable to loan sample, error: " << result << std::endl;
        });
#endif
    } else if (sim_ == Config::sim_lcm) {
        std::lock_guard<std::mutex> lk(sim_mtx);
        leg_controller_->Setup_Command(runner_usbcmd_);
        // for (int i = 0; i < 4; i++) {
        //     lowcmd_.q[3 * i] = runner_usbcmd_->q_des_abad[i];
        //     lowcmd_.q[3 * i + 1] = runner_usbcmd_->q_des_hip[i];
        //     lowcmd_.q[3 * i + 2] = runner_usbcmd_->q_des_knee[i];
        //     lowcmd_.qd[3 * i] = runner_usbcmd_->qd_des_abad[i];
        //     lowcmd_.qd[3 * i + 1] = runner_usbcmd_->qd_des_hip[i];
        //     lowcmd_.qd[3 * i + 2] = runner_usbcmd_->qd_des_knee[i];
        //     lowcmd_.tau_ff[3 * i] = runner_usbcmd_->tau_abad_ff[i];
        //     lowcmd_.tau_ff[3 * i + 1] = runner_usbcmd_->tau_hip_ff[i];
        //     lowcmd_.tau_ff[3 * i + 2] = runner_usbcmd_->tau_knee_ff[i];
        // }
        // lowcmd_.start = 1;
        // lowcmd_.kp = runner_usbcmd_->kp_abad[0];
        // lowcmd_.kd = runner_usbcmd_->kd_abad[0];
        // lcm_cmd_.publish("ROS2cheetah", &lowcmd_);
    }
    // runner_timer_.timer_exit(4);
    // std::cout << "Robot Runner:Finish!\n";
    // runner_timer_.timer_record();
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

// void RobotRunner::handleRosCMD(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
// const ros_lowcmd_lcmt *msg) {
// (void) rbuf;
// (void) chan;
// memcpy(&low_cmd_, msg, sizeof(low_cmd_));
// for (int i = 0; i < 4; i++) {
// leg_controller_->leg_command[i].q_des(0) = static_cast<double>(low_cmd_.q_des[3 * i]);
// leg_controller_->leg_command[i].q_des(1) = static_cast<double>(low_cmd_.q_des[3 * i + 1]);
// leg_controller_->leg_command[i].q_des(2) = static_cast<double>(low_cmd_.q_des[3 * i + 2]);

// leg_controller_->leg_command[i].qd_des(0) = static_cast<double>(low_cmd_.qd_des[3 * i]);
// leg_controller_->leg_command[i].qd_des(1) = static_cast<double>(low_cmd_.qd_des[3 * i + 1]);
// leg_controller_->leg_command[i].qd_des(2) = static_cast<double>(low_cmd_.qd_des[3 * i + 2]);

// leg_controller_->leg_command[i].tau_ff(0) = static_cast<double>(low_cmd_.tau_ff[3 * i]);
// leg_controller_->leg_command[i].tau_ff(1) = static_cast<double>(low_cmd_.tau_ff[3 * i + 1]);
// leg_controller_->leg_command[i].tau_ff(2) = static_cast<double>(low_cmd_.tau_ff[3 * i + 2]);

// leg_controller_->leg_command[i].kp_joint(0, 0) = static_cast<double>(low_cmd_.kp_joint[3 * i]);
// leg_controller_->leg_command[i].kp_joint(1, 1) = static_cast<double>(low_cmd_.kp_joint[3 * i + 1]);
// leg_controller_->leg_command[i].kp_joint(2, 2) = static_cast<double>(low_cmd_.kp_joint[3 * i + 2]);

// leg_controller_->leg_command[i].kd_joint(0, 0) = static_cast<double>(low_cmd_.kd_joint[3 * i]);
// leg_controller_->leg_command[i].kd_joint(1, 1) = static_cast<double>(low_cmd_.kd_joint[3 * i + 1]);
// leg_controller_->leg_command[i].kd_joint(2, 2) = static_cast<double>(low_cmd_.kd_joint[3 * i + 2]);
// }
// }

#if defined(SIMULATOR)
void RobotRunner::thread_subscriber_function() {
    while (true) {
        this->robot_runner_timer_->thread_enter_task();
        sim_state_subscriber.take().and_then([this](auto &sample) {
            for (int i = 0; i < 3; i++) {
                runner_imudata_->accel[i] = sample->acc[i];
                runner_imudata_->gyro[i] = sample->gyro[i];
            }
            for (int i = 0; i < 4; i++) {
                runner_imudata_->q[i] = sample->quat[i];
                const int index = i / 2; // (0,1,2,3)->(0,0,1,1)
                const int index_shift = index * 2; // (0,0,2,2)
                runner_usbdata_->chip_datas[index].motor_datas[3 * (i - index_shift)].q = sample->q[3 * i];
                runner_usbdata_->chip_datas[index].motor_datas[3 * (i - index_shift) + 1].q = sample->q[3 * i + 1];
                runner_usbdata_->chip_datas[index].motor_datas[3 * (i - index_shift) + 2].q = sample->q[3 * i + 2];
                runner_usbdata_->chip_datas[index].motor_datas[3 * (i - index_shift)].qd = sample->qd[3 * i];
                runner_usbdata_->chip_datas[index].motor_datas[3 * (i - index_shift) + 1].qd = sample->qd[3 * i + 1];
                runner_usbdata_->chip_datas[index].motor_datas[3 * (i - index_shift) + 2].qd = sample->qd[3 * i + 2];
            }
            for (int i = 0; i < 4; i++) {
                const int index = 2;
                const int index_shift = i / 2;
                runner_usbdata_->chip_datas[index].motor_datas[i + index_shift].q = sample->q[12 + i];
                runner_usbdata_->chip_datas[index].motor_datas[i + index_shift].qd = sample->qd[12 + i];
                // runner_usbdata_->chip_datas[index].motor_datas[i + index_shift].tau = sample->tau_ff[12 + i];
            }
        }).or_else([](auto &result) {
            if (result != iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE) {
                std::cout << "Error receiving chunk." << std::endl;
            }
        });
        this->robot_runner_timer_->thread_finish_task();
    }
}
#endif
