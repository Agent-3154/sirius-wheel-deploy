//
// Created by lingwei on 4/30/24.
//
#include "Robot_Runner.h"
#include <memory>
#include "../estimators/OrientationEstimator.h"
#include "../../utilities/inc/utilities_fun.h"
#include "../../utilities/inc/debug_tools.h"
#include "rerun.hpp"

RobotRunner::RobotRunner(std::string &model_name, Config::run_type sim)
    : sim_(sim), lcm_leg_cmd_(getLcmUrl(255)), lcm_leg_data_(getLcmUrl(255)),
      lcm_leg_esti_(getLcmUrl(255)), runner_timer_(0, 2000), lcm_cmd_receive_(getLcmUrl(255)),
      lcm_data_publish_(getLcmUrl(255))
#if defined(SIMULATOR)
      , sim_state_subscriber({"Robot", "SIM", "State"}),
      sim_motor_publisher({"Robot", "SIM", "Motor"})
#endif
    ,rec_("sirius-wheel", "sirius-wheel")
{
    syn_bool_.store(false);
    
    char error[1024];
    mj_model_ = mj_loadXML(model_name.c_str(), nullptr, error, 1024);
    if (!mj_model_) {
        std::cerr << "Failed to load model: " << error << std::endl;
        std::exit(1);
    }
    mj_data_ = mj_makeData(mj_model_);
    if (!mj_data_) {
        std::cerr << "Failed to make data: " << error << std::endl;
        std::exit(1);
    }
    
    const std::vector<std::string> haa_joints = {
        "RF_HAA",
        "LF_HAA",
        "RH_HAA",
        "LH_HAA"
    };
    
    for (const auto& mj_name : haa_joints) {
        auto id = mj_name2id(mj_model_, mjOBJ_JOINT, mj_name.c_str());
        qpos_addr_[mj_name] = mj_model_->jnt_qposadr[id];
        qvel_addr_[mj_name] = mj_model_->jnt_dofadr[id];
        std::cout << "qpos_addr_[" << mj_name << "]: " << qpos_addr_[mj_name] << std::endl;
        std::cout << "qvel_addr_[" << mj_name << "]: " << qvel_addr_[mj_name] << std::endl;
    }

    auto status = rec_.connect_grpc("rerun+http://127.0.0.1:9876/proxy");
    if (!status.is_ok()) {
        std::cerr << "Failed to connect to rerun server: " << status.description << std::endl;
        std::exit(1);
    }

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

    // // assign address to robot ctrl
    // robot_ctrl_->leg_controller_ = leg_controller_;
    // robot_ctrl_->estimators_ = estimators_;
    // robot_ctrl_->state_esti_ouput_ = &state_esti_ouput_;
    // robot_ctrl_->ctrl_rc_ = runner_rc_;

    // robot_ctrl_->Controller_Init();

    fsm_ = new ControlFSM(runner_rc_, leg_controller_, estimators_);
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
    }
}

void RobotRunner::run_step(int step_count) {
    if (sim_ == Config::real_usb) {
        std::lock_guard<std::mutex> lk(runner_imu_->imu_mtx);
        estimators_->run_estimators();
    } else if (sim_ == Config::sim_mj) {
        estimators_->run_estimators();
    }
    setupStep();

    Eigen::Map<Eigen::VectorXd> qpos_vec(mj_data_->qpos, mj_model_->nq);

    qpos_vec.segment<3>(qpos_addr_["RF_HAA"]) = leg_controller_->leg_data[0].q;
    qpos_vec.segment<3>(qpos_addr_["LF_HAA"]) = leg_controller_->leg_data[1].q;
    qpos_vec.segment<3>(qpos_addr_["RH_HAA"]) = leg_controller_->leg_data[2].q;
    qpos_vec.segment<3>(qpos_addr_["LH_HAA"]) = leg_controller_->leg_data[3].q;

    mj_forward(mj_model_, mj_data_);

    if (step_count % 10 == 0 && rec_.is_enabled()) {

        for (int body = 1; body < mj_model_->nbody; body++) {
            mjtNum xpos[3];
            mjtNum xquat[4];
            int adr_pos = body * 3;
            xpos[0] = mj_data_->xpos[adr_pos];
            xpos[1] = mj_data_->xpos[adr_pos + 1];
            xpos[2] = mj_data_->xpos[adr_pos + 2];

            int adr_quat = body * 4;
            xquat[0] = mj_data_->xquat[adr_quat];
            xquat[1] = mj_data_->xquat[adr_quat + 1];
            xquat[2] = mj_data_->xquat[adr_quat + 2];
            xquat[3] = mj_data_->xquat[adr_quat + 3];
            rec_.log(
                std::string("robot/") + mj_id2name(mj_model_, mjOBJ_BODY, body),
                rerun::Transform3D(
                    rerun::Vec3D(xpos[0], xpos[1], xpos[2]),
                    rerun::Quaternion::from_wxyz(xquat[0], xquat[1], xquat[2], xquat[3])
                )
            );
        }
    }
    fsm_->ControlFSM_run();
    finalStep();
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
