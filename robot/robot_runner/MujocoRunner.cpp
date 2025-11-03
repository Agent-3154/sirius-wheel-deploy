#include "Robot_Runner.h"


MujocoRunner::MujocoRunner(std::string &model_name, Robot_Controller_Base *control_base, Config::run_type sim)
    : RobotRunner(model_name, control_base, sim)
      , sim_state_subscriber({"Robot", "SIM", "State"}),
      sim_motor_publisher({"Robot", "SIM", "Motor"})
{
    syn_bool_.store(false);
    robot_runner_timer_ = std::make_shared<Thread::thread_timer>("Robot Runner", 2000);
}

void MujocoRunner::init_robotrunner() {
    RobotRunner::init_robotrunner();
    thread_subscriber_ = std::thread(&MujocoRunner::thread_subscriber_function, this);
}

void MujocoRunner::setupStep() {
    std::lock_guard<std::mutex> lk(sim_mtx);
    leg_controller_->Update_Data(runner_usbdata_);
}

void MujocoRunner::finalStep() {
    std::lock_guard<std::mutex> lk(sim_mtx);
    leg_controller_->Setup_Command(runner_usbcmd_);
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
}

void MujocoRunner::thread_subscriber_function() {
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