//
// Created by lingwei on 5/10/24.
//
#include "SimulationBridge.h"
#include "../utilities/types/std_cout_colors.h"
#include "../utilities/inc/utilities_fun.h"
#include <iostream>
#include <thread>
#include <mutex>
#include "glfw_adapter.h"
#include "../config/Config.h"
#include "../utilities/inc/easylogging++.h"

INITIALIZE_EASYLOGGINGPP

Simulation::SimulationBridge::SimulationBridge(const std::string &task_name, int task_frequency, std::string &model_name,
                                               Config::run_type _sim_type) : Thread::thread_timer(task_name, task_frequency, Config::mj_sim_time_spy),
                                                                             model_name_(model_name),
                                                                             lcm_(getLcmUrl(255)),
                                                                             usb_2_can_LCM_(getLcmUrl(255)),
                                                                             usb_imu_LCM_(getLcmUrl(255)),
                                                                             sim_(_sim_type),
                                                                             subscriber({"Robot", "SIM", "Motor"}),
                                                                             publisher({"Robot", "SIM", "State"}),
                                                                             plot_subscriber_(
                                                                                 {"Robot", "Plot", "State"}) {
    std::printf("MuJoCo version %s\n", mj_versionString());
    if (mjVERSION_HEADER != mj_version()) {
        mju_error("Headers and library have different versions");
    }
    motor_cmd_ = new USB_Command_t();
    motor_data_ = new USB_Data_t();
    usb_imu_ = new USB_Imu_t();

    // noise
    std::normal_distribution<double> dist_acc(0, Config::noise_acc);
    dist_acc_ = dist_acc;
    std::normal_distribution<double> dist_gyro(0, Config::noise_gyro);
    dist_gyro_ = dist_gyro;
    std::normal_distribution<double> dist_quat(0, Config::noise_quat);
    dist_quat_ = dist_quat;
    std::normal_distribution<double> dist_enc(0, Config::noise_encoder);
    dist_encoder_ = dist_enc;
    std::normal_distribution<double> dist_encv(0, Config::noise_enc_vel);
    dist_encoder_vel_ = dist_encv;
}

Simulation::SimulationBridge::~SimulationBridge() {
    delete usb_imu_;
    delete motor_cmd_;
    delete motor_data_;
}

/**
 * @brie load mujoco model
 * @param file
 * @param sim
 * @return
 */
mjModel *Simulation::SimulationBridge::LoadModel() {
    char filename[mj::Simulate::kMaxFileNameLength];
    mju::strcpy_arr(filename, model_name_.c_str());
    // make sure filename is not empty
    if (!filename[0]) {
        return nullptr;
    }
    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4)) {
        mnew = mj_loadModel(filename, nullptr);
        if (!mnew) {
            mju::strcpy_arr(loadError, "could not load binary model");
        }
    } else {
        mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
        // remove trailing newline character from loadError
        if (loadError[0]) {
            int error_length = mju::strlen_arr(loadError);
            if (loadError[error_length - 1] == '\n') {
                loadError[error_length - 1] = '\0';
            }
        }

        // add build algorithm model
    }
    mju::strcpy_arr(sim_handle_->load_error, loadError);
    if (!mnew) {
        std::printf("%s\n", loadError);
        return nullptr;
    }
    // compiler warning: print and pause
    if (loadError[0]) {
        // mj_forward() below will print the warning message
        std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
        sim_handle_->run = 0;
    }
    return mnew;
}

/**
 * @brief simulation background
 * @param sim
 * @param filename
 */
void Simulation::SimulationBridge::PhysicsThread() {
    if (!model_name_.empty()) {
        sim_handle_->LoadMessage(model_name_);
        m_ = LoadModel();
        // test idea here
        if (m_ != nullptr) {
            // lock the sim mutex
            const std::unique_lock<std::recursive_mutex> lock(sim_handle_->mtx);
            d_ = mj_makeData(m_);
        }
        if (d_ != nullptr) {
            sim_handle_->Load(m_, d_, model_name_); //copy register to the sim_handle
            // lock the sim mutex
            const std::unique_lock<std::recursive_mutex> lock(sim_handle_->mtx);
            mj_resetDataKeyframe(m_, d_, 0);
            mj_forward(m_, d_);
            // allocate ctrlnoise
            free(ctrlnoise);
            ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m_->nu));
            mju_zero(ctrlnoise, m_->nu);
            // mocap
            // std::cout << RED << mocap_esti_indicator_id_ << std::endl;
            std::cout << GREEN << "[Success]: " << RESET << "Succeed loading model and model data!\n";
        } else {
            std::cout << RED << "[Fail]: " << RESET << "Can not load model or model data!\n";
            sim_handle_->LoadMessageClear();
        }
    }
    //recover to default pos:
    // model info summary
    std::cout << "\n//************************ Model Summary ****************************//\n"
            << " Body Number: " << m_->nbody << std::endl
            << " Vel dof Number: " << m_->nv << std::endl
            << " Input dof Number: " << m_->nu << std::endl
            << " Model Output File Path: ~/scripts/model_file.txt\n"
            << "//************************ End ****************************//";
    mj_printData(m_, d_, "model_file.md");

    // convert full M consume 2us.
    //    Debugging::test_timer timer(1);
    //    timer.timer_record();
    //    mjtNum M_test[324];
    //    mj_fullM(m_,M_test,d_->qM);
    //    timer.timer_exit(1);

    PhysicsLoop();
    // delete everything we allocated
    free(ctrlnoise);
    mj_deleteData(d_);
    mj_deleteModel(m_);
}

void Simulation::SimulationBridge::PhysicsLoop() {
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;
    // run until asked to exit
    // note: this loop consumes about 1100us
    //    timer.timer_record();
    while (!sim_handle_->exitrequest.load()) {
        // if (sim_handle_->pending_.start_runner & (!runner_started)) {
        // tp_robot_runner_->Schedule([this]() { t_robot_runner_->thread_loop(robot_runner_); });
        // thread_robot_runner_ = std::thread(&SimulationBridge::thread_robot_runner_function, this);
        // LOG(INFO) << "[Robot Runner]: Start!";
        // sim_handle_->pending_.start_runner = false;
        // runner_started = true;
        // }
        // if (robot_runner_->syn_bool_.load()) {
        // robot_runner_->syn_bool_.store(false);
        // sleep for 1 ms or yield, to let main thread run
        //  yield results in busy wait - which has better timing but kills battery life
        if (sim_handle_->run && sim_handle_->busywait) {
            std::cout << RED << "Busy!\n" << RESET;
            std::this_thread::yield();
        } else {
            //            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        this->thread_enter_task(); {
            // lock the sim mutex
            const std::unique_lock<std::recursive_mutex> lock(sim_handle_->mtx);
            // run only if model is present
            if (m_) {
                // running
                if (sim_handle_->run) {
                    bool stepped = false;
                    // record cpu time at start of iteration
                    const auto startCPU = mj::Simulate::Clock::now();
                    // elapsed CPU and simulation time since last sync
                    const auto elapsedCPU = startCPU - syncCPU;
                    double elapsedSim = d_->time - syncSim;
                    // inject noise
                    if (sim_handle_->ctrl_noise_std) {
                        // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
                        mjtNum rate = mju_exp(-m_->opt.timestep / mju_max(sim_handle_->ctrl_noise_rate, mjMINVAL));
                        mjtNum scale = sim_handle_->ctrl_noise_std * mju_sqrt(1 - rate * rate);
                        for (int i = 0; i < m_->nu; i++) {
                            // update noise
                            ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);
                            // apply noise
                            d_->ctrl[i] = ctrlnoise[i];
                        }
                    }

                    // requested slow-down factor
                    double slowdown = 100 / sim_handle_->percentRealTime[sim_handle_->real_time_index];
                    // misalignment condition: distance from target sim time is bigger than syncmisalign
                    bool misaligned =
                            mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;
                    // out-of-sync (for any reason): reset sync times, step
                    // Note: this condition normally not happens
                    if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                        misaligned || sim_handle_->speed_changed) {
                        std::cerr << "Misaligned\n";
                        // re-sync
                        syncCPU = startCPU;
                        syncSim = d_->time;
                        sim_handle_->speed_changed = false;
                        // run single step, let next iteration deal with timing
                        //                        mj_step(m_, d_);
                        sim_step();
                        stepped = true;
                    }
                    // in-sync: step until ahead of cpu
                    else {
                        bool measured = false;
                        mjtNum prevSim = d_->time;
                        double refreshTime = simRefreshFraction / sim_handle_->refresh_rate;
                        // step while sim lags behind cpu and within refreshTime
                        while (Seconds((d_->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                               mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
                            // measure slowdown before first step
                            if (!measured && (bool) elapsedSim) {
                                sim_handle_->measured_slowdown =
                                        std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                                measured = true;
                            }
                            // call mj_step
                            //                            mj_step(m_, d_);
                            sim_step();
                            stepped = true;
                            // break if reset
                            if (d_->time < prevSim) {
                                break;
                            }
                        }
                    }
                    // save current state to history buffer
                    if (stepped) {
                        sim_handle_->AddToHistory();
                    }
                } // paused
                else {
                    // run mj_forward, to update rendering and joint sliders
                    mj_forward(m_, d_);
                    sim_handle_->speed_changed = true;
                }
            }
        } // release std::lock_guard<std::mutex>
        if (sim_handle_->lcm_pub_) { set_lcm(); }
        if (sim_handle_->pending_.print_dynamic & (!dynamic_printed)) {
            // robot_runner_->quadruped_model_->print_M_C_Matrix();
            std::cout << GREEN << " [Print Data: ]: " << RESET << "OK!\n";
            dynamic_printed = true;
        }
        this->thread_finish_task();
    }
}

// }

void Simulation::SimulationBridge::setup_simulation_bridge(bool real_imu, bool real_control) {
    // initial remote controller if needed
    //! [create subscriber]
    motors_ = new Motor_Control::Motor_Model("../config/Sim_Motor.info");

    //subscrib real imu channel
    if (real_imu) {
        std::cout << GREEN << "[SIM]: " << "Subscribe IMU CHANNEL\n" << RESET;
        usb_imu_LCM_.subscribe("IMU_CHANNEL", &SimulationBridge::USB_IMU_LCM_HANDLE, this);
    }
    real_control_ = real_control;
    if (sim_ == Config::sim_show) {
        std::cout << GREEN << "[SIM]: " << "Subscribe USB DATA Channel\n" << RESET;
        usb_2_can_LCM_.subscribe("MOTOR DATA Serial", &SimulationBridge::USB_DATA_LCM_HANDLE, this);
    }

    sim_handle_ = std::make_unique<
        mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), false, real_imu, real_control);
    // start thread
    physics_handle_ = std::thread(&SimulationBridge::PhysicsThread, this);
    // start simulation UI loop (blocking call)
    sim_handle_->RenderLoop();
    physics_handle_.join();
}

void Simulation::SimulationBridge::sim_step() {
    if (sim_ == Config::sim_mj) {
        mj_checkPos(m_, d_);
        mj_checkVel(m_, d_);
        mj_fwdPosition(m_, d_);
        mj_sensorPos(m_, d_);
        mj_energyPos(m_, d_);
        mj_fwdVelocity(m_, d_);
        mj_sensorVel(m_, d_);
        mj_energyVel(m_, d_);
        sim_control();
        mj_fwdActuation(m_, d_);
        mj_fwdAcceleration(m_, d_);
        mj_fwdConstraint(m_, d_);
        mj_sensorAcc(m_, d_);
        mj_checkAcc(m_, d_);
        // compare forward and inverse solutions if enabled
        if (m_->opt.enableflags & mjENBL_FWDINV) {
            mj_compareFwdInv(m_, d_);
        }
        // integrate with Euler or implicit; RK4 defaults to Euler
        if (m_->opt.integrator == mjINT_IMPLICIT || m_->opt.integrator == mjINT_IMPLICITFAST) {
            mj_implicit(m_, d_);
        } else {
            mj_Euler(m_, d_);
        }
        d_->timer[mjTIMER_STEP].number--;
    } else if (sim_ == Config::sim_show) {
        mj_checkPos(m_, d_);
        mj_checkVel(m_, d_);
        mj_fwdPosition(m_, d_);
        mj_sensorPos(m_, d_);
        mj_energyPos(m_, d_);
        mj_fwdVelocity(m_, d_);
        mj_sensorVel(m_, d_);
        mj_energyVel(m_, d_);
        sim_show_step();
        mj_fwdActuation(m_, d_);
        mj_fwdAcceleration(m_, d_);
        mj_fwdConstraint(m_, d_);
        mj_sensorAcc(m_, d_);
        mj_checkAcc(m_, d_);
        // compare forward and inverse solutions if enabled
        if (m_->opt.enableflags & mjENBL_FWDINV) {
            mj_compareFwdInv(m_, d_);
        }
        // integrate with Euler or implicit; RK4 defaults to Euler
        if (m_->opt.integrator == mjINT_IMPLICIT || m_->opt.integrator == mjINT_IMPLICITFAST) {
            mj_implicit(m_, d_);
        } else {
            mj_Euler(m_, d_);
        }
        d_->timer[mjTIMER_STEP].number--;
    }
}

/**
 * @note algthm: right-hand axis, sim: left-hand axis
 */
void Simulation::SimulationBridge::sim_control() {
    // parse sensor data, order in xml file
    int gyro_adr = m_->sensor_adr[0];
    int quat_adr = m_->sensor_adr[1];
    int acc_adr = m_->sensor_adr[2];

    // add noise

    // std::cout << "[IN LOOP]\n";
    noise_quat << d_->sensordata[quat_adr + 0], d_->sensordata[quat_adr + 1], d_->sensordata[quat_adr + 2],
            d_->sensordata[quat_adr + 3];
    gyro_ << d_->sensordata[gyro_adr + 0], d_->sensordata[gyro_adr + 1], d_->sensordata[gyro_adr + 2];
    acc_ << d_->sensordata[acc_adr + 0], d_->sensordata[acc_adr + 1], d_->sensordata[acc_adr + 2];
    // Vec3<double> noise_rpy = ori::quatToRPY(noise_quat);
    // for (int i = 0; i < 3; i++) {
    //     noise_rpy(i) += dist_quat_(generator);
    // }
    // noise_quat = ori::rpyToQuat(noise_rpy);
    
    subscriber.take()
            .and_then([this](auto &sample) {
            
                for (int i = 0; i < 4; i++) {
                    const int index = i / 2;
                    const int index_shift = index * 2;
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift)].q_des = sample->q[3 * i];
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].q_des = sample->q[3 * i + 1];
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].q_des = sample->q[3 * i + 2];

                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift)].qd_des = sample->qd[3 * i];
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].qd_des = sample->qd[3 * i + 1];
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].qd_des = sample->qd[3 * i + 2];

                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift)].tau_ff = sample->tau_ff[3 * i];
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].tau_ff = sample->tau_ff[3 * i + 1];
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].tau_ff = sample->tau_ff[3 * i + 2];

                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift)].kp = sample->kp[3 * i];
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].kp = sample->kp[3 * i + 1];
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].kp = sample->kp[3 * i + 2];

                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift)].kd = sample->kd[3 * i];
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 1].kd = sample->kd[3 * i + 1];
                    motor_cmd_->chip_cmds[index].motor_cmds[3 * (i - index_shift) + 2].kd = sample->kd[3 * i + 2];
                }

                //for wheel control
                for (int i = 0; i < 4; i++) {
                    const int index = 2;
                    const int index_shift = i / 2;
                    motor_cmd_->chip_cmds[index].motor_cmds[i + index_shift].q_des = sample->q[12 + i];
                    motor_cmd_->chip_cmds[index].motor_cmds[i + index_shift].qd_des = sample->qd[12 + i];
                    motor_cmd_->chip_cmds[index].motor_cmds[i + index_shift].tau_ff = sample->tau_ff[12 + i];
                    motor_cmd_->chip_cmds[index].motor_cmds[i + index_shift].kp = sample->kp[12 + i];
                    motor_cmd_->chip_cmds[index].motor_cmds[i + index_shift].kd = sample->kd[12 + i];
                    // std::cout <<sample->kd[12 + i]<<std::endl;
                }
            })
            .or_else([](auto &result) {
                    if (result != iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE) {
                        std::cout << "Error receiving chunk." << std::endl;
                    }
                }
            );


    // for (int i = 0; i < m_->sensor_dim[0]; i++) {
    // usb_imu_->gyro[i] = static_cast<float>(d_->sensordata[gyro_adr + i] + dist_gyro_(generator));
    //     usb_imu_->q[i] = static_cast<float>(noise_quat(i));
    //     //            usb_imu_->q[i] = (float) d_->sensordata[quat_adr + i];
    //     usb_imu_->accel[i] = static_cast<float>(d_->sensordata[acc_adr + i] + dist_acc_(generator));
    // }
    // // std::cout << "accel: " << usb_imu_->accel[2] << std::endl;
    // usb_imu_->q[3] = static_cast<float>(noise_quat(3));
    // set mocap data
    // if (runner_started) {
    //     Vec3<double> mocap_pw = this->robot_runner_->estimators_->get_result_world_position();
    //     Quat<double> mocap_ori = this->robot_runner_->estimators_->get_result_quat();
    //     for (int i = 0; i < 3; i++) {
    //         d_->mocap_pos[3 * mocap_esti_indicator_id_ + i] = mocap_pw(i);
    //         d_->mocap_quat[4 * mocap_esti_indicator_id_ + i] = mocap_ori(i);
    //     }
    //     d_->mocap_quat[4 * mocap_esti_indicator_id_ + 3] = mocap_ori(3);
    // }

    if (sim_handle_->draw_traject) {
        sim_handle_->set_draw_traj(true);
        plot_subscriber_.take()
                .and_then([this](auto &sample) {
                        for (int i = 0; i < 3; i++) {
                            for (int j = 0; j < 4; j++) {
                                foot_pos_last_des_[j][i] = foot_pos_des_[j][i];
                                foot_pos_des_[j][i] = sample->foot_pos_des_[j][i];
                                foot_pos_last_[j][i] = foot_pos_[j][i];
                                foot_pos_[j][i] = sample->foot_pos_[j][i];
                            }
                            pos_last_des_[i] = pos_des_[i];
                            pos_last_[i] = pos_[i];
                            pos_des_[i] = sample->pos_des_[i];
                            pos_[i] = sample->pos_[i];
                        }
                        // std::cout << "pos_des: " << pos_des_[0] << " | " << pos_des_[1] << " | " << pos_des_[2] << std::endl;
                    }
                )
                .or_else([](auto &result) {
                        if (result != iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE) {
                            std::cout << "Error receiving chunk." << std::endl;
                        }
                    }
                );
        for (int i = 0; i < 4; i++) {
            sim_handle_->set_foot_trajectory_des(foot_pos_last_des_[i], foot_pos_des_[i], i);
            sim_handle_->set_foot_trajectory(foot_pos_last_[i], foot_pos_[i], i);
            sim_handle_->set_pos_trajectory(pos_last_, pos_);
            sim_handle_->set_pos_trajectory_des(pos_last_des_, pos_des_);
            // sim_handle_->set_mpc_force(mpc_force[i], i);
        }
    } else {
        sim_handle_->set_draw_traj(false);
    }
    /* parse joint data
     * body: free joint: xyz | q[4]
     * motor
     */
    // std::cout << "Actuator: \n" << d_->actuator_force[0] << " | " << d_->actuator_force[1] << " | "
    //         << d_->actuator_force[2] << std::endl;
    for (int i = 0; i < 4; i++) {
        const int index = i / 2;
        const int index_shift = index * 2; // (0,2)
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift)].q = static_cast<float>(d_->qpos[Config::abad_pos_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 1].q = static_cast<float>(d_->qpos[Config::hip_pos_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 2].q = static_cast<float>(d_->qpos[Config::knee_pos_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift)].qd = static_cast<float>(d_->qvel[Config::abad_vel_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 1].qd = static_cast<float>(d_->qvel[Config::hip_vel_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 2].qd = static_cast<float>(d_->qvel[Config::knee_vel_addr_offset + 4 * i]);
    }

    //for wheel control
    for (int i = 0; i < 4; i++) {
        const int index = 2;
        const int index_shift = i / 2;
        motor_data_->chip_datas[index].motor_datas[i + index_shift].q = static_cast<float>(d_->qpos[Config::whl_pos_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[i + index_shift].qd = static_cast<float>(d_->qvel[Config::whl_vel_addr_offset + 4 * i]);
        // std::cout << "motor_data_->chip_datas[index].motor_datas[i + index_shift].q: " << motor_data_->chip_datas[index].motor_datas[i + index_shift].q << std::endl;
    }

    motors_->pack_motor_cmd(motor_cmd_, motor_data_);
    publisher.loan()
            .and_then([this](auto &sample) {
                for (int i = 0; i < 3; i++) {
                    sample->gyro[i] = gyro_(i);
                    sample->acc[i] = acc_(i);
                }
                for (int i = 0; i < 4; i++) {
                    sample->quat[i] = noise_quat(i);
                    const int index = i / 2;
                    const int index_shift = index * 2;
                    sample->q[3 * i] = motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift)].q;
                    sample->q[3 * i + 1] = motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 1].q;
                    sample->q[3 * i + 2] = motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 2].q;
                    sample->qd[3 * i] = motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift)].qd;
                    sample->qd[3 * i + 1] = motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 1].qd;
                    sample->qd[3 * i + 2] = motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 2].qd;
                }
                for (int i = 0; i < 4; i++) {
                    const int index = 2;
                    const int index_shift = i / 2;
                    sample->q[12 + i] = motor_data_->chip_datas[index].motor_datas[i + index_shift].q;
                    sample->qd[12 + i] = motor_data_->chip_datas[index].motor_datas[i + index_shift].qd;
                }
                sample.publish();
            })
            .or_else([](auto &result) {
                std::cerr << "Unable to loan sample, error: " << result << std::endl;
            });
    //        std::cout << "torque: ";
    for (int i = 0; i < m_->nu; i++) {
        d_->ctrl[i] = motors_->get_torque(i);
        // std::cout << "d_->ctrl[i]: " << d_->ctrl[i] << std::endl;
        // d_->ctrl[i] = 0.0;
        //            std::cout << d_->ctrl[i] << " | ";
    }
        //    std::cout << std::endl;
}


void Simulation::SimulationBridge::sim_show_step() {
    usb_2_can_LCM_.handleTimeout(0);
    usb_imu_LCM_.handleTimeout(0);
    for (int i = 0; i < 4; i++) {
        const int index = i / 2; // (0, 1, 2, 3) / 2 = (0,1)
        const int index_shift = index * 2; // (0,2)
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift)].q = static_cast<float>(d_->qpos[Config::abad_pos_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 1].q = static_cast<float>(d_->qpos[Config::hip_pos_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 2].q = static_cast<float>(d_->qpos[Config::knee_pos_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift)].qd = static_cast<float>(d_->qvel[Config::abad_vel_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 1].qd = static_cast<float>(d_->qvel[Config::hip_vel_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[3 * (i - index_shift) + 2].qd = static_cast<float>(d_->qvel[Config::knee_vel_addr_offset + 4 * i]);

        // chip data: q, qd, tau_ff, uq,ud
        if (index == 0) {
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i].q_des = sim_local_usbdata.chip1_data[3 * i][0];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i].qd_des = sim_local_usbdata.chip1_data[3 * i][1];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i].tau_ff = 0;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i].kp = Config::joint_kp;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i].kd = Config::joint_kd;

            motor_cmd_->chip_cmds[index].motor_cmds[3 * i + 1].q_des = sim_local_usbdata.chip1_data[3 * i + 1][0];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i + 1].qd_des = sim_local_usbdata.chip1_data[3 * i + 1][1];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i + 1].tau_ff = 0;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i + 1].kp = Config::joint_kp;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i + 1].kd = Config::joint_kd;

            motor_cmd_->chip_cmds[index].motor_cmds[3 * i + 2].q_des = sim_local_usbdata.chip1_data[3 * i + 2][0];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i + 2].qd_des = sim_local_usbdata.chip1_data[3 * i + 2][1];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i + 2].tau_ff = 0;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i + 2].kp = Config::joint_kp;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * i + 2].kd = Config::joint_kd;
        }else if (index == 1) {
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift)].q_des = sim_local_usbdata.chip2_data[3 * (i-index_shift)][0];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift)].qd_des = sim_local_usbdata.chip2_data[3 * (i-index_shift)][1];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift)].tau_ff = 0;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift)].kp = Config::joint_kp;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift)].kd = Config::joint_kd;

            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift) + 1].q_des = sim_local_usbdata.chip2_data[3 * (i-index_shift) + 1][0];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift) + 1].qd_des = sim_local_usbdata.chip2_data[3 * (i-index_shift) + 1][1];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift) + 1].tau_ff = 0;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift) + 1].kp = Config::joint_kp;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift) + 1].kd = Config::joint_kd;

            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift) + 2].q_des = sim_local_usbdata.chip2_data[3 * (i-index_shift) + 2][0];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift) + 2].qd_des = sim_local_usbdata.chip2_data[3 * (i-index_shift) + 2][1];
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift) + 2].tau_ff = 0;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift) + 2].kp = Config::joint_kp;
            motor_cmd_->chip_cmds[index].motor_cmds[3 * (i-index_shift) + 2].kd = Config::joint_kd;
        }
    }

    for (int i = 0; i < 4; i++) {
        const int index = 2;
        const int index_shift = i / 2;
        motor_data_->chip_datas[index].motor_datas[i + index_shift].q = static_cast<float>(d_->qpos[Config::whl_pos_addr_offset + 4 * i]);
        motor_data_->chip_datas[index].motor_datas[i + index_shift].qd = static_cast<float>(d_->qvel[Config::whl_vel_addr_offset + 4 * i]);
    }

    for (int i = 0; i < 4; i++) {
        const int index = 2;
        const int index_shift = i / 2;
        motor_cmd_->chip_cmds[index].motor_cmds[i + index_shift].q_des = sim_local_usbdata.chip3_data[i + index_shift][0];
        motor_cmd_->chip_cmds[index].motor_cmds[i + index_shift].qd_des = sim_local_usbdata.chip3_data[i + index_shift][1];
        motor_cmd_->chip_cmds[index].motor_cmds[i + index_shift].tau_ff = 0;
        motor_cmd_->chip_cmds[index].motor_cmds[i + index_shift].kp = Config::joint_kp;
        motor_cmd_->chip_cmds[index].motor_cmds[i + index_shift].kd = Config::joint_kd;
    }

    motors_->pack_motor_cmd(motor_cmd_, motor_data_);
        //    std::cout << "torque: ";
    for (int i = 0; i < m_->nu; i++) {
        d_->ctrl[i] = motors_->get_torque(i);
        // d_->ctrl[i] = 0.0;
        //            std::cout << d_->ctrl[i] << " | ";
    } {
        std::lock_guard lk(sim_handle_->sim_imu_mtx_);
        for (int j = 0; j < 3; j++) {
            sim_handle_->sim_local_imudata.accel[j] = sim_local_imudata_.accel[j];
            sim_handle_->sim_local_imudata.gyro[j] = sim_local_imudata_.gyro[j];
            sim_handle_->sim_local_imudata.q[j] = sim_local_imudata_.q[j];
            // first 3 represents xyz
            d_->qpos[j + 3] = sim_local_imudata_.q[j];
            d_->qvel[j + 3] = sim_local_imudata_.gyro[j];
        }
        d_->qpos[0] = d_->qpos[1] = 0; // lock the robot .
        sim_handle_->sim_local_imudata.q[3] = sim_local_imudata_.q[3];
        d_->qpos[6] = sim_local_imudata_.q[3];
    }
}


void Simulation::SimulationBridge::set_lcm() {
    sim_ground_truth.p_w_[0] = (float) d_->qpos[0];
    sim_ground_truth.p_w_[1] = (float) d_->qpos[1];
    sim_ground_truth.p_w_[2] = (float) d_->qpos[2];
    Quat<float> q_ori_temp;
    q_ori_temp << (float) d_->qpos[3], (float) d_->qpos[4], (float) d_->qpos[5], (float) d_->qpos[6];
    Vec3<float> rpy_temp;
    rpy_temp = ori::quatToRPY(q_ori_temp);
    sim_ground_truth.p_w_[3] = rpy_temp(0);
    sim_ground_truth.p_w_[4] = rpy_temp(1);
    sim_ground_truth.p_w_[5] = rpy_temp(2);

    for (int i = 0; i < 6; i++) {
        sim_ground_truth.v_w_[i] = (float) d_->qvel[i];
    }
    for (int i = 0; i < 12; i++) {
        sim_ground_truth.q[i] = (float) d_->qpos[i + Config::abad_pos_addr_offset];
        sim_ground_truth.qd[i] = (float) d_->qvel[i + Config::abad_vel_addr_offset];
    }
    lcm_.publish("SIM TRUTH", &sim_ground_truth);
}


void Simulation::SimulationBridge::USB_DATA_LCM_HANDLE(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                                       const usb_data_t *msg) {
    (void) rbuf;
    (void) chan;
    memcpy(&sim_local_usbdata, msg, sizeof(usb_data_t));
    // std::cout << GREEN << "[Log Data]: " << msg->q_knee[2] << " | " << msg->q_knee[3] << " | " <<
    // msg->q_knee[0] << " | " << msg->q_knee[1] << "\n";
}

void Simulation::SimulationBridge::USB_IMU_LCM_HANDLE(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                                      const imu_lcmt *msg) {
    (void) rbuf;
    (void) chan;
    memcpy(&sim_local_imudata_, msg, sizeof(imu_lcmt));
    //        std::cout << GREEN << "[Log Data]: " << sim_local_imudata.q[0] << " | " << sim_local_imudata.q[1] << "\n";
}
