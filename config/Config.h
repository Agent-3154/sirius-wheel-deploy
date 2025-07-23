//
// Created by lingwei on 5/16/24.
//

#ifndef MY_MUJOCO_SIMULATOR_CONFIG_H
#define MY_MUJOCO_SIMULATOR_CONFIG_H

#include <array>

// sim config
namespace Config {
#define ABS_ABAD_OFFSET 0.0f
#define ABS_HIP_OFFSET 0.f
#define ABS_KNEE_OFFSET 0.f

    constexpr bool thread_time_spy = true; // thread task frequency warning
    constexpr int drawing_geom_number_feets = 600;
    constexpr int drawing_geom_number_pw = 6000;

    enum Joint_Axis {
        Joint_Axis_X = 0,
        Joint_Axis_Y,
        Joint_Axis_Z,
    };

    constexpr int Joint_Axises[12] = {
        Joint_Axis_X, Joint_Axis_Y, Joint_Axis_Y,
        Joint_Axis_X, Joint_Axis_Y, Joint_Axis_Y,
        Joint_Axis_X, Joint_Axis_Y, Joint_Axis_Y,
        Joint_Axis_X, Joint_Axis_Y, Joint_Axis_Y,
    };

    enum run_type {
        nothing = 0,
        real_usb = 1, // real_ctrl_byusb
        real_unitree, //real_ctrl_go1
        real_ros_ctrl,
        sim_show, // syn imu and motor datas with real robot
        sim_mj, // sim in mujoco
        sim_lcm,
        sim_embedded_in_other// sim in ros or cheetah
    };

    // TODO: Is the abad axis inverse?
    constexpr double damping_kd = 4;
    constexpr double qd_danger = 25.;
    // Task frequency
    constexpr int sim_remote_controller_task_fre = 200;
    constexpr int sim_robot_runner_task_fre = 500;
    constexpr int sim_task_fre = 1000;
    constexpr bool mj_sim_time_spy = true;
    constexpr int real_control_thread_fre = 500;
    constexpr double bs_rpy_filter = 0.1;
    constexpr double loco_vel_filter = 0.01;
    constexpr double planner_rpy_filter = 0.1;

    // mj_joint addr offset
    constexpr int abad_pos_addr_offset = 7;
    constexpr int hip_pos_addr_offset = 8;
    constexpr int knee_pos_addr_offset = 9;
    constexpr int whl_pos_addr_offset = 10;
    
    constexpr int abad_vel_addr_offset = 6;
    constexpr int hip_vel_addr_offset = 7;
    constexpr int knee_vel_addr_offset = 8;
    constexpr int whl_vel_addr_offset = 9;

    // real robot config
    constexpr uint16_t usb2can_vendor_id = 0x1111;
    constexpr uint16_t usb2can_product_id = 0x2222;
    constexpr uint8_t motors_ep_in = 0x81;
    constexpr uint8_t motors_ep_out = 0x01;

    constexpr uint16_t vendor_id = 0x1234;
    constexpr uint16_t product_id = 0x6789;
    constexpr unsigned char endpoint_1 = 0x81;

    // Spatial Vector Offset
    constexpr int subtree_starts_offset = 6;

    constexpr double G = 9.81;

    // wbc config
    constexpr int num_act_joints = 12;
    constexpr int num_dim_config = 18;
    constexpr int num_joints_on_leg = 3;
    constexpr int num_legs = 4;

    // sensor noise for sim
    constexpr double noise_acc = 0.005;
    constexpr double noise_quat = 0.002;
    constexpr double noise_gyro = 0.005;
    constexpr double noise_encoder = 0.001;
    constexpr double noise_enc_vel = 0.01;
    constexpr int nbody_clip = 14; // use for model body clip
    constexpr double total_mass = 18;
    constexpr double draw_force_scale = 10;
    // mocap
    const std::string mocap_esti_indicator_name = "esti_indicators";

    // sim show kp kd.
    constexpr double joint_kp = 50;
    constexpr double joint_kd = 1;

#if defined ROS_PATH
    const std::string path_2_config_directory = "src/ergo_controller/mj_ctrl/";
#else
    const std::string path_2_config_directory = "../";
#endif

}
#endif //MY_MUJOCO_SIMULATOR_CONFIG_H