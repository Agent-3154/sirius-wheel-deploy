//
// Created by lingwei on 7/4/24.
//

#ifndef MY_FDSC_H
#define MY_FDSC_H

#include "inc/free_dog_sdk_h.hpp"
#include "../../utilities/types/hardware_types.h"
#include "../../lcm-types/cpp/imu_lcmt.hpp"
#include "../../lcm-types/cpp/usb_data_t.hpp"
#include "lcm/lcm-cpp.hpp"

enum connection_type {
    low_wired = 0,
    sim_default,
    high_wifi,
};

constexpr int motors_abad[4] = {
    static_cast<int>(FDSC::Motor::FR_0),
    static_cast<int>(FDSC::Motor::FL_0),
    static_cast<int>(FDSC::Motor::RR_0),
    static_cast<int>(FDSC::Motor::RL_0)
};

constexpr int motors_hip[4] = {
    static_cast<int>(FDSC::Motor::FR_1),
    static_cast<int>(FDSC::Motor::FL_1),
    static_cast<int>(FDSC::Motor::RR_1),
    static_cast<int>(FDSC::Motor::RL_1)
};

constexpr int motors_knee[4] = {
    static_cast<int>(FDSC::Motor::FR_2),
    static_cast<int>(FDSC::Motor::FL_2),
    static_cast<int>(FDSC::Motor::RR_2),
    static_cast<int>(FDSC::Motor::RL_2)
};

inline std::vector<std::string> motors_abad_name = {"FR_0", "FL_0", "RR_0", "RL_0"};
inline std::vector<std::string> motors_hip_name = {"FR_1", "FL_1", "RR_1", "RL_1"};
inline std::vector<std::string> motors_knee_name = {"FR_2", "FL_2", "RR_2", "RL_2"};

class My_FDSC {
public:
    explicit My_FDSC(connection_type type);

    ~My_FDSC() = default;

    void set_buffer(USB_Command_t *usb_command, USB_Data_t *usb_data, USB_Imu_t *usb_imu);

    void FDSC_init();

    void show_info();

    static void show_joint_info(const std::vector<FDSC::MotorState> &mobj);

    void parse_data();

    void pack_cmd();

    std::mutex data_mtx_;
    std::mutex cmd_mtx_;

private:
    FDSC::lowCmd low_cmd_{};
    std::unique_ptr<FDSC::UnitreeConnection> ptr_connection_;
    FDSC::lowState low_state_{};
    FDSC::MotorCmdArray motor_cmd_array_{};
    USB_Command_t *usb_command_ = nullptr;
    USB_Data_t *usb_data_ = nullptr;
    USB_Imu_t *usb_imu_ = nullptr;

    imu_lcmt imu_data_lcm_{};
    usb_data_t usbdata_lcm_{};
    lcm::LCM imu_data_LCM;
    lcm::LCM usb_data_LCM;
};

#endif //MY_FDSC_H
