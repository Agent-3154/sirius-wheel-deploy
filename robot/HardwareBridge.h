//
// Created by lingwei on 4/16/24.
//

#ifndef MY_MUJOCO_SIMULATOR_HARDWAREBRIDGE_H
#define MY_MUJOCO_SIMULATOR_HARDWAREBRIDGE_H

#include <memory>

#include "thread_tasks/inc/thread_usb_hardware.h"
#include "thread_tasks/inc/thread_rc.h"
#include "../utilities/types/hardware_types.h"
#include "robot_runner/Robot_Runner.h"
#include "thread_tasks/inc/thread_robot_runner.h"
#include "../config/Config.h"

namespace HardwareBridge {
    class My_HardwareBridge {
    public:
        explicit My_HardwareBridge(std::string &model_name, Robot_Controller_Base *robot_controller, Config::run_type type_);

        ~My_HardwareBridge();

        [[noreturn]] void setup_HardwareBridge(bool real_imu, bool real_usb2can, bool real_rc, bool unitree_);
        //todo Add robot controller
    private:
        std::thread thread_usb2can_;
        std::thread thread_imu_;
        std::thread thread_rc_;
        std::thread thread_free_sdk_;

        void thread_usb2can_function();
        void thread_rc_function();
        void thread_imu_function();
        // std::shared_ptr<Utilities::ThreadPool> tp_usb_;
        // std::shared_ptr<Utilities::ThreadPool> tp_rc_;
        // std::shared_ptr<Utilities::ThreadPool> tp_robot_runner_;
        // std::shared_ptr<Utilities::ThreadPool> tp_fdsk_;

        std::shared_ptr<Thread::thread_usb_hardwares> t_usb_;
        std::shared_ptr<Thread::thread_rc> t_rc_;
        std::shared_ptr<Thread::thread_robot_runner> t_robot_runner_;

        RobotRunner* robot_runner_ = nullptr;
        USB_HARDWARE::Beast_USB2CAN *usb2can_board_handle_ = nullptr;
        USB_HARDWARE::USB_IMU *imu_handle_ = nullptr;
        USB_HARDWARE::USB_Hardware_Containers* usb_container_;
        usb_controller::logic_remote_controller *rc_handle_ = nullptr;

        USB_Command_t *usb_cmd_;
        USB_Data_t *usb_data_;
        USB_Imu_t *usb_imu_;

    };
}

#endif //MY_MUJOCO_SIMULATOR_HARDWAREBRIDGE_H
