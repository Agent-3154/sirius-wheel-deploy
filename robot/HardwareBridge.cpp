//
// Created by lingwei on 4/16/24.
//
#include "HardwareBridge.h"
#include "../utilities/types/std_cout_colors.h"
#include "../config/Config.h"
#include "../config/robots_config.h"
#include "../utilities/inc/easylogging++.h"
/**
 * @note components initiate in robot runner
 * @param model_name
 * @param robot_controller
 * @param type_
 */
HardwareBridge::My_HardwareBridge::My_HardwareBridge(std::string &model_name, Robot_Controller_Base *robot_controller,
                                                     Config::run_type type_)
{
    usb_cmd_ = new USB_Command_t();
    usb_data_ = new USB_Data_t();
    usb_imu_ = new USB_Imu_t();
    robot_runner_ = new RobotRunner(model_name, robot_controller, type_);
    usb_container_ = new USB_HARDWARE::USB_Hardware_Containers();
    robot_runner_->runner_imudata_ = usb_imu_;
    robot_runner_->runner_usbcmd_ = usb_cmd_;
    robot_runner_->runner_usbdata_ = usb_data_;
}

HardwareBridge::My_HardwareBridge::~My_HardwareBridge()
{
    delete usb_imu_;
    delete usb_cmd_;
    delete usb_data_;
    delete robot_runner_;
    delete usb_container_;
}

/**
 *
 * @param real_imu: control the thread of imu
 * @param real_usb2can: control the thread of usb2can
 */
void HardwareBridge::My_HardwareBridge::setup_HardwareBridge(const bool real_imu, const bool real_usb2can)
{
    t_usb_ = std::make_shared<Thread::thread_usb_hardwares>("USB Hardwares", 0);
    
    if (real_imu)
    {
        imu_handle_ = new USB_HARDWARE::USB_IMU(Config::vendor_id, Config::product_id, Config::endpoint_1, 0x0);
        imu_handle_->usb_imu_set_rx_buffer(usb_imu_);
        // robot runner get handle for mutex accessing
        robot_runner_->runner_imu_ = imu_handle_;
        usb_container_->add_usb_device(imu_handle_);
    }
    else
    {
        imu_handle_ = nullptr;
        robot_runner_->runner_imu_ = nullptr;
    }

    if (real_usb2can)
    {
        usb2can_board_handle_ = new USB_HARDWARE::Beast_USB2CAN(Config::usb2can_vendor_id,
                                                                Config::usb2can_product_id,
                                                                Config::motors_ep_in, Config::motors_ep_out);
        usb2can_board_handle_->USB2CAN_SetBuffer(usb_cmd_, usb_data_);
        robot_runner_->runner_usb2can_ = usb2can_board_handle_;
        usb_container_->add_usb_device(usb2can_board_handle_);
    }
    else
    {
        usb2can_board_handle_ = nullptr;
        robot_runner_->runner_usb2can_ = nullptr;
    }

    // start usb hardwares
    if (usb_container_->usb_container_.empty())
    {
        std::cout << RED << "[USB Hardware Thread]:" << RESET << "No device added\n";
    }
    else
    {
        // tp_usb_->Schedule([this] { t_usb_->thread_loop(usb_container_); });
        thread_usb2can_ = std::thread(&My_HardwareBridge::thread_usb2can_function, this);
        thread_imu_ = std::thread(&My_HardwareBridge::thread_imu_function, this);
    }
}

[[noreturn]] void HardwareBridge::My_HardwareBridge::run()
{
    auto last_print_time = std::chrono::steady_clock::now();
    int step_count = 0;

    // assert rc_handle_ is not nullptr
    if (rc_handle_ == nullptr) {
        LOG(FATAL) << RED << "RC handle is nullptr" << RESET;
    }

    for (;;)
    {
        this->t_robot_runner_->thread_enter_task();
        this->robot_runner_->run_step(step_count);
        this->t_robot_runner_->thread_finish_task();

        step_count++;
        auto current_time = std::chrono::steady_clock::now();
        if (step_count % 1000 == 0)
        {
            auto interval_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_print_time);
            double frequency = (step_count * 1000.0) / interval_duration.count();
            std::cout << GREEN << "[Main Loop Frequency]: " << RESET << frequency << " Hz" << std::endl;
            last_print_time = current_time;
            step_count = 0; // Reset counter for next interval
        }
    }
}

void HardwareBridge::My_HardwareBridge::setup_rc(const std::string &config_file) {
    t_rc_ = std::make_shared<Thread::thread_rc>("RC Thread", 200);
    // tp_rc_ = std::make_shared<Utilities::ThreadPool>(1);
    this->rc_handle_ = new usb_controller::LogicRemoteController(
        "/dev/input/js0",
        false,
        config_file
    );
    // tp_rc_->Schedule([this]() { t_rc_->thread_loop(rc_handle_); });
    this->thread_rc_ = std::thread(&My_HardwareBridge::thread_rc_function, this);
    this->robot_runner_->runner_rc_ = this->rc_handle_;
}

void HardwareBridge::My_HardwareBridge::setup_runner() {
    std::cout << GREEN << "[Setup Runner OK]: " << RESET << "Initialize robot runner!\n";
    if (robot_runner_->sim_ == Config::real_usb)
    {
        robot_runner_->init_robotrunner();
        t_robot_runner_ = std::make_shared<Thread::thread_robot_runner>(
            "Robot Runner Thread", Config::real_control_thread_fre);
        std::cout << GREEN << "[Robot Runner Thread]: " << RESET
                    << "Start running robot runner thread!\n";
    }
    else if (robot_runner_->sim_ == Config::sim_mj)
    {
        robot_runner_->init_robotrunner();
        t_robot_runner_ = std::make_shared<Thread::thread_robot_runner>(
            "Robot Runner Thread", Config::sim_robot_runner_task_fre);
    }
}

void HardwareBridge::My_HardwareBridge::thread_usb2can_function()
{
    struct timeval timestruc{};
    timestruc.tv_sec = 0;
    timestruc.tv_usec = 500; // return immediately
    int compelte = 0;
    usb2can_board_handle_->start_transfer();

    std::cout << GREEN << "[Thread USB2CAN hardware OK]: " << RESET << "Initialize usb2can hardware thread!\n";
    while (true)
    {
        libusb_handle_events_timeout_completed(usb2can_board_handle_->ctx, &timestruc, &compelte);
    }
}

void HardwareBridge::My_HardwareBridge::thread_imu_function()
{
    struct timeval timestruc{};
    timestruc.tv_sec = 0;
    timestruc.tv_usec = 1000; // return immediately
    int compelte = 0;
    imu_handle_->start_transfer();
    std::cout << GREEN << "[Thread IMU hardware OK]: " << RESET << "Initialize IMU hardware thread!\n";
    while (true)
    {
        libusb_handle_events_timeout_completed(imu_handle_->ctx, &timestruc, &compelte);
    }
}

void HardwareBridge::My_HardwareBridge::thread_rc_function()
{
    std::cout << GREEN << "[Thread RC OK]: " << RESET << "Initialize RC thread!\n";
    bool game_pad_connecting = false;
    struct stat buffer;
    while (true)
    {
        t_rc_->thread_enter_task();
        (void)rc_handle_->rc_complete();
        int game_pad_lost = stat("/dev/input/js0", &buffer);
        if (game_pad_lost == -1 && !game_pad_connecting)
        {
            rc_handle_->rc_close();
            game_pad_connecting = true;
            LOG(WARNING) << RED << "Gamesir Lost, reconnecting" << RESET;
        }
        if (game_pad_connecting)
        {
            int ret = rc_handle_->rc_open("/dev/input/js0");
            if (ret != -1)
            {
                rc_handle_->rc_fd_ = ret;
                game_pad_lost = 0;
                game_pad_connecting = false;
                LOG(INFO) << GREEN << "Reconnecting Gamesir success!" << RESET;
            }
        }
        // std::cout  << "RC Read: " << read_bit << "\n"<< std::flush;
        t_rc_->thread_finish_task();
    }
}
