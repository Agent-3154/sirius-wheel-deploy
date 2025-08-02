//
// Created by lingwei on 4/4/24.
//
#include "../include/rt_remote_controller.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "../../utilities/types/std_cout_colors.h"
#include "../../utilities/inc/utilities_fun.h"

namespace usb_controller
{

    ssize_t LogicRemoteController::rc_map_read(int rc_fd, xbox_map_t *map)
    {
        int type, number, value;
        ssize_t length = read(rc_fd, &joystick, sizeof(struct js_event));
        type = joystick.type;
        number = joystick.number;
        value = joystick.value;
        map->time = joystick.time;

        // std::cout << "Joystick time:" << joystick.time << std::endl;
        // std::cout << "Joystick type:" << type << std::endl;
        // std::cout << "Joystick number:" << number << std::endl;
        // std::cout << "Joystick value:" << value << std::endl;

        if (type == JS_EVENT_BUTTON)
        {
            // Use configurable button mappings
            if (number == button_mapping_.getButton("A"))
                map->a = value;
            else if (number == button_mapping_.getButton("B"))
                map->b = value;
            else if (number == button_mapping_.getButton("X"))
                map->x = value;
            else if (number == button_mapping_.getButton("Y"))
                map->y = value;
            else if (number == button_mapping_.getButton("LB"))
                map->lb = value;
            else if (number == button_mapping_.getButton("RB"))
                map->rb = value;
            else if (number == button_mapping_.getButton("START"))
                map->start = value;
            else if (number == button_mapping_.getButton("SELECT"))
                map->select = value;
            else if (number == button_mapping_.getButton("LO"))
                map->lo = value;
            else if (number == button_mapping_.getButton("RO"))
                map->ro = value;
        }
        else if (type == JS_EVENT_AXIS)
        {
            // Use configurable axis mappings
            if (number == button_mapping_.getAxis("LX"))
                map->lx = deadzone_func(value, data_deadzone_width);
            else if (number == button_mapping_.getAxis("LY"))
                map->ly = deadzone_func(value, data_deadzone_width);
            else if (number == button_mapping_.getAxis("RX"))
                map->rx = deadzone_func(value, data_deadzone_width);
            else if (number == button_mapping_.getAxis("RY"))
                map->ry = deadzone_func(value, data_deadzone_width);
            else if (number == button_mapping_.getAxis("LT"))
                map->lt = deadzone_func(value, data_deadzone_width);
            else if (number == button_mapping_.getAxis("RT"))
                map->rt = deadzone_func(value, data_deadzone_width);
            else if (number == button_mapping_.getAxis("XX"))
                map->xx = value;
            else if (number == button_mapping_.getAxis("YY"))
                map->yy = value;
        }
        else
        {
            /* Init do nothing */
            //            std::cout << "STUCH ELSE\n";
        }
        if (print_data_)
        {
            std::cout << MAGENTA << "[RC DATA]: " << RESET << "a:" << map->a << " | " << "b:" << map->b << " | " << "xx: " << map->xx << " | rt: " << map->rt << " | lt: " << map->lt << " | lx: " << map->lx
                      << " | ly: " << map->ly << " | rb: " << map->rb << " | rx: " << map->rx << " | ry: " << map->ry
                      << " | x: " << map->x << " | y: " << map->y << " | yy: " << map->yy << " | lb: " << map->lb
                      << " | lo: " << map->lo << " | ro: " << map->ro << " | start: " << map->start << " | back: "
                      << map->back
                      << " | home: " << map->home << " | select: " << map->select
                      << std::endl;
        }
        return length;
    }

    int LogicRemoteController::rc_open(const char *file_name)
    {
        int local_rc_fd = open(file_name, O_RDONLY | O_NONBLOCK);
        if (local_rc_fd < 0)
        {
            // std::cout << RED << "[RC ERROR]: " << RESET << "Can not open joystick!\n";
            return -1;
        }
        return local_rc_fd;
    }

    void LogicRemoteController::rc_close() const
    {
        close(rc_fd_);
        // std::cout << GREEN << "[RC SUCCESS]: " << RESET << "Close the rc controller!\n";
    }

    bool LogicRemoteController::loadButtonMapping(const std::string &config_file)
    {
        if (config_file.empty())
        {
            std::cout << YELLOW << "[RC WARNING]: " << RESET << "No config file specified, using default Xbox mappings.\n";
            return true;
        }

        try
        {
            YAML::Node config = YAML::LoadFile(config_file);

            // Load button mappings
            if (config["buttons"])
            {
                YAML::Node buttons = config["buttons"];
                for (const auto &button : buttons)
                {
                    std::string name = button.first.as<std::string>();
                    int number = button.second.as<int>();
                    button_mapping_.buttons[name] = number;
                }
            }

            // Load axis mappings
            if (config["axes"])
            {
                YAML::Node axes = config["axes"];
                for (const auto &axis : axes)
                {
                    std::string name = axis.first.as<std::string>();
                    int number = axis.second.as<int>();
                    button_mapping_.axes[name] = number;
                }
            }

            std::cout << GREEN << "[RC SUCCESS]: " << RESET << "Loaded button mapping from " << config_file << std::endl;
            return true;
        }
        catch (const YAML::BadFile &e)
        {
            std::cout << RED << "[RC ERROR]: " << RESET << "Could not open config file: " << config_file << std::endl;
            return false;
        }
        catch (const YAML::ParserException &e)
        {
            std::cout << RED << "[RC ERROR]: " << RESET << "YAML parsing error in " << config_file << ": " << e.what() << std::endl;
            return false;
        }
        catch (const std::exception &e)
        {
            std::cout << RED << "[RC ERROR]: " << RESET << "Error loading config file: " << e.what() << std::endl;
            return false;
        }
    }

    ssize_t LogicRemoteController::rc_complete()
    {
        // rt and lt range:-32767~32767, starts from -32767
        // button and direction(lxy and rxy): -32767 -32767^ 32767_ 32767
        ssize_t length = rc_map_read(rc_fd_, &rc_map_);
        (void)length;
        //        std::cout << "Read length: " << length <<"\n";
        //        if (length < 0) return;
        {
            std::lock_guard lk(rc_mtx_);
            if (rc_map_.lb && rc_map_.a)
                rc_control_.mode = RECOVER_STAND;

            // if (rc_map_.lb && rc_map_.lo) {
            //     rc_control_.mode = SITDOWN;
            // }
            if (rc_map_.lb && rc_map_.x)
            {
                rc_control_.mode = SITDOWN;
            }
            // if (rc_map_.lb && rc_map_.x) {
            //     rc_control_.mode = PASSIVE;
            // }
            if (rc_map_.rb && rc_map_.x)
            {
                rc_control_.mode = PASSIVE;
            }
            // if (rc_map_.lb && rc_map_.rb) {
            //     rc_control_.mode = DAMPING;
            // }
            if (rc_map_.lb && rc_map_.rb)
            {
                rc_control_.mode = DAMPING;
            }
            // if (rc_map_.lb && rc_map_.b)
            //     rc_control_.mode = RL_RUN;
            if (rc_map_.lb && rc_map_.b)
                rc_control_.mode = RL_WALK_2;

            if (rc_map_.lb && rc_map_.y)
                rc_control_.mode = RL_WALK;
            // if (rc_map_.lb && rc_map_.y)
            //     rc_control_.mode = RL_WALK_2;

#ifdef GAME_STAR
            if (rc_map_.lb && rc_map_.ro)
                rc_control_.mode = USER_INTERFACE;
#else
            if (rc_map_.lb && rc_map_.start)
                rc_control_.mode = USER_INTERFACE;
#endif
            // draw lines in simulation
            if (rc_map_.select)
            {
                delay_count++;
                if (!selected && (delay_count > 50))
                {
                    rc_control_.variables[1] = 1;
                    selected = true;
                    delay_count = 0;
                }
                else if (delay_count > 50)
                {
                    selected = false;
                    rc_control_.variables[1] = 2;
                    delay_count = 0;
                }
            }
        }
        if (rc_control_.mode == RL_WALK || rc_control_.mode == RL_WALK_2)
        {
            rc_control_.v_des[0] = -static_cast<float>(rc_map_.ly) / 32768;
            rc_control_.v_des[1] = -static_cast<float>(rc_map_.lx) / 32768;
            rc_control_.v_des[2] = -static_cast<float>(rc_map_.rx) / 32768;
            rc_control_.height_variation = -static_cast<float>(rc_map_.ly) / 32768;
            rc_control_.omega_des[0] = 0;
            rc_control_.omega_des[1] = 0;
            rc_control_.omega_des[2] = 0;
            // std::cout<<rc_map_.yy<<std::endl;
        }
        rc_control_.stand_flag = 0;
        if (rc_map_.lb && (rc_map_.yy < -10000))
        {
            rc_control_.stand_flag = 1;
        }
        if (rc_map_.lb && (rc_map_.yy > 10000))
        {
            rc_control_.stand_flag = 0;
        }

        // if (rc_control_.mode == LOCOMOTION) {
        //     if (rc_map_.y) { rc_control_.variables[0] = 1; } //trot
        //     else if (rc_map_.x) { rc_control_.variables[0] = 0; } //stand
        //     else if (rc_map_.a) { rc_control_.variables[0] = 3; } //walk
        //     else if (rc_map_.b) { rc_control_.variables[0] = 4; } // running
        //     //
        //     // //            if (rc_map_.rt > 30000 && rc_map_.start) joystick_gait = 4;
        //     //             rc_control_.variables[0] = joystick_gait;
        //     if (rc_control_.variables[0] == 4) {
        //         // rc_control_.v_des[0] = -static_cast<float>(rc_map_.ly) / 32768.f * 3.f;
        //         // rc_control_.v_des[1] = 0;
        //         rc_control_.v_des[0] = -static_cast<float>(rc_map_.ly) / 32768.f;
        //         rc_control_.v_des[1] = -1.f * static_cast<float>(rc_map_.lx) / 32768.f;
        //         rc_control_.v_des[2] = 0;
        //         rc_control_.omega_des[0] = 0;
        //         rc_control_.omega_des[1] = 0; //(float)map.ry/32768;//pitch
        //         rc_control_.omega_des[2] = static_cast<float>(rc_map_.rx) / 32768.f / 2.f;
        //         rc_control_.rpy_des[0] = 0;
        //     } else if (rc_control_.variables[0] == 3) {
        //         rc_control_.v_des[0] = = -static_cast<float>(rc_map_.ly) / 32768.f / 4.f;
        //         rc_control_.v_des[1] = 0;
        //         rc_control_.v_des[2] = 0;
        //         rc_control_.omega_des[0] = 0;
        //         rc_control_.omega_des[1] = 0; //(float)map.ry/32768;//pitch
        //         rc_control_.omega_des[2] = static_cast<float>(rc_map_.rx) / 32768.f / 2.f;
        //         rc_control_.rpy_des[0] = 0;
        //     } else if (rc_control_.variables[0] == 1) {
        //         rc_control_.v_des[0] = -static_cast<float>(rc_map_.ly) / 32768.f / 1.f;
        //         rc_control_.v_des[1] = -1.f * static_cast<float>(rc_map_.lx) / 32768.f / 3.f;
        //         rc_control_.v_des[2] = 0;
        //         rc_control_.omega_des[0] = 0;
        //         rc_control_.omega_des[1] = 0; //(float)map.ry/32768;//pitch
        //         rc_control_.omega_des[2] = static_cast<float>(rc_map_.rx) / 32768.f / 2.f;
        //         rc_control_.rpy_des[0] = 0;
        //     }
        //             rc_control_.height_variation = (float) rc_map_.ry / 32768;
        //             if (rc_map_.xx < -30000) rc_control_.step_height -= 0.3;
        //             if (rc_map_.xx > 30000) rc_control_.step_height += 0.3;   //dm
        memcpy(&rc_lcmdata, &rc_control_, sizeof(rc_lcmt));
        rc_LCM.publish("RC_CHANNEL", &rc_lcmdata);
        return length;
    }

    LogicRemoteController::LogicRemoteController(
        const std::string &device_file,
        bool print_data,
        const std::string &config_file) : device_file_(device_file),
                                          print_data_(print_data),
                                          config_file_(config_file)
    {
        rc_fd_ = rc_open(device_file_.c_str()); // defaults to `/dev/input/js0`
        if (rc_fd_ > 0)
        {
            std::cout << GREEN << "[RC SUCCESS]: " << RESET << "Finish open the device js0!\n";
            std::cout << GREEN << "[RC LCM SUCCESS]: " << RESET << "Finish initializing the lcm!\n";
        }
        else
        {
            std::cout << BOLDRED << "[RC ERROR]: " << RESET << "Can not open the device js0!\n";
        }

        // Load button mapping configuration
        if (!config_file_.empty())
        {
            loadButtonMapping(config_file_);
        }

        rc_control_.step_height = 0.8;
        rc_control_.height_variation = 0;
        joystick_gait = 3;
    }
}
