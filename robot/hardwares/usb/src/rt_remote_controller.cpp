//
// Created by lingwei on 4/4/24.
//
#include "../include/rt_remote_controller.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include "../../utilities/types/std_cout_colors.h"
#include "../../utilities/inc/utilities_fun.h"

namespace usb_controller {
    ssize_t logic_remote_controller::rc_map_read(int rc_fd, xbox_map_t *map) {
        int type, number, value;
        ssize_t length = read(rc_fd, &joystick, sizeof(struct js_event));
        type = joystick.type;
        number = joystick.number;
        value = joystick.value;
        map->time = joystick.time;
        //        std::cout << "Joystick time:" << joystick.time << std::endl;
        if (type == JS_EVENT_BUTTON) {
            switch (number) {
                case XBOX_BUTTON_A:
                    map->a = value;
                    //std::cout << "a" << "\n" << std::endl;
                    break;
                case XBOX_BUTTON_B:
                    map->b = value;
                    //std::cout << "b" << "\n" << std::endl;
                    break;
                case XBOX_BUTTON_X:
                    map->x = value;
                    //std::cout << "x" << "\n" << std::endl;
                    break;
                case XBOX_BUTTON_Y:
                    map->y = value;
                    //std::cout << "y" << "\n" << std::endl;
                    break;
                case XBOX_BUTTON_LB:
                    map->lb = value;
                    //std::cout << "lb" << "\n" << std::endl;
                    break;
                case XBOX_BUTTON_RB:
                    map->rb = value;
                    break;
                case XBOX_BUTTON_START:
                    map->start = value;
                    break;
                case XBOX_BUTTON_SELECT:
                    map->select = value;
                    break;
                case XBOX_BUTTON_LO:
                    map->lo = value;
                    break;
                case XBOX_BUTTON_RO:
                    map->ro = value;
                    break;
                default:
                    break;
            }
        } else if (type == JS_EVENT_AXIS) {
            switch (number) {
                case XBOX_AXIS_LX:
                    map->lx = deadzone_func(value, data_deadzone_width);
                    break;
                case XBOX_AXIS_LY:
                    map->ly = deadzone_func(value, data_deadzone_width);
                    break;
                case XBOX_AXIS_RX:
                    map->rx = deadzone_func(value, data_deadzone_width);
                    break;
                case XBOX_AXIS_RY:
                    map->ry = deadzone_func(value, data_deadzone_width);
                    break;
                case XBOX_AXIS_LT:
                    map->lt = deadzone_func(value, data_deadzone_width);
                    break;
                case XBOX_AXIS_RT:
                    map->rt = deadzone_func(value, data_deadzone_width);
                    break;
                case XBOX_AXIS_XX: //方向键
                    map->xx = value;
                    break;
                case XBOX_AXIS_YY:
                    map->yy = value;
                    break;
                default:
                    break;
            }
        } else {
            /* Init do nothing */
            //            std::cout << "STUCH ELSE\n";
        }
        if (print_data_) {
            std::cout << MAGENTA << "[RC DATA]: " << RESET << "a:" << map->a << " | " << "b:" << map->b << " | " <<
                    "xx: " << map->xx << " | rt: " << map->rt << " | lt: " << map->lt << " | lx: " << map->lx
                    << " | ly: " << map->ly << " | rb: " << map->rb << " | rx: " << map->rx << " | ry: " << map->ry
                    << " | x: " << map->x << " | y: " << map->y << " | yy: " << map->yy << " | lb: " << map->lb
                    << " | lo: " << map->lo << " | ro: " << map->ro << " | start: " << map->start << " | back: "
                    << map->back
                    << " | home: " << map->home << " | select: " << map->select
                    << std::endl;
        }
        return length;
    }

    int logic_remote_controller::rc_open(const char *file_name) {
        int local_rc_fd = open(file_name, O_RDONLY | O_NONBLOCK);
        if (local_rc_fd < 0) {
            // std::cout << RED << "[RC ERROR]: " << RESET << "Can not open joystick!\n";
            return -1;
        }
        return local_rc_fd;
    }

    void logic_remote_controller::rc_close() const {
        close(rc_fd_);
        // std::cout << GREEN << "[RC SUCCESS]: " << RESET << "Close the rc controller!\n";
    }

    ssize_t logic_remote_controller::rc_complete() {
        // rt and lt range:-32767~32767, starts from -32767
        // button and direction(lxy and rxy): -32767 -32767^ 32767_ 32767
        ssize_t length = rc_map_read(rc_fd_, &rc_map_);
        (void) length;
        //        std::cout << "Read length: " << length <<"\n";
        //        if (length < 0) return;
        {
            std::lock_guard lk(rc_mtx_);
            if (rc_map_.lb && rc_map_.a)
                rc_control_.mode = RECOVER_STAND;

            // if (rc_map_.lb && rc_map_.lo) {
            //     rc_control_.mode = SITDOWN;
            // }
            if (rc_map_.lb && rc_map_.x) {
                rc_control_.mode = SITDOWN;
            }
            // if (rc_map_.lb && rc_map_.x) {
            //     rc_control_.mode = PASSIVE;
            // }
            if (rc_map_.rb && rc_map_.x) {
                rc_control_.mode = PASSIVE;
            }
            // if (rc_map_.lb && rc_map_.rb) {
            //     rc_control_.mode = DAMPING;
            // }
            if (rc_map_.lb && rc_map_.rb) {
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
            if (rc_map_.select) {
                delay_count++;
                if (!selected && (delay_count > 50)) {
                    rc_control_.variables[1] = 1;
                    selected = true;
                    delay_count = 0;
                } else if (delay_count > 50) {
                    selected = false;
                    rc_control_.variables[1] = 2;
                    delay_count = 0;
                }
            }
        }
        if (rc_control_.mode == RL_WALK || rc_control_.mode == RL_WALK_2) {
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
        if (rc_map_.lb && (rc_map_.yy<-10000)) {
                rc_control_.stand_flag = 1;
            }
            if (rc_map_.lb && (rc_map_.yy>10000)) {
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
        //         rc_control_.v_des[0] = -static_cast<float>(rc_map_.ly) / 32768.f / 4.f;
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


    logic_remote_controller::logic_remote_controller(bool print_data) : print_data_(print_data),
                                                                        rc_LCM(getLcmUrl(255)) {
        rc_fd_ = rc_open("/dev/input/js0");
        if (rc_fd_ > 0) {
            std::cout << GREEN << "[RC SUCCESS]: " << RESET << "Finish open the device js0!\n";
            std::cout << GREEN << "[RC LCM SUCCESS]: " << RESET << "Finish initializing the lcm!\n";
        } else {
            std::cout << BOLDRED << "[RC ERROR]: " << RESET << "Can not open the device js0!\n";
        }
        rc_control_.step_height = 0.8;
        rc_control_.height_variation = 0;
        joystick_gait = 3;
    }
}
