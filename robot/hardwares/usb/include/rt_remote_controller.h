//
// Created by lingwei on 4/4/24.
//

#ifndef MY_MUJOCO_SIMULATOR_RT_REMOTE_CONTROLLER_H
#define MY_MUJOCO_SIMULATOR_RT_REMOTE_CONTROLLER_H

#include <thread>
#include <mutex>
#include <linux/joystick.h>
#include "lcm/lcm-cpp.hpp"
#include "../../lcm-types/cpp/rc_lcmt.hpp"

namespace usb_controller {
#define XBOX_BUTTON_A       0x00
#define XBOX_BUTTON_B       0x01
#define XBOX_BUTTON_X       0x03
#define XBOX_BUTTON_Y       0x02
#define XBOX_BUTTON_LB      0x04
#define XBOX_BUTTON_RB      0x05
// MOCUTE
// #define XBOX_BUTTON_START   0x06
// #define XBOX_BUTTON_BACK    0x07
#define XBOX_BUTTON_START   0x07
#define XBOX_BUTTON_SELECT  0x06

// #define XBOX_BUTTON_HOME    0x08

#define XBOX_BUTTON_LO      0x08    // 左侧控制下压
#define XBOX_BUTTON_RO      0x09    //右侧下压

#define XBOX_BUTTON_ON      0x01
#define XBOX_BUTTON_OFF     0x00
//      /\ y
// x    |
// <-----
//
#define XBOX_AXIS_LX        0x00    /* 左摇杆X轴 */
#define XBOX_AXIS_LY        0x01    /* 左摇杆Y轴 */
#define XBOX_AXIS_RX        0x03    /* 右摇杆X轴 */
#define XBOX_AXIS_RY        0x04    /* 右摇杆Y轴 */
#define XBOX_AXIS_LT        0x02
#define XBOX_AXIS_RT        0x05
#define XBOX_AXIS_XX        0x06    /* 方向键X轴 */
#define XBOX_AXIS_YY        0x07    /* 方向键Y轴 */

    typedef struct rc_control_variable_ {
        int mode;
        float p_des[2];
        float height_variation;
        float v_des[3];
        float rpy_des[3];
        float omega_des[3];
        float variables[3]; // variable 0: used to switch gait
        float step_height;
        int stand_flag = 0;
    } rc_control_variable_t;

    typedef enum RC_MODE {
        PASSIVE = 0,
        RL_WALK,
        RL_WALK_2,
        RL_RUN,
        RECOVER_STAND,
        SITDOWN,
        RL_WALK_STAIRS,
        RL_FALL_RECOVER,
        DAMPING,
        USER_INTERFACE,
    } RC_MODE_t;

    typedef struct xbox_map {
        uint32_t time;
        int a;
        int b;
        int x;
        int y;
        int lb;
        int rb;
        int start;
        int back;
        int select;
        int home;
        int lo;
        int ro;

        int lx;
        int ly;
        int rx;
        int ry;
        int lt;
        int rt;
        int xx;
        int yy;

    } xbox_map_t;

    class logic_remote_controller {
    public:
        std::mutex rc_mtx_;
        rc_control_variable_t rc_control_{};
        xbox_map_t rc_map_{};
        bool selected = false;
        int delay_count{};
        struct js_event joystick{};
        int joystick_gait;
        bool print_data_ = false;
        int rc_fd_;
        lcm::LCM rc_LCM;
        rc_lcmt rc_lcmdata{};
        int data_deadzone_width = 600;

        explicit logic_remote_controller(bool print_data = false);

        ~logic_remote_controller() = default;

        ssize_t rc_map_read(int rc_fd, xbox_map_t *map);

        static int rc_open(const char *file_name);

        void rc_close() const;

        ssize_t rc_complete();
    };
}

#endif //MY_MUJOCO_SIMULATOR_RT_REMOTE_CONTROLLER_H
