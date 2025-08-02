//
// Created by lingwei on 4/4/24.
//

#ifndef MY_MUJOCO_SIMULATOR_RT_REMOTE_CONTROLLER_H
#define MY_MUJOCO_SIMULATOR_RT_REMOTE_CONTROLLER_H

#include <thread>
#include <mutex>
#include <linux/joystick.h>
#include <map>
#include <string>
#include "lcm/lcm-cpp.hpp"
#include "../../lcm-types/cpp/rc_lcmt.hpp"

namespace usb_controller {

// Default Xbox button mappings (fallback if no config file is provided)
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

    // Button mapping configuration structure
    struct ButtonMapping {
        std::map<std::string, int> buttons;
        std::map<std::string, int> axes;
        
        // Default constructor with Xbox mappings
        ButtonMapping() {
            // Button mappings
            buttons["A"] = XBOX_BUTTON_A;
            buttons["B"] = XBOX_BUTTON_B;
            buttons["X"] = XBOX_BUTTON_X;
            buttons["Y"] = XBOX_BUTTON_Y;
            buttons["LB"] = XBOX_BUTTON_LB;
            buttons["RB"] = XBOX_BUTTON_RB;
            buttons["START"] = XBOX_BUTTON_START;
            buttons["SELECT"] = XBOX_BUTTON_SELECT;
            buttons["LO"] = XBOX_BUTTON_LO;
            buttons["RO"] = XBOX_BUTTON_RO;
            
            // Axis mappings
            axes["LX"] = XBOX_AXIS_LX;
            axes["LY"] = XBOX_AXIS_LY;
            axes["RX"] = XBOX_AXIS_RX;
            axes["RY"] = XBOX_AXIS_RY;
            axes["LT"] = XBOX_AXIS_LT;
            axes["RT"] = XBOX_AXIS_RT;
            axes["XX"] = XBOX_AXIS_XX;
            axes["YY"] = XBOX_AXIS_YY;
        }
        
        // Get button number by name
        int getButton(const std::string& name) const {
            auto it = buttons.find(name);
            return (it != buttons.end()) ? it->second : -1;
        }
        
        // Get axis number by name
        int getAxis(const std::string& name) const {
            auto it = axes.find(name);
            return (it != axes.end()) ? it->second : -1;
        }
    };

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

    class LogicRemoteController {
    public:
        std::string device_file_;
        bool print_data_ = false;
        std::string config_file_;  // Path to button mapping configuration file
        ButtonMapping button_mapping_;  // Button mapping configuration
        
        std::mutex rc_mtx_;
        rc_control_variable_t rc_control_{};
        xbox_map_t rc_map_{};
        bool selected = false;
        int delay_count{};
        struct js_event joystick{};
        int joystick_gait;
        int rc_fd_;
        lcm::LCM rc_LCM;
        rc_lcmt rc_lcmdata{};
        int data_deadzone_width = 600;

        explicit LogicRemoteController(
            const std::string &device_file="/dev/input/js0",
            bool print_data = false,
            const std::string &config_file = ""
        );

        ~LogicRemoteController() = default;

        ssize_t rc_map_read(int rc_fd, xbox_map_t *map);

        static int rc_open(const char *file_name);

        void rc_close() const;

        ssize_t rc_complete();
        
        // Load button mapping configuration from YAML file
        bool loadButtonMapping(const std::string& config_file);
        
        // Get button mapping (for debugging/testing)
        const ButtonMapping& getButtonMapping() const { return button_mapping_; }
    };
}

#endif //MY_MUJOCO_SIMULATOR_RT_REMOTE_CONTROLLER_H
