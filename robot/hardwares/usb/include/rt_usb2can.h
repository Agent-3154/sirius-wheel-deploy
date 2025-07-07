//
// Created by lingwei on 3/26/24.
//

#ifndef PROJECT_RT_USB2CAN_H
#define PROJECT_RT_USB2CAN_H
#include <cstdint>
#include "libusb-1.0/libusb.h"
#include "../../utilities/types/hardware_types.h"
#include "../../lcm-types/cpp/usb_command_t.hpp"
#include "../../lcm-types/cpp/usb_data_t.hpp"
#include "lcm/lcm-cpp.hpp"
#include <shared_mutex>

#include "rt_usb_base.h"

namespace USB_HARDWARE {
    constexpr int NUMBER_CHIPS = 3;
    constexpr int usb_motors_in_length = 376;
    constexpr int usb_motors_out_length = 376;
    constexpr int usb_motors_in_check_length = usb_motors_in_length / 4 - 1;
    constexpr int usb_motors_out_check_length = usb_motors_out_length / 4 - 1;
    //todo: Check the size of remote controllers
    // only used for actual robot

#define KNEE_OFFSET_POS (M_PI - 0.18f) //note pos_offset from the motor perspective
#define HIP_OFFSET_POS (M_PI / 2.f - 0.411f)
// #define HIP_OFFSET_POS (M_PI / 2.f)
#define ABAD_OFFSET_POS (-0.39f)
// #define ABAD_OFFSET_POS (-0.0f)
#define NUM_LEG_MOTORS 12
#define NUM_WHEEL_MOTORS 4
#define TIMING_RATIO 2.0f
    // can5 the same as can1
    constexpr float leg_side_sign[18] = {
        1.f, -1.f, 1.f / TIMING_RATIO,
        1.f, 1.f, -1.f / TIMING_RATIO,
        -1.f, -1.f, 1.f / TIMING_RATIO,
        -1.f, 1.f, -1.f / TIMING_RATIO,
        1.f, -1.f, 1.f / TIMING_RATIO,
        -1.f, 1.f, 1.f
    };
    constexpr float whl_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
    constexpr float leg_offset[18] = {
        -ABAD_OFFSET_POS, HIP_OFFSET_POS, KNEE_OFFSET_POS * TIMING_RATIO,
        ABAD_OFFSET_POS, -HIP_OFFSET_POS, -KNEE_OFFSET_POS * TIMING_RATIO,
        ABAD_OFFSET_POS, -HIP_OFFSET_POS, -KNEE_OFFSET_POS * TIMING_RATIO,
        -ABAD_OFFSET_POS, HIP_OFFSET_POS, KNEE_OFFSET_POS * TIMING_RATIO,
        -ABAD_OFFSET_POS, HIP_OFFSET_POS, KNEE_OFFSET_POS * TIMING_RATIO,
        1.f, 1.f, 1.f
    };
    // const float whl_offset[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    typedef struct USB_CMD_Pack {
        float p_cmd_;
        float v_cmd_;
        float kp_;
        float kd_;
        float t_ff_;
    } USB_CMD_PACK_T;

    typedef struct USB_CHIP_CMD {
        USB_CMD_PACK_T cmd_pack[6];
        uint32_t chip_flag[1];
    } USB_CHIP_CMD_T;

    typedef struct USB_DATA_Pack {
        float p_data_;
        float v_data_;
        float t_data_;
        float uq_;
        float ud_;
    } USB_DATA_PACK_T;

    typedef struct USB_CHIP_DATA {
        USB_DATA_PACK_T data_pack[6];
        uint32_t chip_flag[1];
    } USB_CHIP_DATA_T;

    typedef struct Beast_USB_CMD {
        USB_CHIP_CMD_T usb_chip_cmd_[NUMBER_CHIPS];
        uint32_t checksum;
    } Beast_USB_Cmd_T;

    typedef union USB_CMD_UNION {
        Beast_USB_Cmd_T usb_cmd_;
        uint8_t usb_cmd_buff[usb_motors_out_length];
    } USB_Cmd_U;

    typedef struct Beast_USB_DATA {
        USB_CHIP_DATA_T usb_chip_data_[NUMBER_CHIPS];
        uint32_t checksum;
    } Beast_USB_Data_T;

    typedef union USB_DATA_UNION {
        Beast_USB_Data_T usb_data_;
        uint8_t usb_data_buff[usb_motors_in_length];
    } USB_Data_U;

    class Beast_USB2CAN : public USB_Hardware_Base {
    public:
        explicit Beast_USB2CAN(uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin,
                               uint8_t _motors_epout);

        ~Beast_USB2CAN() override;

        // data union of this class is a temp buff, data checkok, memcpy to controll databuff.
        void USB2CAN_SetBuffer(USB_Command_t *_control_cmd, USB_Data_t *_controller_data);

        void start_transfer() override;

        void motor_epin_callback(struct libusb_transfer *_transfer);

        void motor_epout_callback(struct libusb_transfer *_transfer);

        std::shared_mutex usb_shared_in_mutex;
        std::shared_mutex usb_shared_out_mutex;

        static double clampMinMax(double value, double min, double max) {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

    private:
        // for controller data protocals
        lcm::LCM usb_cmd_LCM;
        lcm::LCM usb_data_LCM;

        USB_Data_U *usb_data_u{};
        USB_Cmd_U *usb_cmd_u{};
        // For controllers
        USB_Command_t *control_cmd_serial{};
        USB_Data_t *control_data_serial{};
        USB_Data_t *control_data_serial_offset{};
        // For data debug and lcm
        USB_Command_t *control_cmd_serial_offset{};

        usb_command_t *p_usbcmd_serial_lcmdata;
        usb_data_t *p_usbdata_serial_lcmdata;
        usb_command_t *p_usbcmd_diff_lcmdata;
        usb_data_t *p_usbdata_diff_lcmdata;

        void Deal_Usb_In_Data();

        void Deal_Usb_Out_Cmd();
    };

    void usb_motors_in_cbf_wrapper(struct libusb_transfer *_transfer);

    void usb_motors_out_cbf_wrapper(struct libusb_transfer *_transfer);
}
#endif //PROJECT_RT_USB2CAN_H
