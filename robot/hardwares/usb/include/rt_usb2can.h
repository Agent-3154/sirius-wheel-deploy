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
#include <mutex>
#include "rt_usb_base.h"

namespace USB_HARDWARE {
    const uint8_t NUMBER_CHIPS = 2;
    const uint16_t usb_motors_in_length = 252;
    const uint16_t usb_motors_out_length = 252;
    const uint16_t usb_motors_in_check_length = usb_motors_in_length / 4 - 1;
    const uint16_t usb_motors_out_check_length = usb_motors_out_length / 4 - 1;
    //todo: Check the size of remote controllers
    // only used for actual robot
#if defined GO1
#define KNEE_OFFSET_POS 4.424f
#define HIP_OFFSET_POS (M_PI/2.f)
#define ABAD_OFFSET_POS (-0.1f)
    const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
    const float hip_side_sign[4] = {1.f, -1.f, 1.f, -1.f};
    const float knee_side_sign[4] = {18.0 / 26.0f, -18.0 / 26.0f, 18.0 / 26.0f, -18.0 / 26.0f};
    const float abad_offset[4] = {ABAD_OFFSET_POS, -ABAD_OFFSET_POS, -ABAD_OFFSET_POS, ABAD_OFFSET_POS}; //
    const float hip_offset[4] = {-HIP_OFFSET_POS + 0.1f, HIP_OFFSET_POS - 0.1f, -HIP_OFFSET_POS + 0.15f,
                                 HIP_OFFSET_POS - 0.15f};
    const float knee_offset[4] = {KNEE_OFFSET_POS - 0.127f * 1.44, -KNEE_OFFSET_POS + 0.127f * 1.44,
                                  KNEE_OFFSET_POS - 0.127f * 1.44, -KNEE_OFFSET_POS + 0.127f * 1.44};
#elif defined CHAOJI_GO
#define KNEE_OFFSET_POS (-3.6114f) //note pos_offset from the motor perspective
#define HIP_OFFSET_POS (1.0275f)
#define ABAD_OFFSET_POS (-0.4802f)
    constexpr float abad_side_sign[4] = {1.f, 1.f, -1.f, -1.f};
    constexpr float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
    constexpr float knee_side_sign[4] = {-10.0 / 14.0f, 10.0 / 14.0f, -10.0 / 14.0f, 10.0 / 14.0f};
    constexpr float abad_offset[4] = {-ABAD_OFFSET_POS, ABAD_OFFSET_POS, ABAD_OFFSET_POS, -ABAD_OFFSET_POS}; //
    constexpr float hip_offset[4] = {HIP_OFFSET_POS, -HIP_OFFSET_POS, HIP_OFFSET_POS, -HIP_OFFSET_POS};
    constexpr float knee_offset[4] = {KNEE_OFFSET_POS, -KNEE_OFFSET_POS, KNEE_OFFSET_POS, -KNEE_OFFSET_POS};
#elif defined DG_ENGINEER
#define KNEE_OFFSET_POS (-3.6114f) //note pos_offset from the motor perspective
#define HIP_OFFSET_POS (1.0275f)
#define ABAD_OFFSET_POS (-0.4802f)
    constexpr float abad_side_sign[4] = {1.f, 1.f, -1.f, -1.f};
    constexpr float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
    constexpr float knee_side_sign[4] = {-10.0 / 14.0f, 10.0 / 14.0f, -10.0 / 14.0f, 10.0 / 14.0f};
    constexpr float abad_offset[4] = {-ABAD_OFFSET_POS, ABAD_OFFSET_POS, ABAD_OFFSET_POS, -ABAD_OFFSET_POS}; //
    constexpr float hip_offset[4] = {HIP_OFFSET_POS, -HIP_OFFSET_POS, HIP_OFFSET_POS, -HIP_OFFSET_POS};
    constexpr float knee_offset[4] = {KNEE_OFFSET_POS, -KNEE_OFFSET_POS, KNEE_OFFSET_POS, -KNEE_OFFSET_POS};
#endif
    typedef struct Leg_Cmd {
        float p_abad_cmd[2];
        float p_hip_cmd[2];
        float p_knee_cmd[2];
        float v_abad_cmd[2];
        float v_hip_cmd[2];
        float v_knee_cmd[2];
        float kp_abad_cmd[2];
        float kp_hip_cmd[2];
        float kp_knee_cmd[2];
        float kd_abad_cmd[2];
        float kd_hip_cmd[2];
        float kd_knee_cmd[2];
        float t_abad_cmd[2];
        float t_hip_cmd[2];
        float t_knee_cmd[2];
        uint32_t leg_flag[1];
    } Leg_Cmd_T;

    typedef struct Leg_Data {
        float p_abad_data[2];
        float p_hip_data[2];
        float p_knee_data[2];
        float v_abad_data[2];
        float v_hip_data[2];
        float v_knee_data[2];
        float t_abad_data[2];
        float t_hip_data[2];
        float t_knee_data[2];
        float uq_abad_data[2];
        float uq_hip_data[2];
        float uq_knee_data[2];
        float ud_abad_data[2];
        float ud_hip_data[2];
        float ud_knee_data[2];
        uint32_t leg_flag[1];
    } Leg_Data_T;

    typedef struct USB_Cmd {
        Leg_Cmd_T leg_cmd[2];
        uint32_t checksum;
    } USB_Cmd_T;

    typedef union USB_CMD {
        USB_Cmd_T usb_cmd;
        uint8_t usb_cmd_buff[usb_motors_out_length];
    } USB_Cmd_U;

    typedef struct USB_Data {
        Leg_Data leg_data[2];
        uint32_t checksum;
    } USB_Data_T;

    typedef union USB_DATA {
        USB_Data_T usb_data;
        uint8_t usb_data_buff[usb_motors_in_length];
    } USB_Data_U;

    class Motor_Control_Board : public USB_Hardware_Base {
    public:
        explicit Motor_Control_Board(uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin, uint8_t _motors_epout);

        ~Motor_Control_Board();

        // data union of this class is a temp buff, data checkok, memcpy to controll databuff.
        void USB2CAN_SetBuffer(USB_Command_t *_control_cmd, USB_Data_t *_controller_data);

        void start_transfer() override;

        void motor_epin_callback(struct libusb_transfer *_transfer);

        void motor_epout_callback(struct libusb_transfer *_transfer);

        std::mutex usb_in_mutex;
        std::mutex usb_out_mutex;

    private:
        // for controller data protocals
        lcm::LCM usb_cmd_LCM;
        lcm::LCM usb_data_LCM;

        USB_Data_U *usb_data_u{};
        USB_Cmd_U *usb_cmd_u{};
        USB_Command_t *control_cmd{};
        USB_Data_t *control_data{};

        usb_command_t *p_usbcmd_lcm;
        usb_data_t *p_usbdata_lcm;

        void Deal_Usb_In_Data();

        void Deal_Usb_Out_Cmd();
    };

    void usb_motors_in_cbf_wrapper(struct libusb_transfer *_transfer);

    void usb_motors_out_cbf_wrapper(struct libusb_transfer *_transfer);
}
#endif //SIRIUS_SOFT_RT_USB_INTERFACE_H
