//
// Created by lingwei on 5/10/24.
//

#ifndef MY_MUJOCO_SIMULATOR_RT_USB_BASE_H
#define MY_MUJOCO_SIMULATOR_RT_USB_BASE_H

#include "libusb-1.0/libusb.h"
#include "string"
#include "vector"

namespace USB_HARDWARE {
    class USB_Hardware_Base {
    public:
        USB_Hardware_Base(const std::string &usb_name, uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin,
                          uint8_t _motors_epout);

        virtual ~USB_Hardware_Base();

        virtual void start_transfer() = 0;

        libusb_context *ctx = nullptr;
    protected:
        libusb_transfer *transfer_tx = nullptr;
        libusb_transfer *transfer_rx = nullptr;
        libusb_device_handle *deviceHandle = nullptr;
        uint16_t vendor_id_;
        uint16_t product_id_;
        uint8_t epin_;
        uint8_t epout_;
    };

    class USB_Hardware_Containers {
    public:
        USB_Hardware_Containers() = default;

        ~USB_Hardware_Containers() = default;

        std::vector<USB_Hardware_Base *> usb_container_;

        [[noreturn]] void run_usb_hardwares();

        void add_usb_device(USB_Hardware_Base *device) {
            usb_container_.push_back(device);
        }
    };

    uint32_t data_checksum(const uint32_t *data_to_check, uint32_t check_length);

}

#endif //MY_MUJOCO_SIMULATOR_RT_USB_BASE_H
