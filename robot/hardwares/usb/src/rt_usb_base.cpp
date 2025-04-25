//
// Created by lingwei on 5/10/24.
//
#include "rt_usb_base.h"
#include "iostream"
#include "../../utilities/types/std_cout_colors.h"

USB_HARDWARE::USB_Hardware_Base::USB_Hardware_Base(const std::string &usb_name, uint16_t vendor_id, uint16_t product_id,
                                                   uint8_t epin, uint8_t epout) : vendor_id_(vendor_id),
                                                                                  product_id_(product_id), epin_(epin),
                                                                                  epout_(epout) {
    transfer_tx = libusb_alloc_transfer(0);
    transfer_rx = libusb_alloc_transfer(0);

    libusb_init(&ctx);

    deviceHandle = libusb_open_device_with_vid_pid(ctx, vendor_id, product_id);
    if (deviceHandle == nullptr) {
        std::cout << RED << "[" << usb_name << " ERROR]: " << RESET << "Can not open " << usb_name << "\n";
    }
    if (libusb_kernel_driver_active(deviceHandle, 0x00)) {
        int success = libusb_detach_kernel_driver(deviceHandle, 0x00);
        if (success != 0) {
            std::cout << RED << "[" << usb_name << " ERROR]: " << RESET << "Detach Driver Failed!" << std::endl;
            std::abort();
        }
    }
    int claim_interface = libusb_claim_interface(deviceHandle, 0x00);
    if (claim_interface != 0) {
        std::cout << RED << "[" << usb_name << " ERROR]: " << RESET << "Claim Driver Failed!" << std::endl;
        std::abort();
    }
    std::cout << GREEN << "[" << usb_name << "]: " << RESET << "INITIALIZATION SUCCESS!" << std::endl;
}

USB_HARDWARE::USB_Hardware_Base::~USB_Hardware_Base() {
    libusb_free_transfer(transfer_tx);
    libusb_free_transfer(transfer_rx);
    libusb_release_interface(deviceHandle, 0);
    libusb_close(deviceHandle);
    libusb_exit(nullptr);
}

[[noreturn]] void USB_HARDWARE::USB_Hardware_Containers::run_usb_hardwares() {
    while (true) {
        for (auto usb: usb_container_) {
            libusb_handle_events(usb->ctx);
        }
    }

}

uint32_t USB_HARDWARE::data_checksum(const uint32_t *data_to_check, uint32_t check_length) {
    uint32_t t = 0;
    for (uint32_t i = 0; i < check_length; i++) {
        t = t ^ data_to_check[i];
    }
    return t;
}