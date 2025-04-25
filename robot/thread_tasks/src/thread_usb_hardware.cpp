//
// Created by lingwei on 5/10/24.
//
#include "thread_usb_hardware.h"
#include "../../utilities/types/std_cout_colors.h"
#include <iostream>
#include <utility>
#include <sys/time.h>
#include <sys/select.h>

Thread::thread_usb_hardwares::thread_usb_hardwares(std::string task_name, int task_frequency) : thread_timer(std::move(task_name),
                                                                                                             task_frequency) {

}

void Thread::thread_usb_hardwares::thread_loop(USB_HARDWARE::USB_Hardware_Containers *handle) {
//    std::cout << "Enter this thread\n";

    struct timeval timestruc{};
    timestruc.tv_sec = 0;
    timestruc.tv_usec = 0; // return immediately
    int compelte = 0;

    for(auto usb_device : handle->usb_container_)
    {
        usb_device->start_transfer();
    }

    std::cout << GREEN << "[Thread USB hardware OK]: " << RESET << "Initialize usb hardware thread!\n";
    while (true) {
        for(auto usb_device : handle->usb_container_)
        {
            libusb_handle_events_timeout_completed(usb_device->ctx, &timestruc, &compelte);
        }
    }
}

