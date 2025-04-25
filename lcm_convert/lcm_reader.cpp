#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "../lcm-types/cpp/usb_data_t.hpp"
#include "../lcm-types/cpp/usb_command_t.hpp"
#include "../utilities/inc/easylogging++.h"
#include <iostream>
#include <fstream>
#include <iomanip>

INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv) {
    // the offset on real robot
    constexpr float abad_side_sign[4] = {1.f, 1.f, -1.f, -1.f};
    constexpr float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
    constexpr float knee_side_sign[4] = {-10.0 / 14.0f, 10.0 / 14.0f, -10.0 / 14.0f, 10.0 / 14.0f};
    // log file cmd
    std::fstream read_file_cmd_q;
    std::fstream read_file_cmd_qd;
    std::fstream read_file_cmd_tau;

    // log file data
    std::fstream read_file_data_q;
    std::fstream read_file_data_qd;
    std::fstream read_file_data_tor;
    std::fstream read_file_data_uq;
    std::fstream read_file_data_ud;

    read_file_cmd_q.open("motor_cmd_q.txt", std::ios::out | std::ios::trunc);
    read_file_cmd_qd.open("motor_cmd_qd.txt", std::ios::out | std::ios::trunc);
    read_file_cmd_tau.open("motor_cmd_tau.txt", std::ios::out | std::ios::trunc);

    read_file_data_q.open("motor_data_q.txt", std::ios::out | std::ios::trunc);
    read_file_data_qd.open("motor_data_qd.txt", std::ios::out | std::ios::trunc);
    read_file_data_tor.open("motor_data_tor.txt", std::ios::out | std::ios::trunc);
    read_file_data_uq.open("motor_data_uq.txt", std::ios::out | std::ios::trunc);
    read_file_data_ud.open("motor_data_ud.txt", std::ios::out | std::ios::trunc);


    if (argc < 2) {
        LOG(ERROR) << "Usage: " << argv[0] << " read_log";
        return 1;
    }

    lcm::LogFile log(argv[1], "r");
    if (!log.good()) {
        LOG(ERROR) << "Unable to open log file: " << argv[1];
        return 1;
    }

    while (true) {
        const lcm::LogEvent *event = log.readNextEvent();
        if (!event) {
            break;
        }
        if (event->channel == "MOTOR DATA") {
            usb_data_t msg{};
            if (msg.decode(event->data, 0, event->datalen) != event->datalen)
                continue;
            // decode successful
            for (int i = 0; i < 4; i++) {
                read_file_data_q << std::setw(12) << std::setfill(' ') << msg.q_abad[i] / abad_side_sign[i] << " " <<
                        std::setw(12) << std::setfill(' ') << msg.q_hip[i] / hip_side_sign[i] << " " << std::setw(12) <<
                        std::setfill(' ') << msg.q_knee[i] / knee_side_sign[i] << " ";
                read_file_data_qd << std::setw(12) << std::setfill(' ') << msg.qd_abad[i] / abad_side_sign[i] << " " <<
                        std::setw(12) << std::setfill(' ') << msg.qd_hip[i] / hip_side_sign[i] << " " << std::setw(12)
                        << std::setfill(' ') << msg.qd_knee[i] / knee_side_sign[i] << " ";
                read_file_data_tor << std::setw(12) << std::setfill(' ') << msg.tau_abad[i] * abad_side_sign[i] << " " << std::setw(12) <<
                        std::setfill(' ') << msg.tau_hip[i] * hip_side_sign[i] << " " << std::setw(12) << std::setfill(' ') << msg.tau_knee
                        [i] * knee_side_sign[i] << " ";
                read_file_data_uq << std::setw(12) << std::setfill(' ') << msg.uq_abad[i] << " " << std::setw(12) << std::setfill(' ') <<
                        msg.uq_hip[i] << " " << std::setw(12) << std::setfill(' ') << msg.uq_knee[i] << " ";
                read_file_data_ud << std::setw(12) << std::setfill(' ') << msg.ud_abad[i] << " " << std::setw(12) << std::setfill(' ') <<
                        msg.ud_hip[i] << " " << std::setw(12) << std::setfill(' ') << msg.ud_knee[i] << " ";
            }
            read_file_data_q << std::endl;
            read_file_data_qd << std::endl;
            read_file_data_tor << std::endl;
            read_file_data_uq << std::endl;
            read_file_data_ud << std::endl;
        } else if (event->channel == "MOTOR COMMAND") {
            usb_command_t msg{};
            if (msg.decode(event->data, 0, event->datalen) != event->datalen)
                continue;
            // decode successful
            for (int i = 0; i < 4; i++) {
                read_file_cmd_q << std::setw(12) << std::setfill(' ') << msg.q_des_abad[i] << " " << std::setw(12) << std::setfill(' ') << msg.
                        q_des_hip[i] << " " << std::setw(12) << std::setfill(' ') << msg.q_des_knee[i] << " ";
                read_file_cmd_qd << std::setw(12) << std::setfill(' ') << msg.qd_des_abad[i] / abad_side_sign[i] << " " << std::setw(12) <<
                        std::setfill(' ') << msg.qd_des_hip[i] / hip_side_sign[i] << " " << std::setw(12) << std::setfill(' ') << msg.qd_des_knee[i] /
                        knee_side_sign[i] << " ";
                read_file_cmd_tau << std::setw(12) << std::setfill(' ') << msg.tau_abad_ff[i] * abad_side_sign[i] << " " << std::setw(12) <<
                        std::setfill(' ') << msg.tau_hip_ff[i] * hip_side_sign[i] << " " << std::setw(12) << std::setfill(' ') << msg.tau_knee_ff[i] *
                        knee_side_sign[i] <<
                        " ";
            }
            read_file_cmd_q << std::endl;
            read_file_cmd_qd << std::endl;
            read_file_cmd_tau << std::endl;
        }
    }

    read_file_cmd_q.close();
    read_file_cmd_qd.close();
    read_file_cmd_tau.close();
    read_file_data_q.close();
    read_file_data_qd.close();
    read_file_data_tor.close();
    read_file_data_ud.close();
    read_file_data_uq.close();
    return 0;
}
