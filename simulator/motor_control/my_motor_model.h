//
// Created by lingwei on 5/13/24.
//

#ifndef MY_MUJOCO_SIMULATOR_MY_MOTOR_MODEL_H
#define MY_MUJOCO_SIMULATOR_MY_MOTOR_MODEL_H

#include "../../utilities/types/hardware_types.h"
#include <eigen3/Eigen/Sparse>

namespace Motor_Control {
    class Motor_Model {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        explicit Motor_Model(const std::string &motor_file_);

        ~Motor_Model() = default;

        [[nodiscard]] float get_torque(int id_) const;

        void GetSettings(const std::string &filename, const std::string &setting_name);

        void pack_motor_cmd(const USB_Command_t *usb_cmd, const USB_Data_t *usb_data);

        void set_motor_kp_kd(float kp, float kd);

    private:
        Eigen::Matrix<float, 16, 16> kp_mat_;
        Eigen::Matrix<float, 16, 16> kd_mat_;
        Vec16<float> torq_out_;

        Vec16<float> q_data_;
        Vec16<float> qd_data_;
        Vec16<float> q_cmd_;
        Vec16<float> qd_cmd_;
        Vec16<float> t_ff_;
    };
}

#endif //MY_MUJOCO_SIMULATOR_MY_MOTOR_MODEL_H
