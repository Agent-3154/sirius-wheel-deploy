//
// Created by lingwei on 4/17/24.
//

#ifndef MY_MUJOCO_SIMULATOR_LEG_CONTROL_H
#define MY_MUJOCO_SIMULATOR_LEG_CONTROL_H

#include "../../utilities/types/hardware_types.h"
#include <mutex>
#include <eigen3/Eigen/StdVector>
#include "../../lcm-types/cpp/leg_control_command_lcmt.hpp"
#include "../../lcm-types/cpp/leg_control_data_lcmt.hpp"
// struct type for one leg
template<typename T>
struct Leg_Control_Command {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3<T> q_des;
    Vec3<T> qd_des;
    Vec3<T> tau_ff;
    Vec3<T> foot_force;
    T whl_q_des;
    T whl_qd_des;
    T whl_tau_ff;
    T whl_kp_joint;
    T whl_kd_joint;
    //foot
    Vec3<T> p_des;
    Vec3<T> v_des;
    Mat3<T> kp_cartisian;
    Mat3<T> kd_cartisian;
    Mat3<T> kp_joint;
    Mat3<T> kd_joint;
};

template<typename T>
struct Leg_Control_Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3<T> q;
    Vec3<T> qd;
    Vec3<T> tau;
    Vec3<T> p;
    Vec3<T> v;
    Mat3<T> J;
    T whl_q;
    T whl_qd;
    T whl_tau;
};

template<typename T>
class Leg_Controller {
public:
    explicit Leg_Controller();

    ~Leg_Controller() = default;

    void Setup_Command(USB_Command_t *usb_cmd);

    void Update_Data(const USB_Data_t *usb_data);

    void Zero_Command();

    void Zero_Data();

    void computeLegJacobianAndPosition(Vec3<T> &q, Mat3<T> *J, Vec3<T> *p, int leg);

    Leg_Control_Command<T> leg_command[4];
    Leg_Control_Data<T> leg_data[4];
    void setLcm(leg_control_data_lcmt* data, leg_control_command_lcmt* command);

private:
    int32_t enable_counter{};
    double link1_, link2_, link3_;
    int nlegs_;
};



#endif //MY_MUJOCO_SIMULATOR_LEG_CONTROL_H
