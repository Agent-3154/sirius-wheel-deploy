//
// Created by lingwei on 6/8/24.
//

#ifndef LINEAR_PLANNER_H
#define LINEAR_PLANNER_H

#include <utility>

#include "planner_base.h"

class Linear_Planner : public Planner_Base<double> {
public:
    Linear_Planner(const double dt, const int swing_segment) : Planner_Base(dt, swing_segment) {
        R_z.resize(3, 3);
        Pin_Rz.resize(3, 3);
        Foot_Contact_Point.resize(4,3);
        R_z.setZero();
        Pin_Rz.setZero();
        Foot_Contact_Point.setZero();
    };

    ~Linear_Planner() override = default;

    void run(Control_FSM_Data &data) override;

    void SetupCommand(const Control_FSM_Data &data) override;

    void set_lcm() override;

    void set_start_rpy(Vec3<double> start_rpy) { rpy_des_w_ = std::move(start_rpy); }

    [[nodiscard]] const int *get_contact_table() const { return contact_table_; }
    std::atomic_bool update_{};
    Eigen::MatrixXd Foot_Contact_Point;
    Eigen::Vector3d norm_vec;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_z;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Pin_Rz;
    Eigen::Vector3d temp_norm_vec;
};

#endif //LINEAR_PLANNER_H
