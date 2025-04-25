//
// Created by lingwei on 6/8/24.
//
#include "linear_planner.h"
#include <easylogging++.h>
#include <std_cout_colors.h>
#include "Control_FSM_Data.h"
#include "iostream"


void Linear_Planner::run(Control_FSM_Data &data) {
    // get velocity command w.r.t base frame
    SetupCommand(data);
    auto p_w = data.estimators_->get_result_world_position();
    auto v_w = data.estimators_->get_result_world_velocity();
    const auto quat = data.estimators_->get_result_quat();
    const Mat3<double> r_b_transpose = data.estimators_->shared_esti_data_.result_->r_b_.transpose();
    const Vec3<double> rpy = ori::quatToRPY(quat);
    world_pos_ = p_w;
    rpy_vel_des_w_ = r_b_transpose * rpy_vel_des_b_;
    // rpy_des_w_(2) = rpy(2) + dt_ * rpy_vel_des_w_(2);1
    rpy_des_w_(2) += dt_ * rpy_vel_des_w_(2);
    if (rpy_des_w_(2) > M_PI) {
        rpy_des_w_(2) -= 2 * M_PI;
    } else if (rpy_des_w_(2) < -M_PI) {
        rpy_des_w_(2) += 2 * M_PI;
    }
    // std::cout << "x_vel_des: " << x_vel_des_ << " | y_vel_des_: " << dt_ * yaw_turn_rate_ << " | yaw_des: " << yaw_des_ <<
    // std::endl << "here: " << rpy_temp[2] << std::endl;
    // rpy_des_w_(0) = rpy_des_w_(1) = 0;

    for (int i = 0; i < 4; i++) {
        desired_.pFoot_[i] = p_w + r_b_transpose *
                             (data.quadruped_model_->getHipLocation(i) + data.leg_controller_->leg_data[i].p);
        // std::cout << "Foot id: " << i << "| " << desired_.pFoot_[i].transpose() << std::endl;
    }

    // first run initialization.
    if (firstRun_) {
        iterCounter_ = 0;
        LOG(INFO) << "First Run Initialization!";
        world_position_desired_[0] = p_w[0];
        world_position_desired_[1] = p_w[1];
        world_position_desired_[2] = Config::mpc_height;
        for (int i = 0; i < 4; i++) {
            footSwingTrajectories_[i].setHeight(Config::step_height);
            footSwingTrajectories_[i].setInitialPosition(desired_.pFoot_[i]);
            footSwingTrajectories_[i].setFinalPosition(desired_.pFoot_[i]);
            // footSwingTrajectories_[i].computeTrajectory(0,0);
            footSwingTrajectories_[i].setPosition(desired_.pFoot_[i]);
        }
        pw_ini_ = p_w;
        rpy_ini_ = rpy;
        firstRun_ = false;
    }

    // switch gait
    if ((gaitNumber_ == STAND && currentGait_ != STAND) || firstRun_) {
        stand_traj_[0] = p_w[0];
        stand_traj_[1] = p_w[1];
        stand_traj_[2] = Config::mpc_height;
        stand_traj_[3] = 0;
        stand_traj_[4] = 0;
        stand_traj_[5] = rpy[2];
        world_position_desired_[0] = p_w[0];
        world_position_desired_[1] = p_w[1];
    }

    if (gaitNumber_ == STAND && currentGait_ != STAND) {
        std::cout << BLUE << "[Gait Switch]: " << RESET << "Stand Gait\n";
        iterCounter_ = 0;
        pw_ini_ = p_w;
        rpy_ini_ = rpy;
        work_gait_ = &stand_;
    } else if (gaitNumber_ == TROT && currentGait_ != TROT) {
        std::cout << BLUE << "[Gait Switch]: " << RESET << "Trot Gait\n";
        iterCounter_ = 0;
        work_gait_ = &trotting_;
    } else if (gaitNumber_ == WALK && currentGait_ != WALK) {
        std::cout << BLUE << "[Gait Switch]: " << RESET << "Walk Gait\n";
        iterCounter_ = 0;
        work_gait_ = &walk_;
    } else if (gaitNumber_ == RUNNING && currentGait_ != RUNNING) {
        std::cout << BLUE << "[Gait Switch]: " << RESET << "Running Gait\n";
        iterCounter_ = 0;
        work_gait_ = &running_;
    }
    currentGait_ = gaitNumber_;
    // reset iterations if gait change.
    work_gait_->setIterations(swing_segment_, static_cast<int>(iterCounter_));

    const Vec3<double> v_b_des(x_vel_des_, y_vel_des_, 0);
    Vec3<double> v_w_des = r_b_transpose * v_b_des;
    // what is this compensation?
    if (fabs(v_w[0]) > 0.2) {
        desired_.rpy_int_[1] += dt_ * (rpy_des_b_(1) - rpy[1]) / v_w[0];
    }
    if (fabs(v_w[1]) > 0.1) {
        desired_.rpy_int_[0] += dt_ * (rpy_des_b_(0) - rpy[0]) / v_w[1];
    }
    desired_.rpy_int_[0] = fmin(fmax(desired_.rpy_int_[0], -.25), .25);
    desired_.rpy_int_[1] = fmin(fmax(desired_.rpy_int_[1], -.25), .25);
    desired_.rpy_comp_[1] = v_w[0] * desired_.rpy_int_[1];
    desired_.rpy_comp_[0] = v_w[1] * desired_.rpy_int_[0] * (gaitNumber_ != PRONKING); //turn off for pronking
    // std::cout << "rpy_comp: " << rpy_comp_[0] << " | " << rpy_comp_[1] << std::endl;
    // check ok

    if (work_gait_ != &stand_) {
        world_position_desired_ += dt_ * Vec3<double>(v_w_des[0], v_w_des[1], 0);
    }
    // get swing time
    // std::cout <<"Check Swing time:\n";
    for (int i = 0; i < 4; i++) {
        swingTime_[i] = work_gait_->getCurrentSwingTime(dtFoot_, i);
        // std::cout << swingTime_[i] << " | ";
    }
    // std::cout << std::endl;

    swingStates = work_gait_->getSwingState(); //get swing process
    for (uint8_t i = 0; i < 4; i++) {
        if (swingStates[i] == 0) {
            Foot_Contact_Point.row(i) = desired_.pFoot_[i];
        }
    }

    double d = 0;
    Eigen::RowVector3d centroid = Foot_Contact_Point.colwise().mean();
    Eigen::MatrixXd demean = Foot_Contact_Point;
    demean.rowwise() -= centroid;
    // LOG(INFO) << Foot_Contact_Point;
    Eigen::JacobiSVD svd(demean, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV();
    //Eigen::MatrixXf U = svd.matrixU();
    //Eigen::Matrix3f S = U.inverse() * demean * V.transpose().inverse();
    temp_norm_vec << V(0, 2), V(1, 2), V(2, 2);
    norm_vec = 0.8 * norm_vec + 0.2 * temp_norm_vec;
    d = -norm_vec.transpose() * centroid.transpose();
    R_z << cos(rpy_des_w_[2]), sin(rpy_des_w_[2]), 0, sin(rpy_des_w_[2]), -cos(rpy_des_w_[2]), 0, 0, 0, 1;
    pseudoInverse(R_z, 0.001, Pin_Rz);
    Vec3<double> vec_compute_rp = Pin_Rz * norm_vec;
    double roll = asin(vec_compute_rp(1));
    double pitch = atan(vec_compute_rp(0) / vec_compute_rp(2));
    rpy_des_w_(0) = roll * Config::planner_rpy_filter + rpy_des_w_last_(0) * (1 - Config::planner_rpy_filter);
    rpy_des_w_(1) = pitch * Config::planner_rpy_filter + rpy_des_w_last_(1) * (1 - Config::planner_rpy_filter);

    // rpy_des_w_(0) = abs(rpy_des_w_(0)) < 0.04 ? 0 : rpy_des_w_(0);
    // rpy_des_w_(1) = abs(rpy_des_w_(1)) < 0.04 ? 0 : rpy_des_w_(1);
    rpy_des_w_last_(0) = rpy_des_w_(0);
    rpy_des_w_last_(1) = rpy_des_w_(1);
    // _roll_des = temp_roll_des;
    // _pitch_des = temp_pitch_des;
    // std::cout << "\nroll des: " << rpy_des_w_(0) << " pitch des: " << rpy_des_w_(1) << " Yaw des: " << rpy_des_w_(2) << "\n";

    // prepare data for foot scheduler.
    const double abad_link_length = data.quadruped_model_->abadLinkLength_;
    constexpr double side_sign[4] = {-1., 1., -1., 1.};
    // constexpr double interleave_y[4] = {-0.08, 0.08, 0.02, -0.02}; // what's this?
    // constexpr double interleave_gain = -0.2;
    // double v_abs = std::fabs(v_b_des[0]);
    for (int i = 0; i < 4; i++) {
        if (firstSwing_[i]) {
            swingTimeRemained_[i] = swingTime_[i];
        } else {
            swingTimeRemained_[i] -= dt_;
        }

        footSwingTrajectories_[i].setHeight(Config::step_height);

        Vec3<double> offset_hip(0, side_sign[i] * abad_link_length, 0);
        // Foot Next Point Scheduler
        // NOTE: the hip location is abad location, but foot point schedule is based on real hip location.
        // TODO: Draw this pRobotFrame in the simulation!
        Vec3<double> pRobotFrame = data.quadruped_model_->getHipLocation(i) + offset_hip;
        const double stance_time = work_gait_->getCurrentStanceTime(dtFoot_, i);
        Vec3<double> pYaw = coordinateRotation(ori::CoordinateAxis::Z, -rpy_vel_des_b_(2) * stance_time / 2.) *
                            pRobotFrame;

        // Start Raibert's Compensation
        pf_[i] = p_w + r_b_transpose * (pYaw + v_b_des * swingTimeRemained_[i]);

        // Vec3<double> foot_debug = r_b_transpose * (pYaw_w + des_vel * swingTimeRemained_[i]);
        // std::cout << "Leg id: " << i << foot_debug.transpose() << std::endl;

        // this schedule is w.r.t. the world frame
        constexpr double p_rel_max = Config::foot_reference_max;
        double pfx_rel = v_w[0] * (0.5 + Config::mpc_bonus_swing_x) * stance_time + Config::gain_comp_3 * (
                             v_w[0] - v_w_des[0]) + (Config::gain_comp_4 * p_w[2] / Config::G) * (
                             v_w[1] * rpy_vel_des_w_(2));
        // double v_w1_clipped = v_w[1] > 0.01 ? v_w[1] : 0; // not work
        double pfy_rel = v_w[1] * 0.5 * stance_time + Config::gain_comp_3 * (v_w[1] - v_w_des[1]) +
                         (Config::gain_comp_4 * p_w[2] / Config::G) * (-v_w[0] * rpy_vel_des_w_(2));
        // double pfy_rel = v_w1_clipped * 0.5 * stance_time + Config::gain_comp_3 * (v_w1_clipped - v_w_des[1]) +
        //                  (Config::gain_comp_4 * p_w[2] / Config::G) * (-v_w[0] * rpy_vel_des_w_(2));
        lcm_data_.pf_rel[i][0] = static_cast<float>(pfx_rel);
        lcm_data_.pf_rel[i][1] = static_cast<float>(pfy_rel);
        if (i == 3) {
            lcm_data_.parse_data[0] = static_cast<float>(v_w[1] * 0.5 * stance_time);
            lcm_data_.parse_data[1] = static_cast<float>(Config::gain_comp_3 * (v_w[1] - v_w_des[1]));
            lcm_data_.parse_data[2] = static_cast<float>((Config::gain_comp_4 * p_w[2] / Config::G) * (-v_w[0] * rpy_vel_des_w_(2)));
        }
        pfx_rel = fmin(fmax(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fmin(fmax(pfy_rel, -p_rel_max), p_rel_max);
        pf_[i][0] += pfx_rel;
        pf_[i][1] += pfy_rel;
        pf_[i][2] = -(norm_vec(0) * pf_[i][0] + norm_vec(1) * pf_[i][1] + d) / norm_vec(2) + Config::pf_z;
        // this one seems ok, estimator is shifting.
        footSwingTrajectories_[i].setFinalPosition(pf_[i]);
        pf_ini_[i] = pf_[i];
        // std::cout << "Pf: id" << i << " | " << pf_[i].transpose() << std::endl;
    }
    // std::cout << "\n";
    // solve the mpc
    contact_state_ = work_gait_->getContactState();

    // clip zero if in stand gait
    if (currentGait_ == STAND && contact_state_.isZero(0)) {
        // std::cout << "Trigger Offset!\n";
        for (int i = 0; i < 4; i++) {
            contact_state_(i) += work_gait_->phase_segment_;
        }
    }

    // std::cout << "swing states: " << swingStates.transpose() << std::endl;

    Mat3<double> kp_cartesian;
    kp_cartesian << Config::kp_cartesian, 0, 0, 0, Config::kp_cartesian, 0, 0, 0, Config::kp_cartesian_z;
    Mat3<double> kd_cartesian;
    kd_cartesian << Config::kd_cartesian, 0, 0, 0, Config::kd_cartesian, 0, 0, 0, Config::kd_cartesian;
    // schedule foot after updating mpc

    data.leg_controller_->Zero_Command();
    // std::cout << "Initial pos:\n";// zero cmd in case using mpc directly
    for (int i = 0; i < 4; i++) {
        double swingState = swingStates[i];
        if (swingState > 0) {
            if (firstSwing_[i]) {
                firstSwing_[i] = false;
                desired_.pFoot_[i](2) = 0;
                footSwingTrajectories_[i].setInitialPosition(desired_.pFoot_[i]);
                // std::cout << RED << desired_.pFoot_[i].transpose() << RESET << std::endl;
            }
            // the swingTime_ is the total swing time.
            // std::cout << "swing state:: " << swingState << std::endl;
            footSwingTrajectories_[i].computeTrajectory(swingState, swingTime_[i]);
            Vec3<double> pDesFoot_w = footSwingTrajectories_[i].getPosition();
            Vec3<double> vDesFoot_w = footSwingTrajectories_[i].getVelocity();
            // get pFoot reference w.r.t. abad frame
            const Vec3<double> pDesLeg = data.estimators_->shared_esti_data_.result_->r_b_ * (pDesFoot_w - p_w) -
                                         data.quadruped_model_->getHipLocation(i);
            const Vec3<double> vDesLeg = data.estimators_->shared_esti_data_.result_->r_b_ * (vDesFoot_w - v_w);

            if (!use_wbc_) {
                desired_.pFoot_des_[i] = pDesFoot_w;
                desired_.vFoot_des_[i] = vDesFoot_w;
                data.leg_controller_->leg_command[i].p_des = pDesLeg;
                data.leg_controller_->leg_command[i].v_des = vDesLeg;
                data.leg_controller_->leg_command[i].kp_cartisian = kp_cartesian;
                data.leg_controller_->leg_command[i].kd_cartisian = kd_cartesian;
                data.leg_controller_->leg_command[i].kp_joint.setZero();
                data.leg_controller_->leg_command[i].kd_joint.setZero();
            } else {
                desired_.pFoot_des_[i] = pDesFoot_w;
                desired_.vFoot_des_[i] = vDesFoot_w;
                desired_.aFoot_des_[i] = footSwingTrajectories_[i].getAcceleration();
                // std::cout << "leg id: " << i << " pFoot: " << pDesFoot_w.transpose() << " vFoot: " <<
                //     vDesFoot_w.transpose() << " aFoot: " << desire_data_.aFoot_des_[i].transpose() << std::endl;
            }
            // std::cout << "pDesLeg in trot: id" << i <<  pDesLeg.transpose() << std::endl;
        } else {
            // std::cout << "Foot in stance: " << i << std::endl;
            // foot in stance
            firstSwing_[i] = true;
            Vec3<double> pDesFoot_w = footSwingTrajectories_[i].getPosition();
            Vec3<double> vDesFoot_w = footSwingTrajectories_[i].getVelocity();
            // get pFoot reference w.r.t. abad frame
            // std::cout << "pDes: " << pDesFoot_w.transpose() << std::endl;
            const Vec3<double> pDesLeg = data.estimators_->shared_esti_data_.result_->r_b_ * (pDesFoot_w - p_w) -
                                         data.quadruped_model_->getHipLocation(i);
            const Vec3<double> vDesLeg = data.estimators_->shared_esti_data_.result_->r_b_ * (vDesFoot_w - v_w);

            if (!use_wbc_) {
                // desired_.pFoot_des_[i] = pDesFoot_w;
                // desired_.vFoot_des_[i] = vDesFoot_w;
                data.leg_controller_->leg_command[i].p_des = pDesLeg;
                data.leg_controller_->leg_command[i].v_des = vDesLeg;
                data.leg_controller_->leg_command[i].kp_cartisian = kp_cartesian;
                data.leg_controller_->leg_command[i].kd_cartisian = kd_cartesian;
                data.leg_controller_->leg_command[i].kp_joint.setZero();
                data.leg_controller_->leg_command[i].kd_joint = Mat3<double>::Identity() * 0.2;
            } else {
                data.leg_controller_->leg_command[i].p_des = pDesLeg;
                data.leg_controller_->leg_command[i].v_des = vDesLeg;
                data.leg_controller_->leg_command[i].kp_cartisian.setZero();
                data.leg_controller_->leg_command[i].kd_cartisian = kd_cartesian;
            }
            // std::cout << "pDesLeg in stance: id" << i << pDesLeg.transpose() << std::endl;
        }
    }
    // std::cout << std::endl;
    // update contact for estimator
    data.estimators_->setContactPhase(contact_state_);

    // prepare data
    constexpr double max_pos_error = 0.1;
    double xStart = world_position_desired_[0];
    double yStart = world_position_desired_[1];

    if (xStart - p_w[0] > max_pos_error) xStart = p_w[0] + max_pos_error;
    if (p_w[0] - xStart > max_pos_error) xStart = p_w[0] - max_pos_error;

    if (yStart - p_w[1] > max_pos_error) yStart = p_w[1] + max_pos_error;
    if (p_w[1] - yStart > max_pos_error) yStart = p_w[1] - max_pos_error;

    world_position_desired_[0] = xStart;
    world_position_desired_[1] = yStart; {
        std::lock_guard lk(data.fsm_data_mutex_);
        if (currentGait_ != STAND) {
            // update WBC desire
            desired_.pBody_des_[0] = world_position_desired_[0];
            desired_.pBody_des_[1] = world_position_desired_[1];
            desired_.pBody_des_[2] = Config::mpc_height;
            desired_.vBody_des_[0] = v_w_des[0];
            desired_.vBody_des_[1] = v_w_des[1];
            desired_.vBody_des_[2] = 0;
            desired_.aBody_des_.setZero();
            //why wbc desired rpy = 0?
            desired_.pBody_RPY_des_[0] = rpy_des_w_(0);
            desired_.pBody_RPY_des_[1] = rpy_des_w_(1);
            desired_.pBody_RPY_des_[2] = rpy_des_w_(2);
            desired_.vBody_Ori_des_[0] = rpy_vel_des_b_(0);
            desired_.vBody_Ori_des_[1] = rpy_vel_des_b_(1);
            desired_.vBody_Ori_des_[2] = rpy_vel_des_b_(2);
            desired_.vBody_Ori_des_w_ = rpy_vel_des_w_;
            desired_.current_gait_ = currentGait_;
        } else {
            // update WBC desire
            desired_.pBody_des_[0] = pw_ini_(0);
            desired_.pBody_des_[1] = pw_ini_(1);
            desired_.pBody_des_[2] = Config::mpc_height;
            desired_.vBody_des_[0] = 0;
            desired_.vBody_des_[1] = 0;
            desired_.vBody_des_[2] = 0;
            //why wbc desired rpy = 0?
            desired_.pBody_RPY_des_[0] = 0;
            desired_.pBody_RPY_des_[1] = 0.;
            desired_.pBody_RPY_des_[2] = rpy_ini_(2);
            desired_.vBody_Ori_des_[0] = 0;
            desired_.vBody_Ori_des_[1] = 0;
            desired_.vBody_Ori_des_[2] = 0;
            desired_.current_gait_ = currentGait_;
        }
        desired_.contact_table_ = work_gait_->getMPCTable();
    }
    iterCounter_++;
    set_lcm();
    update_.store(true);
}

void Linear_Planner::SetupCommand(const Control_FSM_Data &data) {
    rpy_vel_des_b_.setZero();
    rpy_vel_des_b_(2) = -data.rc_->rc_control_.omega_des[2];
    const double x_vel_cmd = data.rc_->rc_control_.v_des[0];
    const double y_vel_cmd = data.rc_->rc_control_.v_des[1];

    // TODO add gait number
    // set cmd to controller
    gaitNumber_ = static_cast<gait_number>(data.rc_->rc_control_.variables[0]);
    x_vel_des_ = x_vel_des_ * (1. - Config::loco_vel_filter) + x_vel_cmd * Config::loco_vel_filter;
    y_vel_des_ = y_vel_des_ * (1. - Config::loco_vel_filter) + y_vel_cmd * Config::loco_vel_filter;
}

void Linear_Planner::set_lcm() {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            lcm_data_.foot_pDes[j][i] = static_cast<float>(desired_.pFoot_des_[j](i));
            lcm_data_.foot_pw_[j][i] = static_cast<float>(desired_.pFoot_[j][i]);
            lcm_data_.pf_init_[j][i] = static_cast<float>(pf_ini_[j][i]);
            lcm_data_.foot_vdes[j][i] = static_cast<float>(desired_.vFoot_des_[j][i]);
        }
        lcm_data_.p_des[i] = static_cast<float>(desired_.pBody_des_(i));
        lcm_data_.pw_[i] = static_cast<float>(world_pos_(i));
        lcm_data_.rpy_des_[i] = static_cast<float>(desired_.pBody_RPY_des_(i));
    }
    planner_lcm_.publish("Planner_Channel", &lcm_data_);
}
