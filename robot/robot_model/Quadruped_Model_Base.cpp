//
// Created by lingwei on 4/23/24.
//

#include "Quadruped_Model_Base.h"
#include "../../utilities/types/std_cout_colors.h"
#include <cmath>
#include <iostream>
#include "../../utilities/inc/utilities_fun.h"
#include "Spatial.h"

/**
 * @brief extract model info from mujoco model.
 * @param m
 */
Quadruped_Base::Quadruped_Base(const mjModel *model) : lcm_(getLcmUrl(255)) {
    // ATTENTION: urdf axiis should the z direction
    /* first body is the world body, the second is the base, the FR FL RR FL
     * take FR info for the model information*/
    alg_m_ = mj_copyModel(nullptr, model);
    alg_d_ = mj_makeData(alg_m_);
    if (alg_d_ != nullptr) {
        mj_forward(alg_m_, alg_d_);
        std::cout << GREEN << "[ALG MODEL]: " << RESET << "Load Success!\n";
    }
    q_pos_ = new mjtNum[alg_m_->nq];
    q_vel_ = new mjtNum[alg_m_->nv];

    M_.setZero();
    Cqqd_.setZero();

    nlegs_ = Config::num_legs;
    base_id = mj_name2id(alg_m_, mjOBJ_BODY, "base");
    fr_abad_id = mj_name2id(alg_m_, mjOBJ_BODY, "fr_abad");
    int fr_hip_id = mj_name2id(alg_m_, mjOBJ_BODY, "fr_hip");
    int fr_knee_id = mj_name2id(alg_m_, mjOBJ_BODY, "fr_knee");
    int fr_foot_site_id = mj_name2id(alg_m_, mjOBJ_SITE, "fr_foot_site");

    fr_foot_id = mj_name2id(alg_m_, mjOBJ_BODY, "fr_knee");
    fl_foot_id = mj_name2id(alg_m_, mjOBJ_BODY, "fl_knee");
    rr_foot_id = mj_name2id(alg_m_, mjOBJ_BODY, "rr_knee");
    rl_foot_id = mj_name2id(alg_m_, mjOBJ_BODY, "rl_knee");
    feet_ids << fr_foot_id, fl_foot_id, rr_foot_id, rl_foot_id;
    pGC_.resize(Config::num_legs);
    vGC_.resize(Config::num_legs);

    // not the actual body length, bodyLength + half_motor
    bodyLength_ =
            2.0 * (std::fabs(alg_m_->body_pos[fr_abad_id * 3]) + std::fabs(alg_m_->body_pos[fr_hip_id * 3]));
    bodyWidth_ = 2.0 * (std::fabs(alg_m_->body_pos[fr_abad_id * 3 + 1]));

    // the length of the link must be situated.
#if defined CHAOJI_GO
    abadLinkLength_ = std::fabs(alg_m_->body_pos[fr_knee_id * 3 + 1] - alg_m_->body_pos[fr_abad_id * 3 + 1]);
#else
    abadLinkLength_ = std::fabs(alg_m_->body_pos[fr_hip_id * 3 + 1]);
#endif// y pos

    hipLinkLength_ = std::fabs(alg_m_->body_pos[fr_knee_id * 3 + 2]); //z pos
    //TODO here is the value bug of the kneeLinkLength_
    kneeLinkLength_ =
            std::fabs(alg_m_->site_pos[fr_foot_site_id * 3 + 2]) + alg_m_->site_size[fr_foot_site_id * 3] / 2.0;
    abadLocation_ = Vec3<double>(bodyLength_, bodyWidth_, 0) * 0.5f;
    hipLocation_ = Vec3<double>(0, abadLinkLength_, 0);
    kneeLocation_ = Vec3<double>(0, 0, -hipLinkLength_);

    // clip world body
    for (int i = 1; i < Config::nbody_clip; i++) {
        robotTotalMass_ += alg_m_->body_mass[i];
        if (i == 1 || ((i + 1) % 3 == 0))
            robotBodyMass_ += alg_m_->body_mass[i];
    }
    std::cout << GREEN << "[Build Alg Model]: " << RESET << "Total Mass: " << robotTotalMass_ << " | BodyLength: "
            << bodyLength_ << " | bodyWidth: " << bodyWidth_ << " | AbadLinkLength: " <<
            abadLinkLength_ << " | HipLinkLength: " << hipLinkLength_ << " | KneeLinkLength: " << kneeLinkLength_
            << "\n";

    // prepare variable for contact Jacobian,
    const SVec<double> zeros = SVec<double>::Zero();
    for (int i = 0; i < (alg_m_->nq - 1); i++) {
        S_.push_back(zeros);
    }
    for (int i = 6; i < (alg_m_->nq - 1); i++) {
        // clip body joints
        S_[i] = JointMotionSubspace<double>(static_cast<Config::Joint_Axis>(Config::Joint_Axises[i - 6]));
        //        std::cout << "S id: " << i << "\n" << S_[i].transpose() << "\n";
    }
    //X_tree: Default X when reading the model, use the mujoco model to generate x_tree
    Vec3<double> sub_trans;
    sub_trans.setZero();
    Mat3<double> sub_rot;
    sub_rot.setOnes();
    // push two identity to present world the body tree.
    Xtree_.emplace_back(Mat6<double>::Identity());
    Xtree_.emplace_back(Mat6<double>::Identity());
    //    std::cout << "Setup Xtree:\n";
    // xtree_check ok;
    for (int i = fr_abad_id; i < Config::nbody_clip; i++) {
        sub_trans(0) = alg_m_->body_pos[3 * i];
        sub_trans(1) = alg_m_->body_pos[3 * i + 1];
        sub_trans(2) = alg_m_->body_pos[3 * i + 2];
        Quat<double> body_quat;
        body_quat << alg_m_->body_quat[4 * i],
                alg_m_->body_quat[4 * i + 1],
                alg_m_->body_quat[4 * i + 2],
                alg_m_->body_quat[4 * i + 3];
        sub_rot = ori::quaternionToRotationMatrix(body_quat);
        Xtree_.push_back(createSpatialform(sub_rot, sub_trans));
    }

    // initiate X_up, first joint is the 0 joint.
    Xup_.resize(14);
    Xup_[0] = Mat6<double>::Identity();
    Xa_.resize(14);
    Xa_[0] = Mat6<double>::Identity();
    Xai_.resize(4);
    c_.resize(14);
    c_[0] = SVec<double>::Zero();
    c_[1] = SVec<double>::Zero();
    v_glo_.resize(14);
    avp_.resize(14);
    avp_[0] = SVec<double>::Zero();
    Jcdqd_.resize(4); // only compute the foot
    Jc_.resize(4);
    Jc_[0].setZero();
    Jc_[1].setZero();
    Jc_[2].setZero();
    Jc_[3].setZero();
}

/**
 * @note the axis of the data is the same with mujoco
 */
void Quadruped_Base::run_dynamics() const {
    mju_copy(alg_d_->qpos, q_pos_, alg_m_->nq);
    mju_copy(alg_d_->qvel, q_vel_, alg_m_->nv); //pos forward

    mj_kinematics(alg_m_, alg_d_);
    mj_comPos(alg_m_, alg_d_);
    mj_crb(alg_m_, alg_d_); // timed internally (POS_INERTIA)
    mj_factorM(alg_m_, alg_d_);
    // vel forward, C matrix
    mj_comVel(alg_m_, alg_d_);
    mj_passive(alg_m_, alg_d_);
    mj_referenceConstraint(alg_m_, alg_d_);
    // compute qfrc_bias with abbreviated RNE (without acceleration)
    mj_rne(alg_m_, alg_d_, 0, alg_d_->qfrc_bias);
    //    alg_d_->qacc[2] = 0; // the qacc is besides the g
    // mj_rnePostConstraint(alg_m_, alg_d_);
}

/**
 * @note The C vector include gravity, first position, then orientation. Different from feather stone's book
 */
void Quadruped_Base::update_M_C_matrix() {
    int nv = alg_m_->nv;
    auto *temp = new mjtNum[nv * nv];
    mj_fullM(alg_m_, temp, alg_d_->qM);

    for (int i = 0; i < nv; i++) {
        for (int j = 0; j < nv; j++) {
            M_(j, i) = temp[i * nv + j]; // mujoco column first
        }
        Cqqd_(i) = alg_d_->qfrc_bias[i];
    }

    // std::cout << "M matrix: \n" << M_ << std::endl;
    delete[] temp;
}

void Quadruped_Base::print_M_C_Matrix() const {
    //    std::cout << "M Matrix: \n" << M_ << "\n C Matrix: \n: " << C_;
    mj_printData(alg_m_, alg_d_, "Running_Model.txt");
}

void Quadruped_Base::get_joint_configuration(const Vec19<double> &q_joint, const Vec18<double> &qd_joint,
                                             const Vec3<double> &acc) const {
    for (int i = 0; i < 6; i++) {
        q_pos_[i] = q_joint(i);
        q_vel_[i] = qd_joint(i);
    }
    q_pos_[6] = q_joint(6);
    // TODO Comment this
    //    q_pos_[0] = 0;
    //    q_pos_[1] = 0;
    // take sign:
    for (int i = 0; i < 4; i++) {
        q_pos_[Config::abad_pos_addr_offset + 3 * i] =
                q_joint(Config::abad_pos_addr_offset + 3 * i);
        q_pos_[Config::hip_pos_addr_offset + 3 * i] =
                q_joint(Config::hip_pos_addr_offset + 3 * i);
        q_pos_[Config::knee_pos_addr_offset + 3 * i] =
                q_joint(Config::knee_pos_addr_offset + 3 * i);
        q_vel_[Config::abad_vel_addr_offset + 3 * i] =
                qd_joint(Config::abad_vel_addr_offset + 3 * i);
        q_vel_[Config::hip_vel_addr_offset + 3 * i] =
                qd_joint(Config::hip_vel_addr_offset + 3 * i);
        q_vel_[Config::knee_vel_addr_offset + 3 * i] =
                qd_joint(Config::knee_vel_addr_offset + 3 * i);
    }
    alg_d_->qacc[0] = acc[0];
    alg_d_->qacc[1] = acc[1];
    // alg_d_->qacc[2] = acc[2] - Config::G;
    alg_d_->qacc[2] = -Config::G;
    // std::cout << acc[2] << std::endl;
}

void Quadruped_Base::get_objectSpatialVelocity(int obj_id, mjtNum *res, int flg_local) const {
    int bodyid = 0;
    const mjtNum *pos = 0, *rot = 0;
    bodyid = obj_id;
    pos = alg_d_->xpos + 3 * obj_id;
    rot = (flg_local ? alg_d_->xmat + 9 * obj_id : nullptr);
    mju_transformSpatial(res, alg_d_->cvel + 6 * bodyid, 0, pos, alg_d_->subtree_com + 3 * alg_m_->body_rootid[bodyid],
                         rot);
}

void Quadruped_Base::get_objectSpatialAcceleration(int obj_id, mjtNum *res, int flg_local) const {
    int bodyid = 0;
    const mjtNum *pos = 0, *rot = 0;
    mjtNum correction[3], vel[6];
    bodyid = obj_id;
    pos = alg_d_->xpos + 3 * obj_id;
    rot = (flg_local ? alg_d_->xmat + 9 * obj_id : nullptr);
    mju_transformSpatial(vel, alg_d_->cvel + 6 * bodyid, 0, pos, alg_d_->subtree_com + 3 * alg_m_->body_rootid[bodyid],
                         rot);
    // transform com-based acceleration to local frame
    mju_transformSpatial(res, alg_d_->cacc + 6 * bodyid, 0, pos, alg_d_->subtree_com + 3 * alg_m_->body_rootid[bodyid],
                         rot);
    // acc_tran += vel_rot x vel_tran
    mju_cross(correction, vel, vel + 3);
    mju_addTo3(res + 3, correction);
}

/**
 * @brief update variables not provided by mujoco
 */
void Quadruped_Base::update_variables() {
    Vec3<double> body_pos_w;
    body_pos_w << q_pos_[0], q_pos_[1], q_pos_[2];
    Quat<double> body_quat;
    body_quat << q_pos_[3], q_pos_[4], q_pos_[5], q_pos_[6];
    model_ori_ = body_quat;
    Vec3<double> base_rpy = ori::quatToRPY(body_quat);
    Vec3<double> body_omega;
    Vec3<double> body_vel_w;
    body_omega << q_vel_[3], q_vel_[4], q_vel_[5];
    body_vel_w << q_vel_[0], q_vel_[1], q_vel_[2];
    const Mat3<double> rot = ori::quaternionToRotationMatrix(body_quat);
    Vec3<double> vel_body = rot * body_vel_w;
    // Vec3<double> vel_omega = rot * body_omega_w;

    model_pos_ = body_pos_w;
    model_omega_ = body_omega;
    model_vel_ = body_vel_w;
    for (int i = 0; i < 3; i++) {
        quadruped_data_.base_rpy[i] = static_cast<float>(base_rpy[i]);
        quadruped_data_.base_omega[i] = static_cast<float>(body_omega[i]);
        quadruped_data_.base_pos[i] = static_cast<float>(body_pos_w[i]);
        quadruped_data_.base_vel[i] = static_cast<float>(body_vel_w[i]);
    }

    Xup_[1] = createSpatialform(ori::quaternionToRotationMatrix(body_quat), body_pos_w);
    avp_[1] = SVec<double>::Zero(); // body's bias acc
    // this v is linear:rot
    v_glo_[1] << vel_body, body_omega;
    // Update the kinematic tree
    for (int i = fr_abad_id; i < Config::nbody_clip; i++) {
        int true_ind = i - fr_abad_id;
        Mat6<double> XJ = joint_to_Spatialform(static_cast<Config::Joint_Axis>(Config::Joint_Axises[true_ind]),
                                               q_pos_[Config::abad_pos_addr_offset + true_ind]);
        Xup_[i] = XJ * Xtree_[i];
        SVec<double> V_j =
                S_[Config::subtree_starts_offset + true_ind] * q_vel_[Config::abad_vel_addr_offset + true_ind];
        // ATTENTION : Cvel here is the same as rigid body dynmics
        v_glo_[i] = Xup_[i] * v_glo_[alg_m_->body_parentid[i]] + V_j;
        c_[i] = motionCrossProduct(v_glo_[i], V_j);
        // bias acceleration check here
        avp_[i] = Xup_[i] * avp_[alg_m_->body_parentid[i]] + c_[i];
    }

    // update X_a, the X_a in mujoco are shifting, better not to add constraint.
    for (int i = 1; i < Config::nbody_clip; i++) {
        if (alg_m_->body_parentid[i] == 0) {
            Xa_[i] = Xup_[i];
        } else {
            Xa_[i] = Xup_[i] * Xa_[alg_m_->body_parentid[i]];
        }
    }

    // Update foot contact point and contact Jacobian
    Vec3<double> local_p_GC;
    // foot vector in frame of knee coordinate
    local_p_GC << 0, 0, -kneeLinkLength_;
    //    std::cout << "Start print :\n";
    for (int i = 0; i < feet_ids.size(); i++) {
        Xai_[i] = invertSXform(Xa_[feet_ids[i]]);
        SVec<double> v_spatial = Xai_[i] * v_glo_[feet_ids[i]];
        pGC_.at(i) = sXFormPoint(Xai_[i], local_p_GC); //right
        vGC_.at(i) = spatialToLinearVelocity(v_spatial, pGC_.at(i));
        // update jacobian matrix
        Mat3<double> Rai = Xa_[feet_ids[i]].block<3, 3>(0, 0).transpose();
        Mat6<double> Xc = createSpatialform(Rai, local_p_GC);
        SVec<double> ac = Xc * avp_[feet_ids[i]];
        SVec<double> vc = Xc * v_glo_[feet_ids[i]];
        Jcdqd_[i] = spatialToLinearAcceleration(ac, vc);
        // std::cout << "Jcqd: " << Jcdqd_[i].transpose() << std::endl;
        D3Mat<double> Xout = Xc.topRows<3>();
        // from tips to base
        int body_id = feet_ids[i];
        // starts from tip,
        while (body_id != base_id) {
            Jc_[i].col(Config::subtree_starts_offset + body_id - fr_abad_id) =
                    Xout * S_[Config::subtree_starts_offset + body_id - fr_abad_id];
            D3Mat<double> temp = Xout * Xup_[body_id];
            Xout = temp;
            body_id = alg_m_->body_parentid[body_id];
        }
        Jc_[i].leftCols<6>() = Xout;
        //        if (i == 0) { std::cout << Jc_[i] << std::endl; }
        //        std::cout << "Contact: " << i << " | " << vGC_.at(i).transpose() << std::endl << std::endl;
    }
}

// only contact link matters.
void Quadruped_Base::set_model_lcm() {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            quadruped_data_.pGC[i][j] = static_cast<float>(pGC_[i](j));
            quadruped_data_.vGC[i][j] = static_cast<float>(vGC_[i](j));
        }
    }
    Vec3<double> temp_rpy = ori::quatToRPY(model_ori_);
    for (int i = 0; i < 3; i++) {
        quadruped_data_.base_vel[i] = static_cast<float>(model_vel_[i]);
        quadruped_data_.base_pos[i] = static_cast<float>(model_pos_[i]);
        quadruped_data_.base_omega[i] = static_cast<float>(model_omega_[i]);
        quadruped_data_.base_rpy[i] = static_cast<float>(temp_rpy[i]);
    }
    lcm_.publish("MODEL_DATA", &quadruped_data_);
}

void
Quadruped_Base::update_model(const Vec19<double> &q_joint, const Vec18<double> &qd_joint, const Vec3<double> &acc) {
    this->get_joint_configuration(q_joint, qd_joint, acc);
    this->run_dynamics();
    this->update_M_C_matrix();
    this->update_variables();
    // this->set_model_lcm();
}
