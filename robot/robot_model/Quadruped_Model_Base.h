//
// Created by lingwei on 4/23/24.
//

#ifndef MY_MUJOCO_SIMULATOR_QUADRUPED_MODEL_BASE_H
#define MY_MUJOCO_SIMULATOR_QUADRUPED_MODEL_BASE_H

#include "../../mujoco-3.1.3/include/mujoco/mujoco.h"
#include "../../mujoco-3.1.3/include/mujoco/mjmodel.h"
#include <cassert>
#include "../../utilities/types/hardware_types.h"
#include <eigen3/Eigen/StdVector>
#include "../../config/Config.h"
#include "lcm/lcm-cpp.hpp"
#include "../../lcm-types/cpp/quadruped_model_lcmt.hpp"

class Quadruped_Base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Quadruped_Base(const mjModel *m);

    ~Quadruped_Base() = default;

    /**
     * @note
     * @param leg
     * @return the positon of abad motor w.r.t. com
     */
    auto getHipLocation(int leg) -> Vec3<double> {
        assert(leg >= 0 && leg < 4);
        Vec3<double> pHip((leg == 0 || leg == 1) ? abadLocation_(0) : -abadLocation_(0),
                          (leg == 1 || leg == 3) ? abadLocation_(1) : -abadLocation_(1),
                          abadLocation_(2));
        return pHip;
    }

    [[nodiscard]] auto get_abadLinkLength() const -> double {
        return abadLinkLength_;
    }

    [[nodiscard]] auto get_hipLinkLength() const -> double {
        return hipLinkLength_;
    }

    [[nodiscard]] auto get_kneeLinkLength() const -> double {
        return kneeLinkLength_;
    }

    [[nodiscard]] auto get_nLegs() const -> int {
        return nlegs_;
    }

    [[nodiscard]] Quat<double> get_model_quat_copy() const { return model_ori_; }

    [[nodiscard]] Vec3<double> get_model_omega_copy() const { return model_omega_; }

    [[nodiscard]] Vec3<double> get_model_vel_copy() const { return model_vel_; }

    [[nodiscard]] Vec3<double> get_model_pos_copy() const { return model_pos_; }

    [[nodiscard]] Vec3<double> get_pGC(const int i) const { return pGC_[i]; }

    [[nodiscard]] Vec3<double> get_vGC(const int i) const { return vGC_[i]; }

    [[nodiscard]] D3Mat18<double> get_jc(const int i) const { return Jc_[i]; }

    [[nodiscard]] Vec3<double> get_jcdqd(const int i) const { return Jcdqd_[i]; }

    void get_Mq_Matrix(DMat<double> &Mq) const { Mq = M_; }

    void get_Cqqd_Matrix(DVec<double> &Cqqd) const { Cqqd = Cqqd_; }

    void run_dynamics() const;

    void update_M_C_matrix();

    void get_joint_configuration(const Vec19<double> &q_joint, const Vec18<double> &qd_joint,
                                 const Vec3<double> &acc) const;

    void print_M_C_Matrix() const;

    void get_objectSpatialVelocity(int obj_id, mjtNum *res, int flg_local) const;

    void get_objectSpatialAcceleration(int obj_id, mjtNum *res, int flg_local) const;

    void update_variables();

    void update_model(const Vec19<double> &q_joint, const Vec18<double> &qd_joint, const Vec3<double> &acc);

    void set_model_lcm();

    Vec3<double> abadLocation_, hipLocation_, kneeLocation_;
    double bodyLength_, abadLinkLength_, hipLinkLength_, kneeLinkLength_;
    double robotTotalMass_{}, robotBodyMass_{};
    double bodyWidth_;
    int nlegs_;
    mjModel *alg_m_;
    mjData *alg_d_;
    mjtNum *q_pos_;
    mjtNum *q_vel_;
    Quat<double> model_ori_;
    Vec3<double> model_omega_;
    Vec3<double> model_pos_;
    Vec3<double> model_vel_;
    std::vector<Mat6<double>, Eigen::aligned_allocator<Mat6<double> > > Xtree_;
    // default connections between objects. knee last
    // only update absolute X matrix of contact feet
    std::vector<Mat6<double>, Eigen::aligned_allocator<Mat6<double> > > Xup_, Xa_, Xai_; //increasing joint number
    vectorAligned<SVec<double> > S_;
    vectorAligned<SVec<double> > avp_, c_, v_glo_; // global a is used for inverse dynamic
    Eigen::Matrix<double, 18, 18> M_;
    Vec18<double> Cqqd_; //C(q,qd) this cq includes gravity

    //contact point in world.
    std::vector<Vec3<double> > pGC_;
    std::vector<Vec3<double> > vGC_;
    vectorAligned<D3Mat18<double> > Jc_;
    vectorAligned<Vec3<double> > Jcdqd_;

    //ids
    int fr_abad_id, base_id; // used for marking the start id of legs
    int fr_foot_id, fl_foot_id, rr_foot_id, rl_foot_id;
    Vec4<int> feet_ids;

    lcm::LCM lcm_;
    quadruped_model_lcmt quadruped_data_{};
};

#endif //MY_MUJOCO_SIMULATOR_QUADRUPED_MODEL_BASE_H
