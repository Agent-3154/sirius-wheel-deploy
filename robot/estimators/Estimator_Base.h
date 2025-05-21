//
// Created by lingwei on 4/24/24.
//

#ifndef MY_MUJOCO_SIMULATOR_ESTIMATOR_BASE_H
#define MY_MUJOCO_SIMULATOR_ESTIMATOR_BASE_H

#include "../../utilities/types/hardware_types.h"
#include "../leg_controller/leg_control.h"
#include <eigen3/Eigen/StdVector>
#include "../../lcm-types/cpp/state_estimator_lcmt.hpp"

template<typename T>
struct StateEstimateOutput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec4<T> contactEstimate_;
    Vec3<T> v_b_;
    Quat<T> q_ori_;
    Vec3<T> omega_b_;
    RotMat<T> r_b_;
    Vec3<T> rpy_;

    Vec3<T> p_w_;
    Vec3<T> omega_w_;
    Vec3<T> v_w_;
    Vec3<T> a_b_;
    Vec3<T> a_w_;
    Vec3<T> pFoot_[4];

    void setLcm(state_estimator_lcmt &lcm_data) {
        for (int i = 0; i < 3; i++) {
            lcm_data.p[i] = p_w_[i];
            lcm_data.vWorld[i] = v_w_[i];
            lcm_data.vBody[i] = v_b_[i];
            lcm_data.rpy[i] = rpy_[i];
            lcm_data.omegaBody[i] = omega_b_[i];
            lcm_data.omegaWorld[i] = omega_w_[i];
            lcm_data.aBody[i] = a_b_[i];
            lcm_data.aWorld[i] = a_w_[i];
        }

        for (int i = 0; i < 4; i++) {
            lcm_data.quat[i] = q_ori_[i];
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                lcm_data.pFoot[i][j] = pFoot_[i][j];
            }
        }
    }
};

//TODO Add initialize function for the struct


template<typename T>
struct StateEstimationData {
    StateEstimateOutput<T> *result_;
    USB_Imu_t *usb_imu_data_;
    Leg_Control_Data<T> *legControlData_;
    Vec4<T> *constactPhase_;
};

template<typename T>
class Estimator_Base {
public:
    virtual void run() = 0;

    virtual void setup() = 0;

    virtual ~Estimator_Base() = default;

    StateEstimationData<T> stateEstimateData_;

    void setDataBuff(StateEstimationData<T> data) { stateEstimateData_ = data; }
};

/**
 * @brief this class is used to share estimator datas with all estimators.
 * @tparam T
 */
template<typename T>
class StateEstimatorContainer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StateEstimatorContainer(StateEstimateOutput<T> *stateEstimate, USB_Imu_t *imuData,
                            Leg_Control_Data<T> *LegControlData) {
        phase_ = Vec4<T>::Zero();
        shared_esti_data_.legControlData_ = LegControlData;
        shared_esti_data_.result_ = stateEstimate;
        shared_esti_data_.usb_imu_data_ = imuData;
        shared_esti_data_.constactPhase_ = &phase_;
    }

    template<typename Estimator_Type>
    void addEstimator(const std::string &file_name) {
        Estimator_Base<T> *estimator = new Estimator_Type(file_name);
        estimator->setDataBuff(shared_esti_data_);
        estimators_.push_back(estimator);
    }

    ~StateEstimatorContainer() {
        for (auto estimator: estimators_) {
            delete estimator;
        }
        estimators_.clear();
    }

    void run_estimators() {
        for (auto estimator: estimators_) {
            estimator->run();
        }
    }

    void setContactPhase(Vec4<T> &phase) {
        shared_esti_data_.result_->contactEstimate_ = phase;
    }

    Vec3<T> get_result_world_position() const{
        return shared_esti_data_.result_->p_w_;
    }

    Vec3<T> get_result_world_velocity() const{
        return shared_esti_data_.result_->v_w_;
    }

    Quat<T> get_result_quat() const{
        return shared_esti_data_.result_->q_ori_;
    }

    Vec3<T> get_result_angular_body() const{
        return shared_esti_data_.result_->omega_b_;
    }

    Vec3<T> get_reult_acc_w() const{
        return shared_esti_data_.result_->a_w_;
    }

//private:
    StateEstimationData<T> shared_esti_data_;
    std::vector<Estimator_Base<T> *> estimators_;
    Vec4<T> phase_;
};

#endif //MY_MUJOCO_SIMULATOR_ESTIMATOR_BASE_H
