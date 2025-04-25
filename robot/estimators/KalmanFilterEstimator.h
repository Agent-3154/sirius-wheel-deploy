//
// Created by lingwei on 4/24/24.
//

#ifndef MY_MUJOCO_SIMULATOR_KALMANFILTERESTIMATOR_H
#define MY_MUJOCO_SIMULATOR_KALMANFILTERESTIMATOR_H

#include "Estimator_Base.h"
#include <eigen3/Eigen/Dense>

namespace Estimators {

    template <typename T>
    struct KalmanFilterParameters {
        T imu_process_noise_position_;
        T imu_process_noise_velocity_;
        T foot_process_noise_position_;
        T foot_sensor_noise_position_;
        T foot_sensor_noise_velocity_;
        T foot_height_sensor_noise_;
        T control_dt_;
    };

    template<typename T>
    class LinearKFPositionVelocityEsitmator : public Estimator_Base<T> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        explicit LinearKFPositionVelocityEsitmator(const std::string& file);

        virtual void run();

        virtual void setup();

    private:
        Eigen::Matrix<T, 18, 1> _xhat;
        Eigen::Matrix<T, 12, 1> _ps;
        Eigen::Matrix<T, 12, 1> _vs;
        Eigen::Matrix<T, 18, 18> _A;
        Eigen::Matrix<T, 18, 18> _Q0;
        Eigen::Matrix<T, 18, 18> _P;
        Eigen::Matrix<T, 28, 28> _R0;
        Eigen::Matrix<T, 18, 3> _B;
        Eigen::Matrix<T, 28, 18> _C;

        KalmanFilterParameters<T> paras_;
        void GetSettings(const std::string& filename, const std::string& setting_name);
    };
}
#endif //MY_MUJOCO_SIMULATOR_KALMANFILTERESTIMATOR_H
