//
// Created by lingwei on 4/24/24.
//

#ifndef MY_MUJOCO_SIMULATOR_ORIENTATIONESTIMATOR_H
#define MY_MUJOCO_SIMULATOR_ORIENTATIONESTIMATOR_H

#include "Estimator_Base.h"

namespace Estimators {
    template<typename T>
    class UsbImuOrientationEstimator : public Estimator_Base<T> {
    public:
        explicit UsbImuOrientationEstimator(const std::string &file) {(void ) file;};

        ~UsbImuOrientationEstimator() = default;

        virtual void run();

        virtual void setup() {}

    protected:
        bool first_vist_ = true;
        Quat<T> _ori_ini_inv;
    };

}
#endif //MY_MUJOCO_SIMULATOR_ORIENTATIONESTIMATOR_H
