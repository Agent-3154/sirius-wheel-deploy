//
// Created by lingwei on 5/24/24.
//

#ifndef MY_MUJOCO_SIMULATOR_SIM_ESTIMATOR_H
#define MY_MUJOCO_SIMULATOR_SIM_ESTIMATOR_H

#include "Estimator_Base.h"

namespace Estimators {
    template<typename T>
    class Sim_Estimator : public Estimator_Base<T> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        explicit Sim_Estimator();

        virtual void run();

        virtual void setup();

    private:
    };

    template<typename T>
    void Sim_Estimator<T>::run() {

    }

    template<typename T>
    void Sim_Estimator<T>::setup() {

    }
}

#endif //MY_MUJOCO_SIMULATOR_SIM_ESTIMATOR_H
