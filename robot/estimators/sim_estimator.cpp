//
// Created by lingwei on 5/24/24.
//
#include "sim_estimator.h"

namespace Estimators {

    template<typename T>
    Sim_Estimator<T>::Sim_Estimator() = default;

    template
    class Sim_Estimator<double>;
}


