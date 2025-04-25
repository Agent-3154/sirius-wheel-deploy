//
// Created by lingwei on 5/30/24.
//

#ifndef MY_MUJOCO_SIMULATOR_GAIT_BASE_H
#define MY_MUJOCO_SIMULATOR_GAIT_BASE_H

#include "../../utilities/types/hardware_types.h"

enum gait_number {
    STAND = 0,
    TROT,
    PRONKING,
    WALK,
    RUNNING,
};

template<typename T>
class Gait_Base {
public:
    Gait_Base() = default;

    virtual ~Gait_Base() = default;

    virtual Vec4<T> getContactState() = 0;

    virtual Vec4<T> getSwingState() = 0;

    virtual int *getMPCTable() = 0;

    virtual void setIterations(int iterationBetweenMPC, int currentIter) = 0;

    virtual T getCurrentStanceTime(T dtMPC, int leg) = 0;

    /**
     * @note get the actual swing time, swing_ is the segments of swing.
     * @param dtMPC
     * @param leg
     * @return
     */
    virtual T getCurrentSwingTime(T dtMPC, int leg) = 0;

    virtual int getCurrentGaitPhase() = 0;

    T phase_segment_;
    std::string gait_name_;
};

#endif //MY_MUJOCO_SIMULATOR_GAIT_BASE_H
