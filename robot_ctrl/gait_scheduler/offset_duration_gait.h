//
// Created by lingwei on 5/30/24.
//

#ifndef MY_MUJOCO_SIMULATOR_OFFSET_DURATION_GAIT_H
#define MY_MUJOCO_SIMULATOR_OFFSET_DURATION_GAIT_H

#include "gait_base.h"

template<typename T>
class OffsetDurationGait : public Gait_Base<T> {
public:
    OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> duration, const std::string &name);

    ~OffsetDurationGait();

    Vec4<T> getContactState() override;

    Vec4<T> getSwingState() override;

    int *getMPCTable() override;

    void setIterations(int iterationBetweenMPC, int currentIter) override;

    T getCurrentStanceTime(T dtMPC, int leg) override;

    T getCurrentSwingTime(T dtMPC, int leg) override;

    int getCurrentGaitPhase() override;

private:
    int *mpc_table_;
    // contact offset
    Eigen::Array4<T> offsetTemplate_;
    Eigen::Array4<T> durationTemplate_;
    Eigen::Array4i offsetInt_;
    Eigen::Array4i durationInt_;
    int iteration_;
    int total_iteration_;
    T phase_;
    int stance_;
    int swing_;
};

#endif //MY_MUJOCO_SIMULATOR_OFFSET_DURATION_GAIT_H
