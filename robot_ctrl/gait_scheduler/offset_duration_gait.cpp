//
// Created by lingwei on 5/30/24.
//
#include "offset_duration_gait.h"
#include <iostream>

template<typename T>
int OffsetDurationGait<T>::getCurrentGaitPhase() {
    return iteration_;
}

template<typename T>
T OffsetDurationGait<T>::getCurrentSwingTime(T dtMPC, int leg) {
    (void) leg;
    return dtMPC * swing_;
}

template<typename T>
T OffsetDurationGait<T>::getCurrentStanceTime(T dtMPC, int leg) {
    (void) leg;
    return dtMPC * stance_;
}

/**
 * @TODO currentIter accumulate in stateFSM, maybe overrun?
 * @tparam T
 * @param iterationBetweenMPC: loops of wbc in one mpc loop, now value: 20
 * @param currentIter: time-run iter, its frequency is the same as robot runner
 */
template<typename T>
void OffsetDurationGait<T>::setIterations(int iterationBetweenMPC, int currentIter) {
    // get the real mpc iter, decoupling it with mpc in future
    iteration_ = (currentIter / iterationBetweenMPC) % total_iteration_;
    //iterationBetweenMPC * total_iteration_: total iterations per gait cycle
    phase_ =
            static_cast<T>(currentIter % (iterationBetweenMPC * total_iteration_)) / static_cast<T>(
                iterationBetweenMPC * total_iteration_);
    // for now: iterationbetweenmpc: 20, total_iteration:10. phase increase: 0.005
    Gait_Base<T>::phase_segment_ = 1. / static_cast<T>(iterationBetweenMPC * total_iteration_);
}

/**
 *
 * @tparam T
 * @return the contact table in one horizon.
 */
template<typename T>
int *OffsetDurationGait<T>::getMPCTable() {
    // std::cout << "mpc_table:\n";
    // total iteration: total segment of each gait
    for (int i = 0; i < total_iteration_; i++) {
        int iter = (i + iteration_ + 1) % total_iteration_;
        Eigen::Array4i progress = iter - offsetInt_;
        for (int j = 0; j < 4; j++) {
            if (progress[j] < 0) progress[j] += total_iteration_;
            if (progress[j] < durationInt_[j]) {
                mpc_table_[4 * i + j] = 1; //contact
            } else {
                mpc_table_[4 * i + j] = 0;
            }
            // std::cout << mpc_table_[4*i+j] << " | ";
        }
        // std::cout << "||";
    }
    // std::cout << "\n";
    return mpc_table_;
}

template<typename T>
Vec4<T> OffsetDurationGait<T>::getSwingState() {
    // get offset for swing first
    Eigen::Array4<T> swing_offset = offsetTemplate_ + durationTemplate_;
    for (int i = 0; i < 4; i++) {
        if (swing_offset[i] > 1) {
            swing_offset[i] -= 1.;
        }
    }
    // get swing duration
    Eigen::Array4<T> swing_duration = 1. - durationTemplate_;
    Eigen::Array4<T> progress = phase_ - swing_offset;

    for (int i = 0; i < 4; i++) {
        if (progress[i] < 0) progress[i] += 1.f;
        if (progress[i] > swing_duration[i]) {
            progress[i] = 0.;
        } else {
            progress[i] = progress[i] / swing_duration[i];
        }
    }
    return progress.matrix();
}

/**
 * @brief get progress of contact.
 * @tparam T
 * @return
 */
template<typename T>
Vec4<T> OffsetDurationGait<T>::getContactState() {
    Eigen::Array4<T> progress = phase_ - offsetTemplate_;
    for (int i = 0; i < 4; i++) {
        if (progress[i] < 0) progress[i] += 1.;
        if (progress[i] > durationTemplate_[i]) {
            // swing
            progress[i] = 0.;
        } else {
            progress[i] = progress[i] / durationTemplate_[i];
        }
    }
    return progress.matrix();
}

template<typename T>
OffsetDurationGait<T>::~OffsetDurationGait() {
    delete[] mpc_table_;
}

/**
 * @brief
 * @tparam T
 * @param nSegment
 * @param offset
 * @param duration
 * @param name
 */
template<typename T>
OffsetDurationGait<T>::OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> duration,
                                          const std::string &name): offsetInt_(offset.array()),
                                                                    durationInt_(duration.array()),
                                                                    total_iteration_(nSegment) {
    Gait_Base<T>::gait_name_ = name;
    mpc_table_ = new int[nSegment * 4];
    // normalize the offset from int to point
    offsetTemplate_ = offset.cast<T>() / static_cast<T>(nSegment);
    durationTemplate_ = duration.cast<T>() / static_cast<T>(nSegment);
    stance_ = duration[0];
    swing_ = nSegment - duration[0];
    iteration_ = phase_ = 0;
}

template
class OffsetDurationGait<double>;
