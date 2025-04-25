//
// Created by lingwei on 5/30/24.
//

#ifndef FOOT_TRAJEC_BASE_H
#define FOOT_TRAJEC_BASE_H

#include "../../utilities/types/hardware_types.h"

template<typename T>
class Foot_Trajec_Base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Foot_Trajec_Base() {
        p0_.setZero();
        pf_.setZero();
        p_.setZero();
        v_.setZero();
        a_.setZero();
        height_ = 0;
    }

    virtual ~Foot_Trajec_Base() = default;

    void setInitialPosition(Vec3<T> p0) {
        p0_ = p0;
    }

    void setFinalPosition(Vec3<T> pf) {
        pf_ = pf;
    }

    void setPosition(Vec3<T> pf) {
        p_ = pf;
    }

    void setHeight(T h) {
        height_ = h;
    }

    Vec3<T> getPosition() const {
        return p_;
    }

    Vec3<T> getVelocity() const {
        return v_;
    }

    Vec3<T> getAcceleration() const {
        return a_;
    }

    virtual void computeTrajectory(T phase, T swing_time) = 0;

protected:
    Vec3<T> p0_, pf_, p_, v_, a_;
    T height_;
};

#endif //FOOT_TRAJEC_BASE_H
