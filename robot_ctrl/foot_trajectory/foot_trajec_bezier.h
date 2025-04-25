//
// Created by lingwei on 5/30/24.
//

#ifndef FOOT_TRAJEC_BEZIER_H
#define FOOT_TRAJEC_BEZIER_H

#include "foot_trajec_base.h"
#include "../../utilities/inc/Interpolation.h"

#define fb Foot_Trajec_Base<T>

template<typename T>
class Foot_Trajec_Bezier final : public Foot_Trajec_Base<T> {
public:
    Foot_Trajec_Bezier() : Foot_Trajec_Base<T>() {
    }

    ~Foot_Trajec_Bezier() override = default;

    void computeTrajectory(T phase, T swing_time) override {
        // first interpolate x,y
        fb::p_ = Interpolate::cubicBezier<Vec3<T> >(fb::p0_, fb::pf_, phase);
        fb::v_ = Interpolate::cubicBezierFirstDerivative<Vec3<T> >(fb::p0_, fb::pf_, phase) / swing_time;
        fb::a_ = Interpolate::cubicBezierSecondDerivative<Vec3<T> >(fb::p0_, fb::pf_, phase)
                 / (swing_time * swing_time);

        // then cover z
        T zp, zv, za;
        if (phase < T(0.5)) {
            zp = Interpolate::cubicBezier<T>(fb::p0_[2], fb::p0_[2] + fb::height_, phase * 2);
            zv = Interpolate::cubicBezierFirstDerivative<T>(fb::p0_[2], fb::p0_[2] + fb::height_, phase * 2) * 2 /
                 swing_time;
            za = Interpolate::cubicBezierSecondDerivative<T>(fb::p0_[2], fb::p0_[2] + fb::height_, phase * 2) * 4 / (
                     swing_time * swing_time);
        } else {
            zp = Interpolate::cubicBezier<T>(fb::p0_[2] + fb::height_, fb::pf_[2], phase * 2 - 1);
            zv = Interpolate::cubicBezierFirstDerivative<T>(fb::p0_[2] + fb::height_, fb::pf_[2], phase * 2 - 1) * 2 /
                 swing_time;
            za = Interpolate::cubicBezierSecondDerivative<T>(fb::p0_[2] + fb::height_, fb::pf_[2], phase * 2 - 1) * 4
                 / (swing_time * swing_time);
        }
        fb::p_[2] = zp;
        fb::v_[2] = zv;
        fb::a_[2] = za;
    }
};

#endif //FOOT_TRAJEC_BEZIER_H
