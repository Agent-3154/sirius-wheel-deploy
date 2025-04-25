//
// Created by lingwei on 5/14/24.
//

#ifndef MY_MUJOCO_SIMULATOR_INTERPOLATION_H
#define MY_MUJOCO_SIMULATOR_INTERPOLATION_H

namespace Interpolate {
    template<typename v_t, typename p_t>
    v_t lerp(v_t y_s, v_t y_f, p_t x) {
        if (x < 0) x = 0;
        if (x > 1) x = 1;
        return (y_s + (y_f - y_s) * x);
    }

    template<typename y_t, typename x_t>
    y_t cubicBezier(y_t y0, y_t yf, x_t x) {
        if (x < 0)x = 0;
        if (x > 1)x = 1;

        y_t yDiff = yf - y0;
        x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
        return y0 + bezier * yDiff;
    }

    template<typename y_t, typename x_t>
    y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
        if (x < 0)x = 0;
        if (x > 1)x = 1;

        y_t yDiff = yf - y0;
        x_t bezier = x_t(6) * x * (x_t(1) - x);
        return bezier * yDiff;
    }

    template<typename y_t, typename x_t>
    y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
        if (x < 0)x = 0;
        if (x > 1)x = 1;

        y_t yDiff = yf - y0;
        x_t bezier = x_t(6) - x_t(12) * x;
        return bezier * yDiff;
    }
}


#endif //MY_MUJOCO_SIMULATOR_INTERPOLATION_H
