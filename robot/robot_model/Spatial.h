//
// Created by lingwei on 5/18/24.
//

#ifndef MY_MUJOCO_SIMULATOR_SPATIAL_H
#define MY_MUJOCO_SIMULATOR_SPATIAL_H

#include <eigen3/Eigen/StdVector>
#include "../../utilities/types/hardware_types.h"
#include "../../config/Config.h"
#include "../../utilities/inc/utilities_fun.h"

/**
 * @brief Create Spatial form, linear : rot.
 * @tparam T
 * @tparam T2
 * @param R
 * @param r
 * @return
 */
template<typename T, typename T2>
auto createSpatialform(const Eigen::MatrixBase<T> &R, const Eigen::MatrixBase<T2> &r) {
    Mat6<typename T::Scalar> X = Mat6<typename T::Scalar>::Zero();
    X.template topLeftCorner<3, 3>() = R;
    X.template bottomRightCorner<3, 3>() = R;
    X.template topRightCorner<3, 3>() = -R * ori::vectorToSkewMat(r);
    return X;
}


template<typename T>
Vec3<typename T::Scalar> matToSkewVec(const Eigen::MatrixBase<T> &m) {
    static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                  "Must have 3x3 matrix");
    return 0.5 * Vec3<typename T::Scalar>(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0),
                                          (m(1, 0) - m(0, 1)));
}

/*!
 * Get rotation matrix from spatial transformation
 */
template<typename T>
auto rotationFromSXform(const Eigen::MatrixBase<T> &X) {
    static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                  "Must have 6x6 matrix");
    RotMat<typename T::Scalar> R = X.template topLeftCorner<3, 3>();
    return R;
}

/**
 * @brief get translate vector from spatial
 * @tparam T
 * @param X
 * @return
 */
template<typename T>
auto translationFromSXform(const Eigen::MatrixBase<T> &X) {
    static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                  "Must have 6x6 matrix");
    RotMat<typename T::Scalar> R = rotationFromSXform(X);
    Vec3<typename T::Scalar> r =
            -matToSkewVec(R.transpose() * X.template topRightCorner<3, 3>());
    return r;
}

/**
 * @brief create spatial form of joint
 * @tparam T
 * @param joint_axis
 * @return
 */
template<typename T>
SVec<T> JointMotionSubspace(Config::Joint_Axis joint_axis) {
    Vec3<T> v(0, 0, 0);
    SVec<T> phi = SVec<T>::Zero();
    if (joint_axis == Config::Joint_Axis_X) {
        v(0) = 1;
    } else if (joint_axis == Config::Joint_Axis_Y) {
        v(1) = 1;
    }
    // only revolute joint
    phi.template bottomLeftCorner<3, 1>() = v;
    return phi;
}

template<typename T>
Mat3<T> coordinateRotation(Config::Joint_Axis axis, T theta) {
    T s = std::sin(theta);
    T c = std::cos(theta);
    Mat3<T> R;
    if (axis == Config::Joint_Axis_X) {
        R << 1, 0, 0, 0, c, s, 0, -s, c;
    } else if (axis == Config::Joint_Axis_Y) {
        R << c, 0, -s, 0, 1, 0, s, 0, c;
    } else if (axis == Config::Joint_Axis_Z) {
        R << c, s, 0, -s, c, 0, 0, 0, 1;
    }
    return R;
}

template<typename T>
Mat6<T> spatialRotation(Config::Joint_Axis axis, T theta) {
    RotMat<T> R = coordinateRotation(axis, theta);
    SXform<T> X = SXform<double>::Zero();
    X.template topLeftCorner<3, 3>() = R;
    X.template bottomRightCorner<3, 3>() = R;
    return X;
}

template<typename T>
Mat6<T> joint_to_Spatialform(Config::Joint_Axis axis, T joint_q) {
    Mat6<double> X = Mat6<double>::Zero();
    X = spatialRotation(axis, joint_q);
    return X;
}


template<typename T>
auto motionCrossProduct(const Eigen::MatrixBase<T> &a,
                        const Eigen::MatrixBase<T> &b) {
    SVec<typename T::Scalar> mv;
    mv << a(1) * b(2) - a(2) * b(1), a(2) * b(0) - a(0) * b(2),
            a(0) * b(1) - a(1) * b(0),
            a(1) * b(5) - a(2) * b(4) + a(4) * b(2) - a(5) * b(1),
            a(2) * b(3) - a(0) * b(5) - a(3) * b(2) + a(5) * b(0),
            a(0) * b(4) - a(1) * b(3) + a(3) * b(1) - a(4) * b(0);
    return mv;
}

template<typename T>
auto mjMatToEigenMat(T mj_mat[9]) {
    Mat3<T> ret;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ret(j, i) = mj_mat[3 * i + j];
        }
    }
    return ret;
}


template<typename T>
auto invertSXform(const Eigen::MatrixBase<T> &X) {
    static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                  "Must have 6x6 matrix");
    RotMat<typename T::Scalar> R = rotationFromSXform(X);
    Vec3<typename T::Scalar> r =
            -matToSkewVec(R.transpose() * X.template topRightCorner<3, 3>());
    SXform<typename T::Scalar> Xinv = createSpatialform(R.transpose(), -R * r);
    return Xinv;
}

template<typename T, typename T2>
auto sXFormPoint(const Eigen::MatrixBase<T> &X,
                 const Eigen::MatrixBase<T2> &p) {
    Mat3<typename T::Scalar> R = rotationFromSXform(X);
    Vec3<typename T::Scalar> r = translationFromSXform(X);
    Vec3<typename T::Scalar> Xp = R * (p - r);
    return Xp;
}

// converted
template<typename T, typename T2>
auto spatialToLinearVelocity(const Eigen::MatrixBase<T> &v,
                             const Eigen::MatrixBase<T2> &x) {
    Vec3<typename T::Scalar> vsAng = v.template bottomLeftCorner<3, 1>();
    Vec3<typename T::Scalar> vsLin = v.template topLeftCorner<3, 1>();
    Vec3<typename T::Scalar> vLinear = vsLin + vsAng.cross(x);
    return vLinear;
}

// converted
template<typename T, typename T2>
auto spatialToLinearAcceleration(const Eigen::MatrixBase<T> &a,
                                 const Eigen::MatrixBase<T2> &v) {
    Vec3<typename T::Scalar> acc;
    // classical accleration = spatial linear acc + omega x v
    acc = a.template head<3>() + v.template tail<3>().cross(v.template head<3>());
    return acc;
}

#endif //MY_MUJOCO_SIMULATOR_SPATIAL_H
