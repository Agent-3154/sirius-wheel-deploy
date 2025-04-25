//
// Created by lingwei on 6/3/24.
//

#ifndef LINEAR_QP_SOLVER_H
#define LINEAR_QP_SOLVER_H

#include "Solver_Base.h"
#include "eigen3/Eigen/Dense"
#include "../../third-party/qpOASES/include/qpOASES.hpp"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

class Linear_QP_Solver final : public Solver_Base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Linear_QP_Solver() : Solver_Base() {
    }

    ~Linear_QP_Solver() override = default;

    void setup_problem(double dt, int horizon, double mu, double f_max) override;

    void resize_qp_mats(int horizon);

    void continius_to_qp(const Eigen::Matrix<double, 13, 13>& Ac, Eigen::Matrix<double, 13, 12> Bc, double dt, int horizon);

    static void continue_ss_mats(const Eigen::Matrix<double, 3, 3> &I_world, Eigen::Matrix<double, 3, 4> r_feets,
                                 double m, Eigen::Matrix<double, 3, 3> R_yaw, Eigen::Matrix<double, 13, 13> &A,
                                 Eigen::Matrix<double, 13, 12> &B,
                                 double x_drag);

    double get_solution(int index) const override;

    void solve_problem() override;

private:
    qpOASES::real_t *H_qpoases_{};
    qpOASES::real_t *g_qpoases_{};
    qpOASES::real_t *A_qpoases_{};
    qpOASES::real_t *lb_qpoases_{};
    qpOASES::real_t *ub_qpoases_{};
    qpOASES::real_t *q_soln_{};
    // reduced matrix for speed
    qpOASES::real_t *H_red_{};
    qpOASES::real_t *g_red_{};
    qpOASES::real_t *A_red_{};
    qpOASES::real_t *lb_red_{};
    qpOASES::real_t *ub_red_{};
    qpOASES::real_t *q_red_{};

    Eigen::Matrix<double, Eigen::Dynamic, 13> A_qp_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B_qp_;
    Eigen::Matrix<double, 13, 12> Bdt_;
    Eigen::Matrix<double, 13, 13> Adt_;
    Eigen::Matrix<double, 25, 25> ABc_, expmm_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> S_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_d_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> U_b_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> fmat_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> qH_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> qg_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eye_12h_;

    char var_elim[2000]{}; // what are these two?
    char con_elim[2000]{};

    bool re_allocated_{};
};

inline Eigen::Matrix<double, 3, 3> cross_mat(const Eigen::Matrix<double, 3, 3> &I_inv, Eigen::Matrix<double, 3, 1> r) {
    Eigen::Matrix<double, 3, 3> cm;
    cm << 0.f, -r(2), r(1),
            r(2), 0.f, -r(0),
            -r(1), r(0), 0.f;
    return I_inv * cm;
}

/**
 * @brief convert matrix to qpOASES format
 * @param dst
 * @param src
 * @param rows
 * @param cols
 */
inline void matrix_to_real(qpOASES::real_t *dst, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> src,
                           const int rows, const int cols) {
    int a = 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            dst[a] = src(r, c);
            a++;
        }
    }
}

inline int near_zero(double a) {
    return (a < 0.01 && a > -.01);
}

inline int near_one(double a) {
    return near_zero(a - 1);
}
#endif //LINEAR_QP_SOLVER_H
