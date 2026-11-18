#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace lbr_impedance_control {

inline void pseudo_inverse(const Eigen::MatrixXd &M, Eigen::MatrixXd *M_pinv,
                           bool damped = true) {
  double lambda = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals = svd.singularValues();
  Eigen::MatrixXd S = M;
  S.setZero();

  for (int i = 0; i < sing_vals.size(); ++i)
    S(i, i) = sing_vals(i) / (sing_vals(i) * sing_vals(i) + lambda * lambda);

  *M_pinv = svd.matrixV() * S.transpose() * svd.matrixU().transpose();
}

} // namespace lbr_impedance_control