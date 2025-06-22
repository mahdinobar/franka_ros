// Author: Enrico Corvaglia
// https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_controllers/include/utils/pseudo_inversion.h
// File provided under public domain
// pseudo_inverse() computes the pseudo inverse of matrix M_ using SVD decomposition (can choose
// between damped and not)
// returns the pseudo inverted matrix M_pinv_

#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace franka_example_controllers {

//inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
//  double lambda_ = damped ? 0.01 : 0.0;
//
//  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
//  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
//  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
//  S_.setZero();
//
//  for (int i = 0; i < sing_vals_.size(); i++)
//    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);
//
//  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
//}
inline void pseudoInverseAnalytical(const Eigen::MatrixXd& A, Eigen::MatrixXd& A_pinv, double lambda) {
  int m = A.rows();
  int n = A.cols();

  if (m > n) {
    // Left pseudoinverse: (A^T A + λ² I)^-1 A^T
    Eigen::MatrixXd AtA = A.transpose() * A;
    Eigen::MatrixXd damping = lambda * lambda * Eigen::MatrixXd::Identity(n, n);
    A_pinv = (AtA + damping).ldlt().solve(A.transpose());
  } else {
    // Right pseudoinverse: A^T (A A^T + λ² I)^-1
    Eigen::MatrixXd AAt = A * A.transpose();
    Eigen::MatrixXd damping = lambda * lambda * Eigen::MatrixXd::Identity(m, m);
    A_pinv = A.transpose() * (AAt + damping).ldlt().solve(Eigen::MatrixXd::Identity(m, m));
  }
}
}  // namespace franka_example_controllers
