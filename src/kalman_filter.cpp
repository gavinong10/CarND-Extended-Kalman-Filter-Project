#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODOdone:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODOdone:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (MatrixXd::Identity(K.rows(), H_.cols()) - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODOdone:
    * update the state by using Extended Kalman Filter equations
  */
  // Define h_x by using the h function
  VectorXd h_x(3);

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  double px_py_norm = sqrt(px * px + py * py);

  h_x << px_py_norm, atan2(py, px), (px * vx + py * vy) / px_py_norm;

  // Handle case where distance of detected object is zero to avoid divisions by zero
  float eps = 1e-4;

  if (fabsf(px) < eps) {
    h_x(1) = 0;
  }

  VectorXd y = z - h_x;

  // Normalize result
  while (y[1] > M_PI) {
    y[1] -= 2 * M_PI;
  }
  while (y[1] < -M_PI) {
    y[1] += 2 * M_PI;
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (MatrixXd::Identity(K.rows(), H_.cols()) - K * H_) * P_;
}