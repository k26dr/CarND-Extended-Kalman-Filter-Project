#include "kalman_filter.h"
#include "tools.h"
#include <math.h>
#include <iostream>

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
  TODO:
    * predict the state
  */
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd y_ = z - H_*x_; // (2,1)
  MatrixXd S_ = H_*P_*H_.transpose() + R_; // (2,2)
  MatrixXd K_ = P_*H_.transpose()*S_.inverse(); // (4,2)
  MatrixXd I_ = MatrixXd::Identity(4,4);
  
  x_ = x_ + K_*y_;
  P_ = (I_ - K_*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
/**
TODO:
  * update the state by using Extended Kalman Filter equations
*/
  Tools tools;
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float norm = sqrt(px*px + py*py);


  VectorXd hx_ = VectorXd(3);
  hx_ << norm, atan2(py,px), (px*vx + py*vy) / norm;
  H_ = tools.CalculateJacobian(x_);

  MatrixXd y_ = z - hx_;
  while (y_(1) > M_PI)
    y_(1) -= 2*M_PI;
  while (y_(1) < -M_PI)
    y_(1) += 2*M_PI;

  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_*H_.transpose()*S_.inverse();
  MatrixXd I_ = MatrixXd::Identity(4,4);

  x_ = x_ + K_*y_;
  P_ = (I_ - K_*H_) * P_;

}
