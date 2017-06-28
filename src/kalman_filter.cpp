#include "kalman_filter.h"
#include "tools.h"

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
  VectorXd y_ = z - H_*x_;
  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_*H_.transpose()*S_.inverse();
  MatrixXd I_ = MatrixXd::Identity(2,2);
  
  x_ = x_ + K_*y_;
  P_ = (I_ - K_*H_) * P_
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

  VectorXd h = VectorXd(3);
  h << norm, arctan(py/px), (px*vx + py*vy) / norm;
  H_ = tools.CalculateJacobian(x_);

  y_ = z - h.array() * x_.array();
  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_*H_.transpose()*S_.inverse();
  MatrixXd I_ = MatrixXd::Identity(3,3);
  
  x_ = x_ + K_*y_;
  P_ = (I_ - K_*H_) * P_
}
