#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 1,1,1,1;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    cout << "Error: estimations size error" << endl;
    return rmse;
  }
  
  //accumulate squared residuals
  VectorXd residual_sum(4);
  residual_sum << 0,0,0,0;
  for(int i=0; i < estimations.size(); ++i) {
	  VectorXd diff = ground_truth[i] - estimations[i];
	  residual_sum += diff.cwiseProduct(diff);
  }

  //calculate the mean
  VectorXd mean(4);
  mean = 1.0 / estimations.size() * residual_sum;

  //calculate the squared root
  rmse = mean.cwiseSqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float px_2 = px*px;
  float py_2 = py*py;
  float vx_2 = vx*vx;
  float vy_2 = vy*vy;

  // check division by zero
  float denom = px_2 + py_2;
  if (denom == 0)
      cout << "Error: Divide by zero" << endl;

  Hj << px/sqrt(denom), py/sqrt(denom), 0, 0,
        -py/denom, px/denom, 0, 0,
        py*(vx*py - vy*px) / pow(denom, 1.5), px*(vy*px - vx*py) / pow(denom, 1.5), px/sqrt(denom), py/sqrt(denom);

  return Hj;
}
