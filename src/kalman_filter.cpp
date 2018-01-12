#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  std::cout<<"before predict x:"<<x_(0)<<";"<<x_(1)<<";"<<x_(2)<<";"<<x_(3)<<"\n";
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  std::cout<<"after predict x:"<<x_(0)<<";"<<x_(1)<<";"<<x_(2)<<";"<<x_(3)<<"\n";
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double theta = atan2(x_(1), x_(0));

  double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
  VectorXd z_pred = VectorXd(3); // h(x_)
  z_pred << rho, theta, rho_dot;

  VectorXd y = z - z_pred;
  std::cout<<"z:"<<z(0)<<";"<<z(1)<<";"<<z(2)<<"\n";
  std::cout<<"z_pred:"<<z_pred(0)<<";"<<z_pred(1)<<";"<<z_pred(2)<<"\n";
  if(y(1)>M_PI)
	  y(1)=y(1)-2*M_PI;
  if(y(1)<-M_PI)
	  y(1)=y(1)+2*M_PI;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  std::cout<<"after measure x:"<<x_(0)<<";"<<x_(1)<<";"<<x_(2)<<";"<<x_(3)<<"\n";
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
