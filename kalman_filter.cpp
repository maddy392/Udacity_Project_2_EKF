#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

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
  cout << "x_" << endl << x_ << endl << endl;
  cout << "F_" << endl << F_ << endl << endl;

  x_ = F_* x_;

  cout << "P_" << endl << P_ << endl << endl;
  cout << "Q_" << endl << Q_ << endl << endl;
  
  P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  // new state 
  x_ = x_ + (K_ * y);
  //long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(4, 4);
  P_ = (I_ - K_ * H_) * P_;

  cout << "Lidar update finished" << endl << endl;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = z(0);
  float py = z(1);
  float vx = z(2);
  float vy = z(3);

  VectorXd z_pred = VectorXd(3);

  z_pred << sqrt(px*px + py*py), atan2(py,px); (px*vx + py*vy)/(sqrt(px*px + py*py));

  VectorXd y = z - z_pred; // in radar we need to do h(x) instead of H*x as in the case of lidar;

  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  // new state 
  x_ = x_ + (K_ * y);
  //long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(4, 4);
  P_ = (I_ - K_ * H_) * P_;

  cout << "Radar update finished" << endl << endl;

}
