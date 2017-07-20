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
  //cout << "x_" << endl << x_ << endl << endl;
  //cout << "F_" << endl << F_ << endl << endl;

  //printf("%s %d\n",__FILE__,__LINE__);

  x_ = F_* x_;

  //cout << "P_" << endl << P_ << endl << endl;
  //cout << "Q_" << endl << Q_ << endl << endl;
  //printf("%s %d\n",__FILE__,__LINE__);
  
  P_ = F_ * P_ * F_.transpose() + Q_;
  //printf("%s %d\n",__FILE__,__LINE__);

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  //printf("%s %d\n",__FILE__,__LINE__);
  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  //printf("%s %d\n",__FILE__,__LINE__);
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();
  //printf("%s %d\n",__FILE__,__LINE__);

  // new state 
  x_ = x_ + (K_ * y);
  //printf("%s %d\n",__FILE__,__LINE__);
  //long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(4, 4);
  //printf("%s %d\n",__FILE__,__LINE__);
  P_ = (I_ - K_ * H_) * P_;

  //cout << "Lidar update finished" << endl << endl;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //printf("%s %d\n",__FILE__,__LINE__);

  //cout << "Vector z is " << z << endl; 
  //cout << "Vector x_ is " << x_ << endl; 
  float px = x_(0);
  //printf("%s %d\n",__FILE__,__LINE__);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  //printf("%s %d\n",__FILE__,__LINE__);

  VectorXd z_pred = VectorXd(3);
  //printf("%s %d\n",__FILE__,__LINE__);

  z_pred << sqrt(px*px + py*py), atan2(py,px), (px*vx + py*vy)/(sqrt(px*px + py*py));

  //printf("%s %d\n",__FILE__,__LINE__);

  VectorXd y = z - z_pred; // in radar we need to do h(x) instead of H*x as in the case of lidar;

  // normalizing y(1) to be between -pi and pi. 
  while (y(1) < M_PI)
    y(1) += 2*M_PI;
  while (y(1) > M_PI)
    y(1) -= 2*M_PI;

  //printf("%s %d\n",__FILE__,__LINE__);

  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;

  //printf("%s %d\n",__FILE__,__LINE__);
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  //printf("%s %d\n",__FILE__,__LINE__);

  // new state 
  x_ = x_ + (K_ * y);

  //printf("%s %d\n",__FILE__,__LINE__);
  //long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(4, 4);

  //printf("%s %d\n",__FILE__,__LINE__);
  P_ = (I_ - K_ * H_) * P_;

  //printf("%s %d\n",__FILE__,__LINE__);

  //cout << "Radar update finished" << endl << endl;

}
