#include "kalman_filter.h"
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

const float DoublePI = 2 * M_PI;

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
  //Calculate new best estimate for position
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  //Calculate new best estimate for position uncertainty
  P_ = F_ *P_ *Ft + Q_;
  cout<< "Predict x_ =" << endl << x_<<endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // compare our predicated position (z) withour measured position (H_ * x_)
  VectorXd y = z - H_ * x_;
  KalmanCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  //convert predicated state values into polar coordinates
  float ro = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  
  // prevent division by 0
  if (ro < 0.0001) {
    ro = 0.0000001;
    }
    
  float theta = atan2(x_(1),x_(0));
  float ro_dot = (x_(0)*x_(2) + x_(1)*x_(3))/ro;
  
  VectorXd Hj(3);
  Hj << ro, theta, ro_dot;
  cout<< "Theta" << theta << endl;
  VectorXd y = z - Hj;
  
    // normalize the angle between -pi to pi
  while(y(1) > M_PI){
    y(1) -= DoublePI;
  }

  while(y(1) < -M_PI){
    y(1) += DoublePI;
}
  KalmanCommon(y);
}

void KalmanFilter::KalmanCommon(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  
  //new state
  // new position
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  //new covariance matrix
  P_ = (I - K * H_) * P_;
  cout<< "Update x_ =" << endl << x_<< endl;

}
