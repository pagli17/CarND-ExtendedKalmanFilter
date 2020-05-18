#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
    std::cout  << "KalmanFilter::Predict()" << std::endl;
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

    std::cout << "KalmanFilter::Update" << std::endl;
 	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;

    // Update the remaining values.
    // This is common for Lidar and Radar
    UpdateCommon(y);
}

void KalmanFilter::UpdateCommon(const VectorXd &y){
  
  	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_ * Ht;
  	MatrixXd S = H_ * PHt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = PHt * Si;

	x_ = x_ + (K * y);
	long x_dimension = x_.size();
	MatrixXd I = MatrixXd::Identity(x_dimension, x_dimension);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    std::cout << "KalmanFilter::UpdateEKF" << std::endl;

	float px = x_(0);
	float py = x_(1);

    
    float rho = sqrt(px*px + py*py);
    if(fabs(rho) < 0.0001) { // Avoid dividing by zero
      return;
    }

    if(py == 0 && px == 0) {// Atan2(0,0) would give error.
      return;
    }
    float theta = atan2(py, px);

    float vx = x_(2);
    float vy = x_(3);
    float rho_dot = (px*vx + py*vy)/rho;

    VectorXd z_pred(3);
    z_pred << rho, theta, rho_dot;

    VectorXd y = z - z_pred;

    // Normalizing Angles between -pi and pi
    y[1] = atan2(sin(y[1]), cos(y[1]));


    // Update the remaining values.
    // This is common for Lidar and Radar
    UpdateCommon(y);
}
