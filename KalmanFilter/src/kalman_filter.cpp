#include "kalman_filter.h"

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
    x_ = F_*x_;
    MatrixXd FT= F_.transpose();
    P_= F_*P_*FT+Q_;
    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd y;    
    y= z - (H_ * x_);
    UpdateCommon(y);
   
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    
    double rho  = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    double thetha = atan(x_(1) / x_(0));
    double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    VectorXd  h = VectorXd(3);
    h << rho ,thetha,rho_dot;
    
    VectorXd  y = z-h;
    UpdateCommon(y);
    
}

void KalmanFilter::UpdateCommon(const Eigen::VectorXd& y)
{
    MatrixXd  Ht, S , Si ,K;
    Ht = H_.transpose();
    S = H_ * P_ * Ht + R_;
    Si = S.inverse();
    K= P_ * Ht * Si;
  
    x_ = x_+ (K * y);
    int size  = x_.size();
    MatrixXd I  = MatrixXd::Identity(size,size); 
    
    P_ = (I - K * H_)*P_; 
}