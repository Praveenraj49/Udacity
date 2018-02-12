#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ =0.5 ;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  // Initialize Values 
  
  is_initialized_ = false;
  n_x_ =5;
  n_aug_ =7;
  lambda_  = 0;
  time_us_ =0;
  
          
  x_ = VectorXd(5);
  P_ = MatrixXd(5,5);
  
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  weights_ = VectorXd(2*n_aug_+1);
  
  //Noise Matrixes
  
     R_radar  =MatrixXd(3,3);
    R_laser = MatrixXd(2,2);
    
    // NIS
    NIS_R , NIS_L =0.0;
  
 
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    
    
    if(!is_initialized_)
    {
        if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            double rho = meas_package.raw_measurements_(0);
            double phi  = meas_package.raw_measurements_(1);
            double rhodot = meas_package.raw_measurements_(2);
            
            x_ << rho*cos(phi), rho *sin(phi) ,4 , rhodot * cos(phi) , rhodot * sin(phi) ;
            
            P_ << std_radr_*std_radr_ ,0,0,0,0,
                 0,std_radr_*std_radr_,0,0,0,
                 0,0,1,0,0,
                 0,0,0,std_radphi_*std_radphi_,0,
                 0,0,0,0,std_radphi_*std_radphi_;
            
            R_radar <<std_radr_*std_radr_ ,0,0,
                      0,std_radphi_*std_radphi_,0,
                      0,0,std_radrd_*std_radrd_;
        }
        
        if(meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            x_ << meas_package.raw_measurements_(0) ,  meas_package.raw_measurements_(1),4.0,0.5,0;
            
            P_ << std_laspx_*std_laspx_,0,0,0,0,
                  0,std_laspy_*std_laspy_,0,0,0,
                  0,0,1,0,0,
                  0,0,0,1,0,
                  0,0,0,0,1;
            
            R_laser << std_laspx_*std_laspx_,0,
                       0,std_laspy_*std_laspy_;
                    
        }
        
        is_initialized_ = true;
        time_us_ = meas_package.timestamp_;
        return;
    }
    
    double delta_t  = (meas_package.timestamp_ -time_us_)/1000000.0;
    time_us_ = meas_package.timestamp_;
    
    Prediction(delta_t);
    
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        UpdateRadar(meas_package);
        
    }
    
    if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        UpdateLidar(meas_package);
    }
    
    
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
    
    MatrixXd Xsig = MatrixXd(n_x_ , 2*n_x_+1);

    //Calculate the square root  of P
    MatrixXd A = P_.llt().matrixL();

    lambda_ = 3-n_x_;

    Xsig.col(0) = x_;



    for(int i=0;i<n_x_;i++)
    {
        Xsig.col(i+1) = x_ + sqrt(lambda_+n_x_)*A.col(i);
        Xsig.col(i+n_x_+1) = x_ - sqrt(lambda_+n_x_)*A.col(i);
    }


    lambda_  = 3-n_aug_;

      //create augmented mean vector
    VectorXd x_aug = VectorXd(7);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);


     //create augmented mean state
    //create augmented covariance matrix
    //create square root matrix
    //create augmented sigma points

    x_aug.head(n_x_) = x_;
    x_aug(5) =0;
    x_aug(6) =0;

    P_aug.fill(0);
    P_aug.topLeftCorner(n_x_, n_x_) =P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;

    A = P_aug.llt().matrixL();

    Xsig_aug.col(0) = x_aug;
    for(int i=0;i<n_aug_;i++)
    {
        Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*A.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug- sqrt(lambda_+n_aug_)*A.col(i);
    }
    
    //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  
  
  for(int i=0;i<2*n_aug_+1;i++)
  {
      double px = Xsig_aug(0,i);
      double py = Xsig_aug(1,i);
      double v  = Xsig_aug(2,i);
      double yaw = Xsig_aug(3,i);
      double yawdot = Xsig_aug(4,i);
      double noisev = Xsig_aug(5,i);
      double noiseyaw  = Xsig_aug(6,i);
      
      double p_px, p_py;
      
      if(fabs(yawdot)>0.0001)
      {
          p_px  =v/yawdot*(sin(yaw+yawdot*delta_t) - sin(yaw));
          p_py = v/yawdot*(-cos(yaw+yawdot*delta_t)+cos(yaw));
      }
      else
      {
          p_px = v*cos(yaw)*delta_t;
          p_py = v*sin(yaw)*delta_t;
      }
      Xsig_pred_(0,i) = px+p_px+0.5*delta_t*delta_t*cos(yaw)*noisev;
      Xsig_pred_(1,i) = py+p_py+0.5*delta_t*delta_t*sin(yaw)*noisev;
      Xsig_pred_(2,i) = v+delta_t*noisev;
      Xsig_pred_(3,i) = yaw+yawdot*delta_t+0.5*delta_t*delta_t*noiseyaw;
      Xsig_pred_(4,i) = yawdot+delta_t*noiseyaw;
  }
    
     //create vector for predicted state
  VectorXd x_pred = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P_pred = MatrixXd(n_x_, n_x_);

   weights_(0) = lambda_/(lambda_+n_aug_);
  for(int i=1;i<2*n_aug_+1;i++)
  {
      weights_(i) = 0.5/(lambda_+n_aug_);
  } 
   
   x_pred.fill(0.0);
    for(int i=0;i<2*n_aug_+1;i++)
  {
      x_pred = x_pred + weights_(i)* Xsig_pred_.col(i);
  }

   P_pred.fill(0.0);
   
     for(int i=0;i<2*n_aug_+1;i++)
  {
      VectorXd xdiff = Xsig_pred_.col(i) - x_;
      while(xdiff(3) > M_PI) xdiff(3) -=2*M_PI;
      while(xdiff(3) <-M_PI) xdiff(3) +=2*M_PI;
      P_pred = P_pred+ weights_(i) * xdiff * xdiff.transpose();
  }

  x_ = x_pred;
  P_ = P_pred;
   
      
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
    
    int n_z = 2;
   
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  
  //transform sigma points into measurement space
  //calculate mean predicted measurement
  //calculate innovation covariance matrix S
  
  for(int i=0;i< 2*n_aug_+1;i++)
  {
      double  px  = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      

      Zsig(0,i) = px;
      Zsig(1,i)  = py;
  }

    z_pred.fill(0.0);
    for(int i=0;i< 2*n_aug_+1;i++)
    {
        z_pred = z_pred + weights_(i)*Zsig.col(i);
    }

    S.fill(0.0);
    for(int  i=0;i<2*n_aug_+1;i++)
    {
        VectorXd z_diff  = Zsig.col(i) -z_pred;

        S = S + weights_(i) * z_diff * z_diff.transpose();

    }

    S  = S+R_laser;
    
    // Update the prediction 
    
      MatrixXd Tc = MatrixXd(n_x_, n_z);
      
      //calculate cross correlation matrix
  //calculate Kalman gain K;
  //update state mean and covariance matrix
  
    // Incoming Lidar measurements
     VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1);
    
    Tc.fill(0.0);
    for(int  i=0;i<2*n_aug_+1;i++)
    {
       VectorXd xdiff  = Xsig_pred_.col(i) -x_;
       VectorXd zdiff  = Zsig.col(i) - z_pred;
       while(xdiff(3) > M_PI) xdiff(3) -= 2*M_PI;
       while(xdiff(3) < -M_PI) xdiff(3) += 2*M_PI;


       Tc  = Tc+ weights_(i)* xdiff * zdiff.transpose();
    }

    MatrixXd  K  = MatrixXd(n_x_, n_z);
     K = Tc * S.inverse();

     VectorXd zdiff = z - z_pred;

     while(zdiff(1) > M_PI) zdiff(1) -= 2*M_PI;
     while(zdiff(1) < -M_PI) zdiff(1) += 2*M_PI;
     
     NIS_L = zdiff.transpose()*S.inverse()*zdiff;

     x_ = x_+ K*zdiff;
     P_ = P_ - K*S*K.transpose();

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
    
    int n_z = 3;
   
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  
  //transform sigma points into measurement space
  //calculate mean predicted measurement
  //calculate innovation covariance matrix S
  
  for(int i=0;i< 2*n_aug_+1;i++)
  {
      double  px  = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);
      
      double rho = sqrt(px*px + py*py);
      Zsig(0,i) = rho;
      Zsig(1,i)  = atan2(py,px);
      Zsig(2,i) = (px*cos(yaw)*v + py*sin(yaw)*v)/rho;
  }

    z_pred.fill(0.0);
    for(int i=0;i< 2*n_aug_+1;i++)
    {
        z_pred = z_pred + weights_(i)*Zsig.col(i);
    }

    S.fill(0.0);
    for(int  i=0;i<2*n_aug_+1;i++)
    {
        VectorXd z_diff  = Zsig.col(i) -z_pred;

        while(z_diff(1) > M_PI ) z_diff(1) -=2*M_PI;
        while(z_diff(1) < -M_PI) z_diff(1) +=2*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();

    }

    S  = S+R_radar;
    
    // Update the prediction 
    
      MatrixXd Tc = MatrixXd(n_x_, n_z);
      
      //calculate cross correlation matrix
  //calculate Kalman gain K;
  //update state mean and covariance matrix
  
    // Incoming radar measurements
     VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1),meas_package.raw_measurements_(2);
    
    Tc.fill(0.0);
    for(int  i=0;i<2*n_aug_+1;i++)
    {
       VectorXd xdiff  = Xsig_pred_.col(i) -x_;
       VectorXd zdiff  = Zsig.col(i) - z_pred;
       while(xdiff(3) > M_PI) xdiff(3) -= 2*M_PI;
       while(xdiff(3) < -M_PI) xdiff(3) += 2*M_PI;

       while(zdiff(1) > M_PI) zdiff(1) -= 2*M_PI;
       while(zdiff(1) < -M_PI) zdiff(1) += 2*M_PI;

       Tc  = Tc+ weights_(i)* xdiff * zdiff.transpose();
    }

    MatrixXd  K  = MatrixXd(n_x_, n_z);
     K = Tc * S.inverse();

     VectorXd zdiff = z - z_pred;

     while(zdiff(1) > M_PI) zdiff(1) -= 2*M_PI;
     while(zdiff(1) < -M_PI) zdiff(1) += 2*M_PI;
     
     NIS_R  = zdiff.transpose()*S.inverse()*zdiff;

     x_ = x_+ K*zdiff;
     P_ = P_ - K*S*K.transpose();

}
