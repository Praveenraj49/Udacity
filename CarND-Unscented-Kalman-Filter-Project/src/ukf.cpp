#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

using namespace std;

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
    std_a_ = 0.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.2;

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
    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_aug_;
    time_us_ = 0;

    x_ = VectorXd(n_x_);

    P_ = MatrixXd(n_x_, n_x_);

    P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1; 

    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    weights_ = VectorXd(2 * n_aug_ + 1);

    weights_(0) = lambda_ / (lambda_ + n_aug_);

    for (int i = 1; i < 2 * n_aug_ + 1; i++) {
        weights_(i) = 0.5 / (n_aug_ + lambda_);
    }

    //Noise Matrixes

    R_radar = MatrixXd(3, 3);
    R_laser = MatrixXd(2, 2);

    R_radar << std_radr_ * std_radr_, 0, 0,
            0, std_radphi_ * std_radphi_, 0,
            0, 0, std_radrd_ * std_radrd_;


    R_laser << std_laspx_ * std_laspx_, 0,
            0, std_laspy_ * std_laspy_;

    // NIS
    NIS_R, NIS_L = 0.0;

    Q = MatrixXd(2, 2);
    Q << std_a_ * std_a_, 0,
            0, std_yawdd_ * std_yawdd_;



}

UKF::~UKF() {
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {


    if (!is_initialized_) {
        double px, py, v, phi, yawd;
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = meas_package.raw_measurements_[0];
            double phi = meas_package.raw_measurements_[1];
            double v = meas_package.raw_measurements_[2];

            px = rho * cos(phi);
            py = rho * sin(phi);
            yawd = 0.0;
            x_ << px, py, v, phi, yawd;
        }

        if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            px = meas_package.raw_measurements_[0];
            py = meas_package.raw_measurements_[1];
            x_ << px, py, 0, 0, 0;
            

        }

        is_initialized_ = true;
        time_us_ = meas_package.timestamp_;
        return;
    }

    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    Prediction(delta_t);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);

    }

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package);
    }

    cout << "NIS Laser : " << NIS_L << endl;
    cout << "NSR Radar :" << NIS_R << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

    /*
     * 
     * Augmented Sigma Points Calculation 
     */


    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);


    //create augmented mean state
    //create augmented covariance matrix
    //create square root matrix
    //create augmented sigma points

    x_aug.fill(0.0);

    x_aug.block<5, 1>(0, 0) = x_;


    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug.bottomRightCorner(2, 2) = Q;




    MatrixXd A = P_aug.llt().matrixL();

    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
    }


    //predict sigma points
    //avoid division by zero
    //write predicted sigma points into right column


    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        double px = Xsig_aug(0, i);
        double py = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawdot = Xsig_aug(4, i);
        double noisev = Xsig_aug(5, i);
        double noiseyaw = Xsig_aug(6, i);

        double p_px, p_py;

        if (fabs(yawdot) > 0.001) {
            p_px = px + v / yawdot * (sin(yaw + yawdot * delta_t) - sin(yaw));
            p_py = py + v / yawdot * (-cos(yaw + yawdot * delta_t) + cos(yaw));
        } else {
            p_px = px + v * cos(yaw) * delta_t;
            p_py = py + v * sin(yaw) * delta_t;
        }
        Xsig_pred_(0, i) = p_px + 0.5 * delta_t * delta_t * cos(yaw) * noisev;
        Xsig_pred_(1, i) = p_py + 0.5 * delta_t * delta_t * sin(yaw) * noisev;
        Xsig_pred_(2, i) = v + delta_t*noisev;
        Xsig_pred_(3, i) = yaw + yawdot * delta_t + 0.5 * delta_t * delta_t*noiseyaw;
        Xsig_pred_(4, i) = yawdot + delta_t*noiseyaw;
    }

 


    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    P_.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd xdiff = Xsig_pred_.col(i) - x_;
        while (xdiff(3) > M_PI) xdiff(3) -= 2 * M_PI;
        while (xdiff(3) <-M_PI) xdiff(3) += 2 * M_PI;
        P_ = P_ + weights_(i) * xdiff * xdiff.transpose();
    }


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

    int n_z = 2;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);

    //transform sigma points into measurement space
    //calculate mean predicted measurement
    //calculate innovation covariance matrix S

    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        double px = Xsig_pred_(0, i);
        double py = Xsig_pred_(1, i);


        Zsig(0, i) = px;
        Zsig(1, i) = py;
    }

    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }


    // Update the prediction 

    MatrixXd Tc = MatrixXd(n_x_, n_z);

    //calculate cross correlation matrix
    //calculate Kalman gain K;
    //update state mean and covariance matrix

    // Incoming Lidar measurements
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

    S.fill(0.0);
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S = S + weights_(i) * z_diff * z_diff.transpose();
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;


        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    S = S + R_laser;

    MatrixXd K = MatrixXd(n_x_, n_z);
    K = Tc * S.inverse();

    VectorXd z_diff = z - z_pred;

    while(z_diff(1) > M_PI) z_diff(1) -= 2*M_PI;
    while(z_diff(1) < -M_PI) z_diff(1) += 2*M_PI;

    NIS_L = z_diff.transpose() * S.inverse() * z_diff;

    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

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
    MatrixXd S = MatrixXd(n_z, n_z);

    //transform sigma points into measurement space
    //calculate mean predicted measurement
    //calculate innovation covariance matrix S

    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        double px = Xsig_pred_(0, i);
        double py = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);

        double rho = sqrt(px * px + py * py);

        Zsig(0, i) = rho;
        Zsig(1, i) = atan2(py, px);
        if (rho < 0.0001) {
            Zsig(2, i) = (px * cos(yaw) * v + py * sin(yaw) * v) / 0.0001;
        }
        Zsig(2, i) = (px * cos(yaw) * v + py * sin(yaw) * v) / rho;
    }

    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }


    for (int i = 0; i < 2 * n_aug_ + 1; i++) {


    }


    // Update the prediction 

    MatrixXd Tc = MatrixXd(n_x_, n_z);

    //calculate cross correlation matrix
    //calculate Kalman gain K;
    //update state mean and covariance matrix

    // Incoming radar measurements
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

    S.fill(0.0);
    Tc.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //while(z_diff(1) > M_PI ) z_diff(1) -=2*M_PI;
        //while(z_diff(1) < -M_PI) z_diff(1) +=2*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();

        VectorXd xdiff = Xsig_pred_.col(i) - x_;
        while (xdiff(3) > M_PI) xdiff(3) -= 2 * M_PI;
        while (xdiff(3) < -M_PI) xdiff(3) += 2 * M_PI;

        Tc = Tc + weights_(i) * xdiff * z_diff.transpose();
    }

    S = S + R_radar;


    MatrixXd K = MatrixXd(n_x_, n_z);
    K = Tc * S.inverse();

    VectorXd zdiff = z - z_pred;

    while (zdiff(1) > M_PI) zdiff(1) -= 2 * M_PI;
    while (zdiff(1) < -M_PI) zdiff(1) += 2 * M_PI;



    x_ = x_ + K * zdiff;
    P_ = P_ - K * S * K.transpose();

    NIS_R = zdiff.transpose() * S.inverse() * zdiff;


}
