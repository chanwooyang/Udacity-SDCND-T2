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
  is_initialized_ = false;

  // Initialize State dimension
  n_x_ = 5;

  // Initialize Augmented state dimension
  n_aug_ = 7;

  // Initialize Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // Initialize Weights of sigma points dimension
  weights_ = VectorXd(2*n_aug_+1);

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);


  // Need to be tuned
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Need to be tuned
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below; these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above; these are provided by the sensor manufacturer.

  // Initialize Xsig_pred dimension
  Xsig_pred_ = MatrixXd(n_x, 2*n_aug_+1);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!initialized) {
    // First Measurement
    cout << "UKF: " << endl;
    x_ << 1, 1, 1, 1;

    // Initialize the first timestamp
    time_us_ = meas_package.timestamp_;

    // Initialize Covariance Matrices
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;


    if (meas_package.sensor_type_ == meas_package::RADAR){
      // Convert radar measurement from ploar to cartesian
      // Initialize state
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float rho_d = meas_package.raw_measurements_[2];

      float px = rho * cos(phi);
      float py = rho * sin(phi);

      x_ << px, py, 0, 0, 0;

    } else if (meas_package.sensor_type_ == meas_package::LASER){
      // Initialize state
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];

      x_ << px, py, 0, 0, 0;

    }
  // Initialization complete
  is_initialized_ = true;
  return;
  }





}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} dt (delta t) the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // // Generate Sigma Points
  // MatrixXd Xsig = MatrixXd(n_x_,2*n_x_+1);
  // MatrixXd L = P_.llt().matrixL();

  // Xsig.col(0) = x_;
  // for (int i=0;i<n_x_;i++){
  //   Xsig.col(i+1) = x_ + sqrt(lambda_ + n_x_) * L.col(i);
  //   Xsig.col(i+1+n_x) = x - sqrt(lambda_ + n_x_) * L.col(i);
  // }

  // Generate Sigma Points with Augmentation
  VectorXd x_aug = VectorXd(n_aug_);        //[px, py, v, psi, psi_d, nu_a, nu_yaw_dd]
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);

  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;           // = Q(0,0)
  P_aug(6,6) = std_yawdd_ * std_yawdd_;   // = Q(1,1)

  MatrixXd L_aug = P_aug.llt().matrixL();

  Xsig_aug.col(0) = x_aug;
  for (int i=0;i<n_aug_;i++){
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda + n_aug_) * L_aug.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda + n_aug_) * L_aug.col(i);
  }

  // Sigma Points Prediction (Mapping Xsig to Nonlinear)
  for (int i=0;i<2*n_aug_+1;i++){
    VectorXd Xsig_col = Xsig_aug.col(i);
    float px = Xsig_col(0);
    float py = Xsig_col(1);
    float v = Xsig_col(2);
    float psi = Xsig_col(3);
    float psi_d = Xsig_col(4);
    float nu_a = Xsig_col(5);
    float nu_yaw_dd = Xsig_col(6);

    //Nonlinear Mapping
    if (fabs(psi_d) < 0.001){
      px += v * cos(psi) * dt;
      py += v * sin(psi) * dt;
    } else {
      px += (v/psi_d)*(sin(psi+psi_d*dt) - sin(psi));
      py += (v/psi_d)*(-cos(psi+psi_d*dt) + cos(psi));
    }
    psi += psi_d*dt;

    //Add Noise
    px += (1/2)*(dt*dt)*cos(psi)*nu_a;
    py += (1/2)*(dt*dt)*sin(psi)*nu_a;
    v += dt*nu_a;
    psi += (1/2)*(dt*dt)*nu_yaw_dd;
    psi_d += dt*nu_yaw_dd;

    Xsig_pred_.col(i) << px, py, v, psi, psi_d;
  }

  // Predict Mean & Covariance
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1;i<2*n_aug_+1;i++){
    weights_(i) = 1/(2*(lambda_+n_aug_));
  }

  for (int i=0;i<2*n_aug_+1;i++){
    x_ += weights_(i)*Xsig_pred_.col(i);          // Predict Mean
  }

  for (int i=0;i<2*n_aug_;i++){
    MatrixXd error = Xsig_pred_.col(i) - x_;
    P_ += weights_(i)*error*error.transpose();    // Predict Covariance
  }

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
}
