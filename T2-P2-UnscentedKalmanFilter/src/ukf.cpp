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
  std_a_ = 5;

  // Need to be tuned
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
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
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    // First Measurement
    cout << "UKF: " << endl;
    x_ << 1, 1, 1, 1, 1;

    // Initialize the first timestamp
    time_us_ = meas_package.timestamp_;

    // Initialize Covariance Matrices
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;


    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
      // Convert radar measurement from ploar to cartesian
      // Initialize state
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      // float rho_d = meas_package.raw_measurements_[2];

      float px = rho * cos(phi);
      float py = rho * sin(phi);

      x_ << px, py, 1, 1, 1;

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
      // Initialize state
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];

      x_ << px, py, 1, 1, 1;

    }
  // Initialization complete
  is_initialized_ = true;
  return;
  }

  cout << "init done" << endl;


  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  const float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  this -> Prediction(dt);

  cout << "pred done" << endl;


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true){
    this -> UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true) {
    this -> UpdateLidar(meas_package);
  }

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} dt (delta t) the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(const double dt) {
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
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ + n_aug_) * L_aug.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L_aug.col(i);
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
void UKF::UpdateLidar(const MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  // Initialize LIDAR Measurement Dimension: [px, py]
  int n_z = 2;

  // Predicted LIDAR Measurement is equal to Xsig
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd Zsig_pred = MatrixXd(n_z,2*n_aug_+1);

  for (int i=0;i<2*n_aug_+1;i++){
    VectorXd Xsig_pred_col = Xsig_pred_.col(i);

    float px = Xsig_pred_col(0);
    float py = Xsig_pred_col(1);

    Zsig_pred.col(i) << px, py;
  }
  for (int i=0;i<2*n_aug_+1;i++){
    z_pred += weights_(i)*Zsig_pred.col(i);
  }

  // Predict Innovation Covariance Matrix
  MatrixXd S = MatrixXd(n_z,n_z);
  MatrixXd R = MatrixXd(n_z,n_z);

  R.fill(0.0);
  R(0,0) = std_laspx_*std_laspx_;
  R(1,1) = std_laspy_*std_laspy_;

  for (int i=0;i<2*n_aug_+1;i++){
    S += weights_(i)*Zsig_pred.col(i);
  }

  S += R;

  // UKF Update: State Vector, Covariance

  // Cross-correlation Matrix
  MatrixXd Tc = MatrixXd(n_x_,n_z);
  for (int i=0;i<2*n_aug_+1;i++){
    VectorXd meas_pred_error = Zsig_pred.col(i) - z_pred;
    VectorXd state_pred_error = Xsig_pred_.col(i) - x_;

    Tc += weights_(i)*state_pred_error*meas_pred_error.transpose();
  }

  // Kalman Gain
  MatrixXd K = Tc*S.inverse();

  // Update State Vector
  VectorXd z = meas_package.raw_measurements_;
  x_ += K*(z - z_pred);

  // Update Covariance Matrix
  P_ += -K*S*K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  // Initialize Radar Measurement Dimension: [rho, phi, rho_d]
  int n_z = 3;


  // Predict Radar Measurement based on predicted Xsig: z_k+1|k = h(x_k+1|k) + w_k+1
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd Zsig_pred = MatrixXd(n_z,2*n_aug_+1);

  for (int i=0;i<2*n_aug_+1;i++){
    VectorXd Xsig_pred_col = Xsig_pred_.col(i);
    float px = Xsig_pred_col(0);
    float py = Xsig_pred_col(1);
    float v = Xsig_pred_col(2);
    float psi = Xsig_pred_col(3);
    // float psi_d = Xsig_pred_col(4);

    //Radar Measurement Model
    float rho = sqrt(px*px + py*py);
    float phi = atan2(py,px);
    float rho_d = (px*cos(psi)*v + py*sin(psi)*v)/sqrt(px*px + py*py);

    Zsig_pred.col(i) << rho, phi, rho_d;
  }

  for (int i=0;i<2*n_aug_+1;i++){
    z_pred += weights_(i)*Zsig_pred.col(i);
  }

  // Predict Innovation Covariance Matrix
  MatrixXd S = MatrixXd(n_z,n_z);
  MatrixXd R = MatrixXd(n_z,n_z);

  R.fill(0.0);
  R(0,0) = std_radr_*std_radr_;
  R(1,1) = std_radphi_*std_radphi_;
  R(2,2) = std_radrd_*std_radrd_;

  for (int i=0;i<2*n_aug_+1;i++){
    VectorXd pred_error = Zsig_pred.col(i) - z_pred;
    S += weights_(i)*pred_error*pred_error.transpose();
  }
  S += R;

  // UKF Update: State Vector, Covariance

  // Cross-correlation Matrix
  MatrixXd Tc = MatrixXd(n_x_,n_z);
  for (int i=0;i<2*n_aug_+1;i++){
    VectorXd state_pred_error = Xsig_pred_.col(i) - x_;
    VectorXd meas_pred_error = Zsig_pred.col(i) - z_pred;

    Tc += weights_(i)*state_pred_error*meas_pred_error.transpose();
  }

  // Kalman Gain
  MatrixXd K = Tc * S.inverse();

  // Update State Vector
  VectorXd z = meas_package.raw_measurements_;
  x_ += K*(z - z_pred);

  // Update Covariance Matrix
  P_ += -K*S*K.transpose();

}
