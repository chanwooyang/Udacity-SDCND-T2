#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <random>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
  // H_laser
  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;
  
  //  Initial State Covariance Matrix
  // default_random_engine generator;
  // normal_distribution<float> distribution(0.0, 0.05);
  // float ran_num[4];
  // for (int i=0 ; i<4 ; i++){
  //   srand(time(0));
  //   ran_num[i] = distribution(generator);
  //   cout << ran_num[i] << endl;
  // }

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
  // Initial Process Covariance Matrix
  ekf_.Q_ = MatrixXd(4, 4);

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /***
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    ***/
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // initialize first timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /***
      Convert radar from polar to cartesian coordinates and initialize state.
      ***/
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];

      float px = rho * cos(phi);
      float py = rho * sin(phi);

      ekf_.x_ << px, py, 5, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /***
      Initialize state.
      ***/
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];

      ekf_.x_ << px, py, 5, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /***
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   ***/

  // Compute the time step, dt, in sec
  const float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  const float dt2 = dt*dt;
  const float dt3 = dt2*dt;
  const float dt4 = dt3*dt;

  // Update the State Transition Matrix with time step, dt, in sec
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Update Process Noise Covariance Matrix
  ekf_.Q_ << (dt4/4)*noise_ax, 0, (dt3/2)*noise_ax, 0,
             0, (dt4/4)*noise_ay, 0, (dt3/2)*noise_ay,
             (dt3/2)*noise_ax, 0, dt2*noise_ax, 0,
             0, (dt3/2)*noise_ay, 0, dt2*noise_ay;



  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /***
  Use the sensor type to perform the update step.
  Update the state and covariance matrices.
  ***/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    Hj_ = tools.CalculateJacobian(ekf_.x_);    // Jacobian Matrix
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
