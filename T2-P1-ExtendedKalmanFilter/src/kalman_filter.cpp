#include "kalman_filter.h"
#include "tools.h"

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;  // State Vector
  P_ = P_in;  // State Covariance Matrix
  F_ = F_in;  // State Transition Matrix (= A in Control System Theory)
  H_ = H_in;  // Measurement Matrix (= C in Control System Theory)
  R_ = R_in;  // Measurement Covariance Matrix
  Q_ = Q_in;  // Process Covariance Matrix
}

void KalmanFilter::Predict() {
  /***
  Predict the state

  Linear Motion
  ***/
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /***
  update the state by using Kalman Filter equations
  ***/
  MatrixXd Ht = H_.transpose();   // Transpose of Measurement Matrix, H
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_*Ht*S_inv;       // Kalman Gain: K = P*C_T*(R + C*P*C_T)^-1

  VectorXd z_pred = H_ * x_;      // Predicted Sensor Measurement
  VectorXd y = z - z_pred;        // Difference between measurement and predicted measurement

  // New Estimate
  x_ = x_ + (K * y);              // Update state matrix
  P_ = P_ - K * H_ * P_;          // Update state covariance matrix
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /***
  update the state by using Extended Kalman Filter equations
  
  Non-linear System:
  x_t+1 = f(x_t)
  z_t+1 = h(x_t)

  For Radar Measurement:
  [rho, phi, rho_dot].transpose() = [H] * [px, py, vx, vy].transpose()

  H is non-linear matrix, so Jacobian Matrix of H, Hj, needs to be computed.
  ***/
  MatrixXd Ht = H_.transpose();   // Transpose of Measurement Matrix, H
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_*Ht*S_inv;       // Kalman Gain: K = P*C_T*(R + C*P*C_T)^-1

  // Retrieve state
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  // z_t+1 = h(x_t)
  VectorXd z_pred(3);
  z_pred(0) = sqrt(px*px+py*py);                // rho
  
  // "atan2()" outputs an angle between -pi and pi
  z_pred(1) = atan2(py,px);                     // phi

  z_pred(2) = (px*vx+py*vy)/sqrt(px*px+py*py);  // rho_dot

  // Check if a raw measurement of 'phi' is within the range between -pi and pi
  
  // For Debugging purpose
  // cout << "pred_phi: " << z_pred(1) << endl;
  // cout << "z_raw_phi: " << z(1) << endl;
  // cout << "M_PI: "  << M_PI << endl;

  const float precision = 10000.0;
  if(z(1)*precision > M_PI*precision && z(1)*z_pred(1) < 0){
    z_pred(1) += 2*M_PI;
    // For Debugging purpose
    // cout << "pred_phi: " << z_pred(1) << endl;
    // cout << "z_raw_phi: " << z(1) << endl;
  } else if(z(1)*precision < -M_PI*precision && z(1)*z_pred(1) < 0){
    z_pred(1) -= 2*M_PI;
    // For Debugging purpose
    // cout << "pred_phi: " << z_pred(1) << endl;
    // cout << "z_raw_phi: " << z(1) << endl;
  } else if(z(1)*z_pred(1) < 0){
    if (z(1) > 3){
      z_pred(1) += 2*M_PI;
    } else if (z(1) < -3){
      z_pred(1) -= 2*M_PI;
    }
    // For Debugging purpose
    // cout << "pred_phi: " << z_pred(1) << endl;
    // cout << "z_raw_phi: " << z(1) << endl;
  }

  VectorXd y = z - z_pred;

  // New Estimate
  x_ = x_ + (K * y);      // Update state matrix
  P_ = P_ - K * H_ * P_;  // Update state covariance matrix
}