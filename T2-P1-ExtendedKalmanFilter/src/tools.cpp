#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
							const vector<VectorXd> &ground_truth) {
	/***
	Calculate the RMSE here.
	***/

	// Initialize RMSE vector
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// Check the validity of estimations size and ground_truth size
	if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	// Accumulate squared residuals
	for (int i=0; i < estimations.size(); i++){
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	// Compute square-root of mean
	rmse = (rmse/estimations.size()).array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/***
	Hj = d_rho/d_px,     d_rho/d_py,     d_rho/d_vx,     d_rho/d_vy,
		d_phi/d_px,     d_phi/d_pu.     d_phi/d_vx,     d_phi/d_vy,
		d_rho-dot/d_px, d_rho-dot/d_py, d_rho-dot/d_vx, d_rho-dot/d_vy
	***/

	MatrixXd Hj(3,4);
	// Recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// Create substitutes
	float denom1 = px*px + py*py;
	float denom2 = sqrt(denom1);
	float denom3 = denom1*denom2;

	// Check division by zero
	if (fabs(denom1) < 0.0001){
		cout << "computeJacobian() - Error - Division by Zero" << endl;
		return Hj;
	}

	// Compute Jacobian Matrix
	Hj << (px/denom2), (py/denom2), 0, 0,
		  -(py/denom1), (px/denom1), 0, 0,
		  py*(vx*py - vy*px)/denom3, px*(vy*px - vx*py)/denom3, px/denom2, py/denom2;

	return Hj;
}