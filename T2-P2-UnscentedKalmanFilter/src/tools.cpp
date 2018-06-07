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
	rmse << 0,0,0,0;

	// Check the validity of estimation size and ground_truth size
	if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
		cout << "Invalid estimation or ground_truth vector size" << endl;
		return rmse;
	}

	// Accumulate squared residuals
	for (int i=0;i<int(estimations.size());i++){
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	// Compute square-root of mean
	rmse = (rmse/estimations.size()).array().sqrt();

	return rmse;
}