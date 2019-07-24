#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	// accumulate squared residuals
	for (unsigned int i=0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - ground_truth[i];
		// coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	// calculate the mean
	rmse = rmse/estimations.size();

	// calculate the squared root
	rmse = rmse.array().sqrt();

	// return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);
	
	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// TODO: YOUR CODE HERE 
	float sum_px_py = pow(px,2) + pow(py,2);
	// check division by zero
	if (fabs(sum_px_py) < 0.0001){
		cout << "ERROR -Division by Zero !" << endl;
	  	return Hj;
	}
	// compute the Jacobian matrix
	Hj(0, 0) = px / sqrt(sum_px_py);
	Hj(0, 1) = py / sqrt(sum_px_py);
	Hj(1, 0) = -py / sum_px_py ;
	Hj(1, 1) = px / sum_px_py;    
	Hj(2, 0) = py * (vx*py-vy*px) / pow(sum_px_py, 3/2);
	Hj(2, 1) = px * (-vx*py+vy*px) / pow(sum_px_py, 3/2);       
	Hj(2, 2) = px / sqrt(sum_px_py);  
	Hj(2, 3) = py / sqrt(sum_px_py); 

	return Hj;
}
