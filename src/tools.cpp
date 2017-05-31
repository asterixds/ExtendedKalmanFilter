#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

const float EPS = 0.0001;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
 
  // estimation vector  len should be non-zero and equal ground truth vector len 
  if(estimations.size() != ground_truth.size()
  			|| estimations.size() == 0){
  	cout << "Invalid estimation or ground_truth data" << endl;
  	return rmse;
  }

  //calculate squared errors
  for(unsigned int i=0; i < estimations.size(); ++i){
  		VectorXd errors = estimations[i] - ground_truth[i];
  		errors = errors.array()*errors.array(); //coefficient-wise multiplication
  		rmse += errors;
  	}

  	//calculate the mean square root
  	rmse = rmse/estimations.size();
  	rmse = rmse.array().sqrt();

  	//return the result
  	return rmse;		
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj = MatrixXd::Zero(3, 4);
  //recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float c1 = px*px+py*py;
  
  //check division by zero
  if(fabs(c1) < EPS){
    cout << "c1 = " << c1 << endl;
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }
	float c2 = sqrt(c1);
	float c3 = (c1*c2);
  
  //compute the Jacobian 
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}
