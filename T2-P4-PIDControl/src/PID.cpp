#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kprop, double Kinteg, double Kdiff) {
	Kp = Kprop;
	Ki = Kinteg;
	Kd = Kdiff;

	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	prev_cte = 0.0;
}

void PID::UpdateError(double cte) {
	p_error = cte;
	d_error = cte - prev_cte;
	prev_cte = cte;
	i_error += cte;
	i_error = min(i_error,10.0);
	i_error = max(i_error,-10.0);

	cout << "P error: " << p_error << endl;
	cout << "I error: " << i_error << endl;
	cout << "D error: " << d_error << endl;

}

double PID::ControlInput(){
	return -Kp*p_error - Ki*i_error - Kd*d_error;
}

double PID::TotalError() {
}

