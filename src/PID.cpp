#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID():
	Kp(0.0),Ki(0.0),Kd(0.0),
	IsInitialized(false),
	p_error(0.0), i_error(0.0), d_error(0.0),
	prev_err(0.0),
	Kp_incr (0.005),
	Ki_incr (0.0002),
	Kd_incr (0.25),
	twiddler_next_param(0),
	run_counter(0),
	Twiddle_State_enum(TWIDDLE_STATE_ENUM::NEW_TWIDDLE_PARAM),
	twiddle_best_err (1e9),
	twiddle_err(0)
{}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Kd = Kd;
	this->Ki = Ki;
	this->IsInitialized = true;
}

void PID::UpdateError(double cte) {
	double error;
	error = (0.0 - cte);

	p_error = error;
	i_error = error + i_error;   // Previous error + current error
	i_error = i_error >  1.0 ? 1.0 : i_error;
	i_error = i_error < -1.0 ? -1.0 : i_error;

	d_error = error - prev_err;   // Current error - previous error

	// Update prev_err value
	prev_err = (0.0 - cte);


}

double PID::TotalError() {
	double tot_error;
	tot_error = Kp * p_error + Ki * i_error + Kd * d_error;

	// Limit total error between -1 to 1
	tot_error = tot_error >=  1.0 ?  1.0 : tot_error;
	tot_error = tot_error <= -1.0 ? -1.0 : tot_error;

	return tot_error;
}

void PID::Twiddle()
{
	double current_error;

	if (run_counter > 7000)
	{
		current_error = twiddle_err / run_counter;   // Divide error by total number of samples

		run_counter = 0;
		twiddle_err = 0.0;



		// Start twiddling operation
		if (Twiddle_State_enum == TWIDDLE_STATE_ENUM::NEW_TWIDDLE_PARAM)
		{


			twiddler_next_param = (twiddler_next_param + 1) > 2 ? 0 : twiddler_next_param + 1;

			switch (twiddler_next_param)
			{
				case 0:  // Increment Kp
					Kp = Kp + Kp_incr;
					break;
				case 1:
					Ki = Ki + Ki_incr;
					break;
				case 2:
					Kd = Kd + Kd_incr;
					break;
			}

			cout << "Twiddle State : NEW TWIDDLE PARAMETER" << std::endl;
			// Switch state to Check-Err for next iteration
			Twiddle_State_enum = TWIDDLE_STATE_ENUM::CHECK_TWIDDLE_ERROR;
		}
		else if (Twiddle_State_enum == TWIDDLE_STATE_ENUM::CHECK_TWIDDLE_ERROR)
		{
			// Debug output
			std::cout << "PID parameters are " << std::endl;
			std::cout << "====================" << std::endl;
			std::cout << "Kp  = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << std::endl;
			std::cout << "Kp_inc  = " << Kp_incr << ", Ki_inc = " << Ki_incr << ", Kd_inc = " << Kd_incr << std::endl;
			std::cout << "Current Error is : " << current_error << std::endl;
			// ===================================================================================


			if (current_error < twiddle_best_err)
			{
				twiddle_best_err = current_error;

				switch (twiddler_next_param)
				{
					case 0:
						Kp_incr = Kp_incr * 1.1;
						break;
					case 1:
						Ki_incr = Ki_incr * 1.1;
						break;
					case 2:
						Kd_incr = Kd_incr * 1.1;
						break;
				}

				Twiddle_State_enum = TWIDDLE_STATE_ENUM::NEW_TWIDDLE_PARAM;
				run_counter = 10000; // Force to jump into NEW_TWIDDLE_PARAM on next call

				cout << "Twiddle State : CHECK TWIDDLE ERROR (err  < twiddle best err)" << std::endl;

			}
			else
			{
				// Backup parameter produce error that is greater than best-err
				switch (twiddler_next_param)
				{
					case 0:
						Kp = Kp - 2 * Kp_incr;
						break;
					case 1:
						Ki = Ki - 2 * Ki_incr;
						break;
					case 2:
						Kd = Kd - 2 * Kd_incr;
						break;
				}

				// Go to double backup state, and re-run test for another 5000 iterations
				Twiddle_State_enum = TWIDDLE_STATE_ENUM::DOUBLEBACKUP_TWIDDLE_TUNING;

				cout << "Twiddle State : CHECK TWIDDLE ERROR (err  > twiddle best err)," << std::endl
					<< "Entering Double Backup Parameters " << std::endl;

			}
		}
		else if (Twiddle_State_enum == TWIDDLE_STATE_ENUM::DOUBLEBACKUP_TWIDDLE_TUNING)
		{
			// Debug output
			std::cout << "PID parameters are " << std::endl;
			std::cout << "====================" << std::endl;
			std::cout << "Kp  = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << std::endl;
			std::cout << "Kp_inc  = " << Kp_incr << ", Ki_inc = " << Ki_incr << ", Kd_inc = " << Kd_incr << std::endl;
			std::cout << "Current Error is : " << current_error << std::endl;
			// ===================================================================================


			if (current_error < twiddle_best_err)
			{
				twiddle_best_err = current_error;

				switch (twiddler_next_param)
				{
				case 0:
					Kp_incr = Kp_incr * 1.1;
					break;
				case 1:
					Ki_incr = Ki_incr * 1.1;
					break;
				case 2:
					Kd_incr = Kd_incr * 1.1;
					break;
				}

				// Go back to new parameter on next iteration
				Twiddle_State_enum = TWIDDLE_STATE_ENUM::NEW_TWIDDLE_PARAM;
				run_counter = 10000; // Force to jump into NEW_TWIDDLE_PARAM on next call

				std::cout << "TWIDDLE STATE: Double Backup Parameter (err < best_err)" << std::endl;

			}
			else
			{
				// Restore to default parameter
				switch (twiddler_next_param)
				{
				case 0:
					Kp = Kp + Kp_incr;
					Kp_incr = Kp_incr * 0.9;
					break;
				case 1:
					Ki = Ki  + Ki_incr;
					Ki_incr = Ki_incr * 0.9;
					break;
				case 2:
					Kd = Kd + Kd_incr;
					Kd_incr = Kd_incr * 0.9;
					break;
				}

				// Restore to default parameter 
				Twiddle_State_enum = TWIDDLE_STATE_ENUM::NEW_TWIDDLE_PARAM;
				run_counter = 10000;

				std::cout << "TWIDDLE STATE: Double Backup Parameter (err > best_err)" << std::endl
					<< "Restored default parameter " << std::endl;

			}


		}


	}


}

