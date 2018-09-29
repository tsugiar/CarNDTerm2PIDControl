#ifndef PID_H
#define PID_H

class PID {
public:

	enum class TWIDDLE_STATE_ENUM : int 
	{
		NEW_TWIDDLE_PARAM = 0,
		CHECK_TWIDDLE_ERROR,
		DOUBLEBACKUP_TWIDDLE_TUNING,
		RESTORE_DEFAULT_PARAMETER
	};

	TWIDDLE_STATE_ENUM Twiddle_State_enum;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prev_err;
  double twiddle_err;
  double twiddle_best_err;
  
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  bool IsInitialized;

  double Kp_incr;   // Kp incremental rate of increase for twiddler 
  double Ki_incr;   // Ki incremental rate of increase for twiddler
  double Kd_incr;   // Kd incremental rate of increase for twiddler

  int twiddler_next_param;  // Parame use
  int run_counter;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  *  Twiddle with PID parameters
  */
  void Twiddle();

};

#endif /* PID_H */
