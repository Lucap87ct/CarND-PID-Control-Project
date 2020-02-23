#ifndef PID_H
#define PID_H

class PID {
public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  void Init(const double Kp, const double Ki, const double Kd);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(const double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Auto-tune control parameters of the PDI.
   */
  void UpdateControlParameters(const double cte);

private:
  /**
   * PID Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;
  double cte_previous_;

  /**
   * PID Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;
  static constexpr double d_tol_rel_{0.1};
};

#endif // PID_H
