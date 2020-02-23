#ifndef PID_H
#define PID_H
#include <array>

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

  // Twiddle auto-tuning parameters
  static constexpr double dev_rel_init{
      0.1}; // relative deviation for twiddle initialization
  static constexpr double dev_rel_tolerance_{
      0.01}; // relative tolerance for twiddle completion
  static constexpr int n_steps_tune{
      100}; // number of steps for twiddle tuning phase
  static constexpr int n_steps_validation{
      1000}; // number of steps for twiddle validation phase

  // Twiddle auto-tuning variables
  int index_tuning{0};
  int index_validation{0};
  std::array<double, 3>
      dev_max_; // maximum deviation for twiddle parameter tuning
  std::array<double, 3> dev_tolerance_; // deviation tolerance
  std::array<double, 3> dev_current_;   // deviation tolerance

  /**
   * Auto-tune control parameters of the PDI.
   */
  void UpdateControlParameters(const double cte);
};

#endif // PID_H
