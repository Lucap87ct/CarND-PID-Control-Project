#ifndef PID_H
#define PID_H
#include <array>
#include <limits>

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
  static constexpr int n_steps_tuning_{
      25}; // number of steps for twiddle tuning phase

  // Twiddle auto-tuning variables
  int index_tuning_{0};
  double tot_error_tuning_{0.0};
  double best_error_tuning_{std::numeric_limits<double>::max()};
  std::array<double, 3>
      dev_max_; // maximum deviation for twiddle parameter tuning
  std::array<double, 3> dev_tolerance_; // deviation tolerance
  std::array<double, 3> dev_current_;   // deviation tolerance
  bool twiddle_init_{true};
  bool twiddle_increase_gain_phase_{true};

  /**
   * Auto-tune control parameters of the PDI.
   */
  void UpdateControlParameters(const double cte);
};

#endif // PID_H
