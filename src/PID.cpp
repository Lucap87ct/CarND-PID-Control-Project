#include "PID.h"
#include <limits>

PID::PID() {}

PID::~PID() {}

void PID::Init(const double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  cte_previous_ = 0.0;
}

void PID::UpdateError(const double cte) {
  p_error_ = cte;
  i_error_ += cte;
  d_error_ = cte - cte_previous_;
  cte_previous_ = cte;
}

double PID::TotalError() {
  double total_error{0.0};
  total_error += Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
  if (total_error > 1.0) {
    total_error = 1.0F;
  } else if (total_error < -1.0) {
    total_error = -1.0;
  }

  return total_error;
}
