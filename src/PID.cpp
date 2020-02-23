#include "PID.h"
#include <iostream>
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

  // initializing twiddle parameters
  dev_max_[0] = Kp * dev_rel_init;
  dev_max_[1] = Ki * dev_rel_init;
  dev_max_[2] = Kd * dev_rel_init;
  dev_tolerance_[0] = Kp * dev_rel_tolerance_;
  dev_tolerance_[1] = Ki * dev_rel_tolerance_;
  dev_tolerance_[2] = Kd * dev_rel_tolerance_;
  for (std::size_t i = 0; i < dev_current_.size(); i++) {
    dev_current_[i] = dev_max_[i];
  }
}

void PID::UpdateError(const double cte) {
  p_error_ = cte;
  i_error_ += cte;
  d_error_ = cte - cte_previous_;
  cte_previous_ = cte;

  UpdateControlParameters(cte);
}

double PID::TotalError() {
  double total_error{0.0};
  total_error += Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
  if (total_error > 1.0) {
    total_error = 1.0;
  } else if (total_error < -1.0) {
    total_error = -1.0;
  }

  return total_error;
}

void PID::UpdateControlParameters(const double cte) {
  bool is_tuning_ongoing{false};
  for (std::size_t i = 0; i < dev_tolerance_.size(); i++) {
    is_tuning_ongoing |= (dev_current_[i] > dev_tolerance_[i]);
  }

  if (is_tuning_ongoing) {
    std::cout << "Tuning ongoing" << std::endl;
    if (index_tuning_ == 0) {
      std::cout << "Tuning init" << std::endl;
      tot_error_tuning_ = 0.0;
      Kp_ += dev_current_[0];
      Ki_ += dev_current_[1];
      Kd_ += dev_current_[2];
      index_tuning_++;
    } else if (index_tuning_ < n_steps_tuning_) {
      tot_error_tuning_ += cte;
      std::cout << "Error calculation ongoing" << std::endl;
      index_tuning_++;
    } else {
      std::cout << "Tuning finished" << std::endl;
      std::cout << "Final total error = " << tot_error_tuning_ << std::endl;
      index_tuning_ = 0;
    }
  }
}
