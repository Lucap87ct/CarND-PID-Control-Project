#include "PID.h"
#include <cmath>
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
    if (index_tuning_ == 0) {
      if (!twiddle_init_) {
        Kd_ += dev_current_[2];
      }
      tot_error_tuning_ = 0.0;
      std::cout << "Current Kd = " << Kd_ << std::endl;
      std::cout << "Current Dev Kd = " << dev_current_[2] << std::endl;
      index_tuning_++;
    }

    else if (index_tuning_ < n_steps_tuning_) {
      tot_error_tuning_ += fabs(cte);
      index_tuning_++;
    }

    else {
      std::cout << "Final total error = " << tot_error_tuning_ << std::endl;
      if (!twiddle_init_) {

        if (tot_error_tuning_ < best_error_tuning_) {
          std::cout << "Error improved" << std::endl;
          best_error_tuning_ = tot_error_tuning_;
          Kd_ += dev_current_[2];
          for (std::size_t i = 0; i < dev_current_.size(); i++) {
            dev_current_[i] *= 1.1;
          }
        } else {
          std::cout << "Error did not improve" << std::endl;
          Kd_ -= 2.0 * dev_current_[2];
          twiddle_increase_gain_phase_ = false;
        }
      }

      else {
        best_error_tuning_ = tot_error_tuning_;
        twiddle_init_ = false;
      }

      index_tuning_ = 0;
    }
  }

  /*
    if (is_tuning_ongoing) {
      if (index_tuning_ == 0) {
        if (first_twiddle_tuning_) {
          // Kp_ += dev_current_[0];
          Kd_ += dev_current_[2];
          first_twiddle_tuning_ = false;
        }
        tot_error_tuning_ = 0.0;
        // std::cout << "Current Kp = " << Kp_ << std::endl;
        // std::cout << "Current Dev Kp = " << dev_current_[0] << std::endl;
        std::cout << "Current Kd = " << Kd_ << std::endl;
        std::cout << "Current Dev Kd = " << dev_current_[2] << std::endl;
        index_tuning_++;
      } else if (index_tuning_ < n_steps_tuning_) {
        tot_error_tuning_ += fabs(cte);
        index_tuning_++;
      } else {
        std::cout << "Final total error = " << tot_error_tuning_ << std::endl;
        if (!twiddle_init_) {
          if (tot_error_tuning_ < best_error_tuning_) {
            std::cout << "Error improved" << std::endl;
            best_error_tuning_ = tot_error_tuning_;
            // Kp_ += dev_current_[0];
            Kd_ += dev_current_[2];
            for (std::size_t i = 0; i < dev_current_.size(); i++) {
              dev_current_[i] *= 1.1;
            }
          } else {
            std::cout << "Error did not improve" << std::endl;
            Kp_ -= dev_current_[0];
            Kd_ -= dev_current_[2];
            for (std::size_t i = 0; i < dev_current_.size(); i++) {
              dev_current_[i] *= 0.9;
            }
          }
        } else {
          best_error_tuning_ = tot_error_tuning_;
          twiddle_init_ = false;
          first_twiddle_tuning_ = true;
        }
        index_tuning_ = 0;
      }
    }*/
}
