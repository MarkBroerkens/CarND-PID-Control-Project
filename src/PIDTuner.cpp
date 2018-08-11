/*
 * pidtuner.cpp
 *
 *  Created on: 10.08.2018
 *      Author: academy
 */

#include "PIDTuner.h"
#include <iostream>

#include "PID.h"

PIDTuner::PIDTuner(PID& pid_steering, PID& pid_throttle, double tolerance)
    : pid_steering_(pid_steering),
      pid_throttle_(pid_throttle),
      tolerance_(tolerance),
      ran_for_steps_(0),
      iterations_(0),
      params_ { pid_steering.Kp, pid_steering.Ki, pid_steering.Kd /*, pid_throttle.Kp, pid_throttle.Ki, pid_throttle.Kd */},
      d_params_ { 0.5, 0.5, 0.5 /*, 0.5, 0.5, 0.5 */},
      best_err_(-1),
      curr_error_(0),
      twiddle_state_(START),
      twiddle_param_(0),
      error_(0),
      ignore_initial_steps(100) {
}

PIDTuner::~PIDTuner() {
}

bool PIDTuner::hasFinishedRun() {
  ran_for_steps_ += 1;
  return (2000 < ran_for_steps_);
}

bool PIDTuner::isOffTrack(double cte, double speed) {
  if (ran_for_steps_ > 2 * ignore_initial_steps) {
    return speed < 4 || cte > 6;
  } else {
    return false;
  }
}

void PIDTuner::accumulateCTE(double cte) {
  if (ran_for_steps_ > ignore_initial_steps) {
    error_ += cte * cte;
  }
}

double PIDTuner::calcAverageError() {
  return error_ / (ran_for_steps_ - ignore_initial_steps);
}

void PIDTuner::twiddle() {
  curr_error_ = calcAverageError();

  double sum = 0.0;
  for (auto& n : d_params_) {
    sum += n;
  }
  // we only have to twiddle if the sum of the parameter modification steps
  // is smaller than the tolerance
  if (sum > tolerance_) {
    switch (twiddle_state_) {
      case START: {
        best_err_ = curr_error_;
        params_[twiddle_param_] += d_params_[twiddle_param_];
        twiddle_state_ = INCREMENTING;
        break;
      }
      case INCREMENTING: {
        if (curr_error_ < best_err_) {
          // incrementing work well, so we increment even more next time
          best_err_ = curr_error_;
          d_params_[twiddle_param_] *= 1.1;
          goToNextParam();
          params_[twiddle_param_] += d_params_[twiddle_param_];
        } else {
          // otherwise try in the opposite direction
          params_[twiddle_param_] -= 2 * d_params_[twiddle_param_];
          twiddle_state_ = DECREMENTING;
        }
        break;
      }
      case DECREMENTING: {
        if (curr_error_ < best_err_) {
          // decrementing work well, so we increment even more next time
          best_err_ = curr_error_;
          d_params_[twiddle_param_] *= 1.1;
        } else {
          // otherwise we reset the current parameter to its original value and
          // reduce the amount modification
          params_[twiddle_param_] += d_params_[twiddle_param_];
          d_params_[twiddle_param_] *= 0.9;
        }
        twiddle_state_ = INCREMENTING;
        goToNextParam();
        params_[twiddle_param_] += d_params_[twiddle_param_];
        break;
      }
    }
    pid_steering_.Init(params_[0], params_[1], params_[2]);
    /* pid_throttle_.Init(params_[3], params_[4], params_[5]); */
  }
  ran_for_steps_ = 0;
  error_ = 0;
}

void PIDTuner::goToNextParam() {
  twiddle_param_ = (twiddle_param_ + 1) % params_.size();
  if (twiddle_param_ == 0) {
    ++iterations_;
  }
}

void PIDTuner::print() {
  std::cout << "Iter: " << iterations_ << " best_err: " << best_err_
            << " curr err: " << curr_error_ << " current param: "
            << twiddle_param_ << " current state: " << twiddle_state_
            << std::endl;
  std::cout << "next params: " << params_[0] << ", " << params_[1] << ", "
            << params_[2] << ", learning:" << std::endl;
}