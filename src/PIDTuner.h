/*
 * pidtuner.h
 *
 *  Created on: 10.08.2018
 *      Author: academy
 */

#ifndef SRC_PIDTUNER_H_
#define SRC_PIDTUNER_H_

#include "PID.h"

enum class Mode { drive, tune };

class PIDTuner {
 public:
  PIDTuner(PID& pid_steering, PID& pid_throttle);
  virtual ~PIDTuner();
  bool hasFinishedRun();
  void updateParams();
 private:
  int ticks_ = 0;
  PID pid_steering_, pid_throttle_;
};

#endif /* SRC_PIDTUNER_H_ */
