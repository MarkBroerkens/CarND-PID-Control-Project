/*
 * pidtuner.cpp
 *
 *  Created on: 10.08.2018
 *      Author: academy
 */

#include "PIDTuner.h"

#include "PID.h"

PIDTuner::PIDTuner(PID& pid_steering, PID& pid_throttle) {
  // TODO Auto-generated constructor stub

}

PIDTuner::~PIDTuner() {
  // TODO Auto-generated destructor stub
}

bool PIDTuner::hasFinishedRun() {
  ticks_ +=1;
  return (3000 < ticks_);
}

void PIDTuner::updateParams() {
  ticks_ = 0;
}

//void twiddle(double pp, double pi, double pd, double tolerance,
//             PID &pid) {
//  std::vector<double> p = {pp, pi, pd};
//  std::vector<double> dp = {1, 1, 1};
//
//  //double p[3] = { pp, pi, pd };
//  //double dp[3] = { 1, 1, 1 };
//
//  double sum = 0.0;
//
//  for (auto& n : dp) {
//    sum += n;
//  }
//
//  int iteration = 0;
//  while (sum > tolerance) {
//
//    sum = 0;
//    for (auto& n : dp) {
//      sum += n;
//    }
//
//    double best_error;
//
//    for (int i = 0; i < p.size(); ++i) {
//        p[i] += dp[i];
//        // initialize the pids
//
//    }
//
//    iteration += 1;
//  }
//}
