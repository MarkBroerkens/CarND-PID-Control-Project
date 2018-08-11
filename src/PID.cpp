#include "PID.h"

PID::PID() {
  Init(0, 0, 0);
}

PID::~PID() {
}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  // here, the p_error contains the value of the previous cte
  d_error = cte - p_error;

  p_error = cte;

  i_error += cte;
}

double PID::TotalError() {
  return p_error * Kp + i_error * Ki + d_error * Kd;
}

double PID::GetControl() {
  double control_value = -Kp*p_error - Kd*d_error - Ki*i_error;
  if (control_value < -1) {
    control_value = -1;
  } else if (control_value > 1) {
    control_value = 1;
  }
  return control_value;
}
