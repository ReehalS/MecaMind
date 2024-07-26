#include "Rotation_PID.h"
#include <Arduino.h> // Include this for millis()

Rotation_PID::Rotation_PID(double kp, double ki, double kd)
  : kp_(kp), ki_(ki), kd_(kd), lastError_(0), integral_(0), lastTime_(0) {}

void Rotation_PID::setTunings(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

double Rotation_PID::compute(double setpoint, double input) {
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime_) / 1000.0; // Convert to seconds
  
  double error = setpoint - input;
  integral_ += error * timeChange;
  double derivative = (error - lastError_) / (timeChange + 1e-6); // Add epsilon to avoid division by zero

  lastError_ = error;
  lastTime_ = now;

  return kp_ * error + ki_ * integral_ + kd_ * derivative;
}
