#include "Direction_PID.h"

// Constructor
Direction_PID::Direction_PID(double kp, double ki, double kd)
  : kp_(kp), ki_(ki), kd_(kd), lastError_(0), integral_(0), lastTime_(millis()), integralMin_(-1000), integralMax_(1000) {}

// Set PID tunings
void Direction_PID::setTunings(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

// Compute PID output
double Direction_PID::compute(double setpoint, double input) {
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime_);
  
    // Prevent division by zero
    if (timeChange <= 0) {
        return 0;
    }

    double error = setpoint - input;
    integral_ += error * timeChange;
    
    // Clamp integral to prevent windup
    integral_ = constrain(integral_, integralMin_, integralMax_);

    double derivative = (error - lastError_) / timeChange;

    lastError_ = error;
    lastTime_ = now;

    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}
