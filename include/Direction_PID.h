#ifndef DIRECTION_PID_H
#define DIRECTION_PID_H

#include <Arduino.h>

class Direction_PID {
public:
    Direction_PID(double kp, double ki, double kd);
    void setTunings(double kp, double ki, double kd);
    double compute(double setpoint, double input);

private:
    double kp_;         // Proportional gain
    double ki_;         // Integral gain
    double kd_;         // Derivative gain
    double lastError_;  // Previous error value
    double integral_;   // Integral term
    unsigned long lastTime_; // Last computation time

    // Limits for integral term to prevent windup
    double integralMin_;
    double integralMax_;
};

#endif 
