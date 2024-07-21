class PID {
public:
  PID(double kp, double ki, double kd);
  void setTunings(double kp, double ki, double kd);
  double compute(double setpoint, double input);

private:
  double kp_, ki_, kd_;
  double lastError_, integral_;
  unsigned long lastTime_;
};
