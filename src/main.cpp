#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "PID.h"

// Motor shield setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorA = AFMS.getMotor(1);
Adafruit_DCMotor *motorB = AFMS.getMotor(2);
Adafruit_DCMotor *motorD = AFMS.getMotor(3);
Adafruit_DCMotor *motorC = AFMS.getMotor(4);

// MPU6050 setup
Adafruit_MPU6050 mpu;

// PID variables
double setpoint, z_rot_input, rot_output = 0;
double x_velocity = 0, y_velocity = 0, dir_input, dir_output;
double kp = 10.0, ki = 0.0, kd = 1.0;
PID rot_pid(kp, ki, kd);

double dirKp = 0.1, dirKi = 0.0, dirKd = 0.1;
PID dir_pid(dirKp, dirKi, dirKd);

unsigned long lastTime = 0;

void moveRobot(double direction, double speed, unsigned long duration);
void adjustMotorSpeeds(double direction, double rotationOutput, double maxSpeed);
void stopMotors();

void setup() {
    Serial.begin(115200);

    // Initialize motor shield
    if (!AFMS.begin()) {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        while (1);
    }

    // Initialize motors
    motorA->setSpeed(0);
    motorB->setSpeed(0);
    motorD->setSpeed(0);
    motorC->setSpeed(0);

    setpoint = 0; // Maintain no rotation
    lastTime = millis();
}

void loop() {
    // Example call to move the robot
    int speed = 150;
    int time = 4000;
    int angle = 90;
    Serial.print("Moving robot at angle ");
    Serial.print(angle);
    Serial.print(" degrees at speed ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(time);
    Serial.println(" milliseconds.");
    moveRobot(angle, speed, time);
    Serial.print("Done");
    while (true);
}

void readIMU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Use gyroscope data for PID control
    z_rot_input = g.gyro.z;

    unsigned long currentTime = millis();
    double dt = (currentTime - lastTime) / 1000.0; // Time difference in seconds
    lastTime = currentTime;

    // Integrate acceleration to get velocity
    x_velocity += a.acceleration.x * dt;
    y_velocity += a.acceleration.y * dt;

    Serial.print("X Acceleration: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y Acceleration: ");
    Serial.print(a.acceleration.y);

    Serial.print(". X Velocity: ");
    Serial.print(x_velocity);
    Serial.print(", Y Velocity: ");
    Serial.print(y_velocity);

    // Direction robot is actually moving in (90 degrees = left)
    dir_input = atan2(y_velocity, x_velocity) * 180 / PI;
}

void moveRobot(double direction, double speed, unsigned long duration) {
    unsigned long startTime = millis();
    double moveDirection = direction;

    double directionRad = direction * PI / 180.0;

    double initSpeedFL = speed * cos(directionRad + PI / 4);
    double initSpeedBL = speed * cos(directionRad - PI / 4);
    double initSpeedFR = speed * cos(directionRad - PI / 4);
    double initSpeedBR = speed * cos(directionRad + PI / 4);

    motorA->setSpeed(constrain(abs(initSpeedFL), 0, 255));
    motorB->setSpeed(constrain(abs(initSpeedBL), 0, 255));
    motorD->setSpeed(constrain(abs(initSpeedBR), 0, 255));
    motorC->setSpeed(constrain(abs(initSpeedFR), 0, 255));

    // Set motor directions based on speed signs
    motorA->run(initSpeedFL >= 0 ? FORWARD : BACKWARD);
    motorB->run(initSpeedBL >= 0 ? FORWARD : BACKWARD);
    motorD->run(initSpeedBR >= 0 ? FORWARD : BACKWARD);
    motorC->run(initSpeedFR >= 0 ? FORWARD : BACKWARD);

    delay(200); // Initial movement delay

    while (millis() - startTime < duration) {
        readIMU();

        Serial.print(". Rotation Input: ");
        Serial.print(z_rot_input);

        rot_output = rot_pid.compute(setpoint, z_rot_input);

        Serial.print(". Rotation Output: ");
        Serial.print(rot_output);

        double direction_error = direction - dir_input;
        dir_output = dir_pid.compute(0, direction_error); // Target is 0 error

        moveDirection = direction - dir_output;

        Serial.print(". Direction Input: ");
        Serial.print(dir_input);

        Serial.print(". Direction Output: ");
        Serial.print(dir_output);

        Serial.print(". Move Direction: ");
        Serial.print(moveDirection);

        // Adjust motor speeds based on PID output
        adjustMotorSpeeds(moveDirection, rot_output, speed);

        delay(10); // PID loop delay
    }

    stopMotors();
}

void adjustMotorSpeeds(double direction, double rotationOutput, double maxSpeed) {
    // Convert direction from degrees to radians for calculation
    double directionRad = direction * PI / 180.0;

    // Calculate force components for each motor
    double forceFL = maxSpeed * cos(directionRad + PI / 4) - rotationOutput;
    double forceBL = maxSpeed * cos(directionRad - PI / 4) - rotationOutput;
    double forceFR = maxSpeed * cos(directionRad - PI / 4) + rotationOutput;
    double forceBR = maxSpeed * cos(directionRad + PI / 4) + rotationOutput;

    Serial.print(". Force FL: ");
    Serial.print(forceFL);
    Serial.print(", Force BL: ");
    Serial.print(forceBL);
    Serial.print(", Force FR: ");
    Serial.print(forceFR);
    Serial.print(", Force BR: ");
    Serial.println(forceBR);

    // Constrain speeds to the range [0, 255]
    motorA->setSpeed(constrain(abs(forceFL), 0, 255));
    motorB->setSpeed(constrain(abs(forceBL), 0, 255));
    motorD->setSpeed(constrain(abs(forceBR), 0, 255));
    motorC->setSpeed(constrain(abs(forceFR), 0, 255));

    // Set motor directions based on speed signs
    motorA->run(forceFL >= 0 ? FORWARD : BACKWARD);
    motorB->run(forceBL >= 0 ? FORWARD : BACKWARD);
    motorD->run(forceBR >= 0 ? FORWARD : BACKWARD);
    motorC->run(forceFR >= 0 ? FORWARD : BACKWARD);
}

void stopMotors() {
    motorA->setSpeed(0);
    motorB->setSpeed(0);
    motorD->setSpeed(0);
    motorC->setSpeed(0);

    motorA->run(RELEASE);
    motorB->run(RELEASE);
    motorD->run(RELEASE);
    motorC->run(RELEASE);
}
