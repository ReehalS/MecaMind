#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Rotation_PID.h"
#include "Direction_PID.h"

// Motor shield setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorA = AFMS.getMotor(1);
Adafruit_DCMotor *motorB = AFMS.getMotor(2);
Adafruit_DCMotor *motorD = AFMS.getMotor(3);
Adafruit_DCMotor *motorC = AFMS.getMotor(4);

// MPU6050 setup
Adafruit_MPU6050 mpu;

// PID variables
double setpoint = 0; // Target rotation angle
double input = 0;    // Current rotation rate
double rotationOutput; // Output from rotation PID

// Rotation PID setup
double kpRotation = 1.0, kiRotation = 0.1, kdRotation = 0.1;
Rotation_PID RotationPID(kpRotation, kiRotation, kdRotation);

// Direction PID variables
double intendedDirection = 0; // Desired direction in degrees
double actualDirection = 0;   // Actual direction in degrees
double directionOutput = 0;   // Output from direction PID

// Direction PID setup
double kpDirection = 1, kiDirection = 0.01, kdDirection = 0.01;
Direction_PID DirectionPID(kpDirection, kiDirection, kdDirection);

void moveRobot(double direction, double speed, unsigned long duration);
void adjustMotorSpeeds(double direction, double rotationOutput, double maxSpeed, double directionOutput);
void stopMotors();
double calculateActualDirection(sensors_event_t &accel);

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
}

void loop() {
    // Example call to move the robot
    int speed = 175;
    int time = 4000;
    int angle = 90; // Example direction in degrees
    Serial.print("Moving robot at angle ");
    Serial.print(angle);
    Serial.print(" degrees at speed ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(time);
    Serial.println(" milliseconds.");
    moveRobot(angle, speed, time);
    Serial.println("Done");
    while (true); // Prevents loop from repeating
}

void moveRobot(double direction, double speed, unsigned long duration) {
    unsigned long startTime = millis();
    intendedDirection = direction; // Set the intended direction in degrees

    while (millis() - startTime < duration) {
        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);

        // Use gyroscope data for rotation PID control
        input = gyro.gyro.z;
        rotationOutput = RotationPID.compute(setpoint, input);

        // Calculate actual direction from accelerometer data
        actualDirection = calculateActualDirection(accel);
        double directionError = intendedDirection - actualDirection;
        directionOutput = DirectionPID.compute(intendedDirection, actualDirection);

        // Debugging output
        Serial.print("Input: ");
        Serial.print(input);
        Serial.print(". Rotation Output: ");
        Serial.print(rotationOutput);
        Serial.print(". Actual Direction: ");
        Serial.print(actualDirection);
        Serial.print(". Direction Error: ");
        Serial.print(directionError);
        Serial.print(". Direction Output: ");
        Serial.print(directionOutput);

        // Adjust motor speeds
        adjustMotorSpeeds(direction, rotationOutput, speed, directionOutput);

        delay(10); // PID loop delay
    }
    stopMotors();
}

double calculateActualDirection(sensors_event_t &accel) {
    // Calculate the actual movement direction from accelerometer data
    double x = accel.acceleration.x;
    double y = accel.acceleration.y;
    double angle = atan2(y, x) * 180.0 / PI;
    return angle;
}

void adjustMotorSpeeds(double direction, double rotationOutput, double maxSpeed, double directionOutput) {
    // Convert direction from degrees to radians for calculation
    double directionRad = direction * PI / 180.0;

    // Calculate force components for each motor
    double forceFL = maxSpeed * cos(directionRad + PI/4) - rotationOutput - directionOutput;
    double forceBL = maxSpeed * cos(directionRad - PI/4) - rotationOutput - directionOutput;
    double forceFR = maxSpeed * cos(directionRad - PI/4) + rotationOutput + directionOutput;
    double forceBR = maxSpeed * cos(directionRad + PI/4) + rotationOutput + directionOutput;

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
