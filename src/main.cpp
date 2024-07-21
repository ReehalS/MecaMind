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
double setpoint, input, output;
double kp = 1.0, ki = 0.1, kd = 0.1;
PID pid(kp, ki, kd);

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
}

void loop() {
    // Example call to move the robot
    int speed = 150;
    int time = 4000;
    int angle = 270;
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
    input = g.gyro.z;
}

void moveRobot(double direction, double speed, unsigned long duration) {
    unsigned long startTime = millis();

    while (millis() - startTime < duration) {
        readIMU();
        
        Serial.print("Input: ");
        Serial.print(input);

        output = pid.compute(setpoint, input);

        Serial.print(". Output: ");
        Serial.print(output);

        // Adjust motor speeds based on PID output
        adjustMotorSpeeds(direction, output, speed);

        delay(10); // PID loop delay
    }
    
    stopMotors();
}

void adjustMotorSpeeds(double direction, double rotationOutput, double maxSpeed) {
    // Convert direction from degrees to radians for calculation
    double directionRad = direction * PI / 180.0;

    // Calculate force components for each motor
    double forceFL = maxSpeed * cos(directionRad + PI/4) - rotationOutput;
    double forceBL = maxSpeed * cos(directionRad - PI/4) - rotationOutput;
    double forceFR = maxSpeed * cos(directionRad - PI/4) + rotationOutput;
    double forceBR = maxSpeed * cos(directionRad + PI/4) + rotationOutput;

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
