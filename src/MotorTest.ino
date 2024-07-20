#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// #define FORWARD 1
// #define BACKWARD 2
// #define BRAKE 3
// #define RELEASE 4

Adafruit_DCMotor *motorA = AFMS.getMotor(1); // Front left motor
Adafruit_DCMotor *motorB = AFMS.getMotor(2); // Back left motor
Adafruit_DCMotor *motorC = AFMS.getMotor(4); // Front right motor
Adafruit_DCMotor *motorD = AFMS.getMotor(3); // Back right motor

void rotateDirection(int speed, bool rotateLeft, int targetAngle);
void moveDirection(int speed, float angle, int time);
void stopAllMotors();
float getCurrentAngle();
void updateGyroAngle();

float gyroAngleZ = 0;     // Variable to store the integrated gyroscope angle
unsigned long lastTime;   // Variable to store the last time the gyro was read
const float rotationAngleThreshold = 0.5; // Threshold to filter out small noise
const float moveAngleThreshold = 0.1;
void setup() {
  Serial.begin(115200);           // Set up Serial library at 115200 bps
  Serial.println("Motor test!");

  AFMS.begin();
  // Set motor speeds

  if (!mpu.begin()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  lastTime = millis();

  // Test movement
  moveDirection(150, 0, 1000);  // Move in direction 0 degrees at max speed 150 for 1 second
  delay(500);
  moveDirection(150, 90, 1000);  // Move in direction 90 degrees at max speed 150 for 1 second
  delay(500);
  moveDirection(150, 180, 1000);  // Move in direction 180 degrees at max speed 150 for 1 second
  delay(500);
  moveDirection(150, 270, 1000);  // Move in direction 270 degrees at max speed 150 for 1 second
  delay(500);

  // Test rotation
  rotateDirection(150, true, 90);  // Rotate left by 90 degrees
  delay(500);
  rotateDirection(150, false, 90);  // Rotate right by 90 degrees
  delay(500);
  rotateDirection(150, true, 180);  // Rotate left by 180 degrees
  delay(500);
  rotateDirection(150, false, 180);  // Rotate right by 180 degrees
  delay(500);

  Serial.println("Movement done");

}

void rotateDirection(int speed, bool rotateLeft, int targetAngle) {
  float startAngle = getCurrentAngle();
  float targetAngleAbs = fmod(startAngle + (rotateLeft ? targetAngle : -targetAngle) + 360, 360);

  Serial.print("Starting angle: ");
  Serial.println(startAngle);
  Serial.print("Target angle: ");
  Serial.println(targetAngleAbs);

  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);

  // Set motor directions for rotation
  if (rotateLeft) {
    motorA->run(2);
    motorB->run(2);
    motorC->run(1);
    motorD->run(1);
  } else {
    motorA->run(1);
    motorB->run(1);
    motorC->run(2);
    motorD->run(2);
  }

  delay(10); // Allow some time for motors to start
  
  // Rotate until reaching the target angle
  while (true) {
    updateGyroAngle();
    float currentAngle = getCurrentAngle();
    float angleDiff;

    if (rotateLeft) {
      angleDiff = fmod(currentAngle - startAngle + 360, 360);
    } else {
      angleDiff = fmod(startAngle - currentAngle + 360, 360);
    }

    // Apply the noise threshold
    if (fabs(angleDiff) <= rotationAngleThreshold) {
      angleDiff = 0;
    }

    Serial.print("Current angle: ");
    Serial.println(currentAngle);
    Serial.print("Angle diff: ");
    Serial.println(angleDiff);

    // Stop when the robot has rotated the target angle
    if (angleDiff >= targetAngle) {
      stopAllMotors();
      break;
    }

    delay(10);
  }
  stopAllMotors();
}

void moveDirection(int speed, float angle, int time) {
  // Calculate motor speeds based on desired direction
  float radAngle = angle * PI / 180.0;
  float cosA = cos(radAngle);
  float sinA = sin(radAngle);

  float vA = speed * (cosA - sinA);
  float vB = speed * (cosA + sinA);
  float vC = speed * (cosA + sinA);
  float vD = speed * (cosA - sinA);

  motorA->setSpeed(abs(vA));
  motorB->setSpeed(abs(vB));
  motorC->setSpeed(abs(vC));
  motorD->setSpeed(abs(vD));

  motorA->run(vA >= 0 ? 1 : 2);
  motorB->run(vB >= 0 ? 1 : 2);
  motorC->run(vC >= 0 ? 1 : 2);
  motorD->run(vD >= 0 ? 1 : 2);

  // Gyroscope correction loop
  unsigned long startTime = millis();

  while (millis() - startTime < time) { // Move for 1 second
    updateGyroAngle();
    float currentAngle = getCurrentAngle();
    float angleDiff = angle - currentAngle;
    
    Serial.print("Current angle: ");
    Serial.println(currentAngle);
    Serial.print("Angle Diff: ");
    Serial.println(angleDiff);

    if (fabs(angleDiff) > moveAngleThreshold) {
      // Adjust motor speeds based on angle difference
      float correction = angleDiff * 0.1; // Proportional gain
      motorA->setSpeed(constrain(abs(vA + correction), 0, 255));
      motorB->setSpeed(constrain(abs(vB + correction), 0, 255));
      motorC->setSpeed(constrain(abs(vC - correction), 0, 255));
      motorD->setSpeed(constrain(abs(vD - correction), 0, 255));

      motorA->run((vA + correction) >= 0 ? 1 : 2);
      motorB->run((vB + correction) >= 0 ? 1 : 2);
      motorC->run((vC - correction) >= 0 ? 1 : 2);
      motorD->run((vD - correction) >= 0 ? 1 : 2);
    }

    delay(10);
  }
  stopAllMotors();
}

void updateGyroAngle() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  gyroAngleZ += g.gyro.z * deltaTime * 180 / PI;
  Serial.print("Gyro Angle Z: ");
  Serial.println(gyroAngleZ);
}

float getCurrentAngle() {
  return fmod(gyroAngleZ + 360, 360);
}

void stopAllMotors() {
  motorA->setSpeed(0);
  motorB->setSpeed(0);
  motorC->setSpeed(0);
  motorD->setSpeed(0);
  motorA->run(4);
  motorB->run(4);
  motorC->run(4);
  motorD->run(4);
}

void loop(){
  // Empty loop as setup handles the test
  stopAllMotors();
}
