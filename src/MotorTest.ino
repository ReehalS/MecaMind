#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define FORWARD 1
#define BACKWARD 2
#define RELEASE 0

Adafruit_MPU6050 mpu;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motorA = AFMS.getMotor(1); // Front left motor
Adafruit_DCMotor *motorB = AFMS.getMotor(2); // Back left motor
Adafruit_DCMotor *motorC = AFMS.getMotor(4); // Front right motor
Adafruit_DCMotor *motorD = AFMS.getMotor(3); // Back right motor

void rotateDirection(int speed, bool rotateLeft, int targetAngle);
void moveDirection(int speed, float angle);
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

  // Test rotation
  //rotateDirection(150, true, 90);  // Rotate left by 90 degrees
  delay(100);
  // for(int i=0; i<=360; i+=15){
  //   moveDirection(175, i, 250);
  // }

  moveDirection(175, 90, 1000);  // Move in direction 45 degrees at max speed 150 for 1 second
  stopAllMotors();
  delay(1000);
  moveDirection(175, 270, 2000);
  stopAllMotors();
  delay(1000);
  //stopAllMotors();
  
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
    motorA->run(BACKWARD);
    motorB->run(BACKWARD);
    motorC->run(FORWARD);
    motorD->run(FORWARD);
  } else {
    motorA->run(FORWARD);
    motorB->run(FORWARD);
    motorC->run(BACKWARD);
    motorD->run(BACKWARD);
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

  motorA->run(vA >= 0 ? FORWARD : BACKWARD);
  motorB->run(vB >= 0 ? FORWARD : BACKWARD);
  motorC->run(vC >= 0 ? FORWARD : BACKWARD);
  motorD->run(vD >= 0 ? FORWARD : BACKWARD);

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

      motorA->run((vA + correction) >= 0 ? FORWARD : BACKWARD);
      motorB->run((vB + correction) >= 0 ? FORWARD : BACKWARD);
      motorC->run((vC - correction) >= 0 ? FORWARD : BACKWARD);
      motorD->run((vD - correction) >= 0 ? FORWARD : BACKWARD);
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
  motorA->run(RELEASE);
  motorB->run(RELEASE);
  motorC->run(RELEASE);
  motorD->run(RELEASE);
}

void loop(){
  // Empty loop as setup handles the test
  stopAllMotors();
}
