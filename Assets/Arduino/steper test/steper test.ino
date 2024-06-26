#include <AccelStepper.h>
#include <math.h>

// Define motor interface type
#define motorInterfaceType 4

// Define the pins for the motors
#define MOTOR1_STEP_PIN 8
#define MOTOR1_DIR_PIN 9
#define MOTOR2_STEP_PIN 4
#define MOTOR2_DIR_PIN 5
#define MOTOR3_STEP_PIN 12
#define MOTOR3_DIR_PIN 13

// Create motor objects
AccelStepper motor1(motorInterfaceType, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(motorInterfaceType, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper motor3(motorInterfaceType, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);

// Steps per revolution for the stepper motors
const int stepsPerRevolution = 200;  // This value depends on your stepper motor

// Robot arm parameters
const float L0_Angle = 65.0;
const float L0 = 7.5;
const float L1 = 7.5;
const float L2 = 7.5;
float xi, yi, L;

// Predefined target coordinates
const float targetX = 5.0;
const float targetY = 5.0;
const float targetZ = 5.0;

void setup() {
  // Set enable pins as output and HIGH
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(3, HIGH);

  // Initialize the motors
  motor1.setMaxSpeed(1000);
  motor1.setAcceleration(500);
  
  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(500);
  
  motor3.setMaxSpeed(1000);
  motor3.setAcceleration(500);

  // Start serial communication
  Serial.begin(9600);

  // Calculate initial xi and yi based on L0 and L0_Angle
  xi = L0 * cos(L0_Angle * DEG_TO_RAD);
  yi = L0 * sin(L0_Angle * DEG_TO_RAD);

  // Move to the predefined target position
  goToNextPos(targetX, targetY, targetZ);
}

void loop() {
  // Add any additional logic here if needed
}

// Function to calculate and move to the specified position
void goToNextPos(float x, float y, float z) {
  L = sqrt(y * y + x * x);

  if (L > L1 + L2) {
    Serial.println("Out of ARM Scope");
    return;
  }

  float k1 = (-L * L + L1 * L1 + L2 * L2) / (2 * L1 * L2);
  float k2 = (L * L + L1 * L1 - L2 * L2) / (2 * L1 * L);

  float gamma = atan2(x, z) * RAD_TO_DEG;
  xi = L0 * cos((L0_Angle + gamma) * DEG_TO_RAD);
  yi = L0 * sin((L0_Angle + gamma) * DEG_TO_RAD);

  double alpha_2 = acos(k1) * RAD_TO_DEG;
  double beta_1 = acos(k2) * RAD_TO_DEG;
  double beta_2 = atan2(y, x) * RAD_TO_DEG;
  double alpha_1 = beta_1 + beta_2;

  Serial.println("Angles: " + String(alpha_1) + ", " + String(alpha_2) + ", " + String(gamma));

  // Move the motors to the calculated angles
  moveToAngles(alpha_1, alpha_2 - 180, gamma - 90);
}

// Function to convert degrees to steps
long degreesToSteps(float degrees) {
  return (long)(degrees * stepsPerRevolution / 360.0);
}

// Function to move motors to specified angles
void moveToAngles(float angle1, float angle2, float angle3) {
  long target1 = degreesToSteps(angle1);
  long target2 = degreesToSteps(angle2);
  long target3 = degreesToSteps(angle3);
  
  motor1.moveTo(target1);
  motor2.moveTo(target2);
  motor3.moveTo(target3);

  // Wait for the motors to reach their positions
  while (motor1.isRunning() || motor2.isRunning() || motor3.isRunning()) {
    motor1.run();
    motor2.run();
    motor3.run();
  }
}
