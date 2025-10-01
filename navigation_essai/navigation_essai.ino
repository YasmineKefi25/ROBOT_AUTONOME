#include <TimerOne.h>


//  Pin Definitions
#define IN1 3    // Motor Right control pin 1
#define IN2 2    // Motor Right control pin 2
#define IN3 4    // Motor Left control pin 1
#define IN4 5    // Motor Left control pin 2

#define interruptPinRA 18  // Encoder Right channel A
#define interruptPinRB 19  // Encoder Right channel B
#define interruptPinLA 20  // Encoder Left channel A
#define interruptPinLB 21  // Encoder Left channel B

//  Physical Robot Constants
#define tickcmR 58.6          // Ticks per cm (Right wheel)
#define tickcmL 58.9          // Ticks per cm (Left wheel)
#define wheel_radius 39.55    // Wheel radius in mm
#define entreaxe 305          // Distance between wheels in mm

//  Motion & Speed Limits
float PWM_MIN = 70;           
float PWM_MAX = 180;

float PWM_MIN_DOURA = 85;
float PWM_MAX_DOURA = 150;

//  PID Gains

// Right Wheel
float kp_R = 0.1;
float ki_R = 0.05;

// Left Wheel
float kp_L = 0.12;
float ki_L = 0.045;

// Orientation Correction for Straight Line
float kTheta = 2.0;

// Rotation Gains
float kp_dour = 0.001;       // Rotation proportional gain
float k_position = 0.5;      // Rotation wheel balancing

//  Global Variables
volatile long encoderRightCount = 0;
volatile long encoderLeftCount = 0;

long previousRightCount = 0;
long previousLeftCount = 0;

float totalR = 0; // total distance right wheel
float totalL = 0; // total distance left wheel

float theta = 0;  // robot orientation (rad)

// Speed measurement
float speedR = 0;
float speedL = 0;

// Errors
float right_erreur = 0;
float left_erreur = 0;
float i_right_erreur = 0;
float i_left_erreur = 0;

float orientation_erreur = 0;
float Theta_correction = 0;

float PWM_R = 0;
float PWM_L = 0;

//  Encoder Interrupts
void interruptR() {
  if (digitalRead(interruptPinRA) == digitalRead(interruptPinRB)) {
    encoderRightCount--;
  } else {
    encoderRightCount++;
  }
}

void interruptL() {
  if (digitalRead(interruptPinLA) == digitalRead(interruptPinLB)) {
    encoderLeftCount++;
  } else {
    encoderLeftCount--;
  }
}

//  Motor Control
void run() {
  // Right Motor
  if (PWM_R >= 0) {
    analogWrite(IN1, PWM_R);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, -PWM_R);
  }

  // Left Motor
  if (PWM_L >= 0) {
    analogWrite(IN3, PWM_L);
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, -PWM_L);
  }
}

void stopmotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

//  Odometry Update
void updateOdometrie() {
  long deltaRight = encoderRightCount - previousRightCount;
  long deltaLeft = encoderLeftCount - previousLeftCount;

  previousRightCount = encoderRightCount;
  previousLeftCount = encoderLeftCount;

  // Distance each wheel traveled (mm)
  float dsR = (deltaRight / tickcmR) * 10.0; // cm → mm
  float dsL = (deltaLeft / tickcmL) * 10.0;

  totalR += dsR;
  totalL += dsL;

  // Orientation change
  float dTheta = (dsR - dsL) / entreaxe;
  theta += dTheta;
}

//  Move Straight Function
void moveDistance(float distance, float speed) {
  // Reset integrals
  i_right_erreur = 0;
  i_left_erreur = 0;
  totalR = 0;
  totalL = 0;

  while (abs(distance - ((totalR + totalL) / 2.0)) > 1.0) {
    // Calculate errors
    right_erreur = distance - totalR;
    left_erreur = distance - totalL;

    i_right_erreur += right_erreur;
    i_left_erreur += left_erreur;

    // --- Wheel-Specific PI ---
    PWM_R = kp_R * right_erreur + ki_R * i_right_erreur;
    PWM_L = kp_L * left_erreur + ki_L * i_left_erreur;

    // --- Orientation Correction ---
    orientation_erreur = totalR - totalL;
    Theta_correction = kTheta * orientation_erreur;

    PWM_R -= Theta_correction;
    PWM_L += Theta_correction;

    // Limit PWM
    if (PWM_R > PWM_MAX) PWM_R = PWM_MAX;
    if (PWM_R < PWM_MIN) PWM_R = PWM_MIN;
    if (PWM_L > PWM_MAX) PWM_L = PWM_MAX;
    if (PWM_L < PWM_MIN) PWM_L = PWM_MIN;

    // Send to motors
    run();
  }

  stopmotors();
}


//  Rotate In Place Function
void dour(float angle, float speed, bool stopAfter = true) {
  // Distance each wheel must travel for given angle
  float distance = (angle * PI * entreaxe) / 180.0;

  totalR = 0;
  totalL = 0;

  while ((abs(totalR) + abs(totalL)) / 2.0 < distance) {
    float orientation_error = distance - ((abs(totalR) + abs(totalL)) / 2.0);

    // Base rotation control
    PWM_R = kp_dour * orientation_error;
    PWM_L = kp_dour * orientation_error;

    // Make right wheel go backward
    PWM_R = -PWM_R;

    // Position balancing between wheels
    float position_diff = abs(totalR) - abs(totalL);
    float position_correction = k_position * position_diff;

    PWM_R -= position_correction;
    PWM_L += position_correction;

    // Limit PWM
    if (PWM_R > PWM_MAX_DOURA) PWM_R = PWM_MAX_DOURA;
    if (PWM_R < PWM_MIN_DOURA) PWM_R = PWM_MIN_DOURA;
    if (PWM_L > PWM_MAX_DOURA) PWM_L = PWM_MAX_DOURA;
    if (PWM_L < PWM_MIN_DOURA) PWM_L = PWM_MIN_DOURA;

    run();
  }

  if (stopAfter) stopmotors();
}

// =========================
//  Setup & Loop
// =========================
void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(interruptPinRA, INPUT);
  pinMode(interruptPinRB, INPUT);
  pinMode(interruptPinLA, INPUT);
  pinMode(interruptPinLB, INPUT);

  attachInterrupt(digitalPinToInterrupt(interruptPinRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinLA), interruptL, CHANGE);

  Timer1.initialize(5000); // 5ms timer
  Timer1.attachInterrupt(updateOdometrie);

  Serial.println("Robot Ready");
}

void loop() {
  // Example trajectory
  moveDistance(1000, 2500);   // Move forward 1 meter
  dour(90, 1500, true);       // Rotate 90 degrees
  moveDistance(500, 2500);    // Move forward 50 cm
  stopmotors();

  while (1); // Stop here
}
