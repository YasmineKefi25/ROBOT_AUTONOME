#include <TimerOne.h>

// MOTOR PINS
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

// ENCODER PINS
#define interruptPinRA 19
#define interruptPinRB 18
#define interruptPinLA 3
#define interruptPinLB 2

#define PI 3.14159265358979323846

// ROBOT SPECS
#define nb_ticks 800
#define wheel_radius 4.0   // in cm
#define entreaxe 33.5      // distance between wheels in cm

// ODOMETRY VARIABLES
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
float currentvelocityRight = 0;
float currentvelocityLeft = 0;
float theta = 0.0;
float dS_total = 0.0;
float totalL = 0.0;
float totalR = 0.0;

long lastEncoderLeftCount = 0;
long lastEncoderRightCount = 0;

float dsL = 0;
float dsR = 0;

int speed_ech = 10;
float total_ech_l = 0;
float total_ech_r = 0;

// INTERRUPTS
void interruptR() {
  if (digitalRead(interruptPinRA) == digitalRead(interruptPinRB)) encoderRightCount--;
  else encoderRightCount++;
}

void interruptL() {
  if (digitalRead(interruptPinLA) == digitalRead(interruptPinLB)) encoderLeftCount++;
  else encoderLeftCount--;
}

// DISTANCE CALCULATION
float calculDistance(long deltaLeftCount, long deltaRightCount) {
  dsL = (deltaLeftCount / (float)nb_ticks) * 2.0 * PI * wheel_radius;
  dsR = (deltaRightCount / (float)nb_ticks) * 2.0 * PI * wheel_radius;
  return (dsL + dsR) / 2.0;
}

// SPEED CALCULATION
void speed_calcul() {
  float dt = speed_ech * 0.015; // seconds
  float right_encoder_speed = total_ech_r / dt;
  float left_encoder_speed  = total_ech_l / dt;
  float alpha_speed = (total_ech_r - total_ech_l) / dt;

  currentvelocityRight = (right_encoder_speed + left_encoder_speed) / 2 + alpha_speed * wheel_radius / 2;
  currentvelocityLeft  = (right_encoder_speed + left_encoder_speed) / 2 - alpha_speed * wheel_radius / 2;

  total_ech_l = total_ech_r = 0;
}

// ODOMETRY UPDATE
void updateOdometrie() {
  long deltaLeftCount = encoderLeftCount - lastEncoderLeftCount;
  long deltaRightCount = encoderRightCount - lastEncoderRightCount;

  lastEncoderLeftCount  = encoderLeftCount;
  lastEncoderRightCount = encoderRightCount;

  float dS = calculDistance(deltaLeftCount, deltaRightCount);

  totalL += dsL;
  totalR += dsR;
  dS_total += dS;

  total_ech_l += dsL;
  total_ech_r += dsR;

  theta += (dsR - dsL) / entreaxe;

  static int t = 0;
  t++;
  if (t % speed_ech == 0) speed_calcul();

  dsL = dsR = 0;
}

// MOTOR CONTROL
#define PWM_MAX 200
#define PWM_MIN 100

int PWM_L, PWM_R;

void run() {
  if (PWM_R >= 0) { analogWrite(IN1, PWM_R); analogWrite(IN2, 0); }
  else { analogWrite(IN1, 0); analogWrite(IN2, -PWM_R); }

  if (PWM_L >= 0) { analogWrite(IN3, PWM_L); analogWrite(IN4, 0); }
  else { analogWrite(IN3, 0); analogWrite(IN4, -PWM_L); }
}

void stopmotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// SIMPLE PID
float compute(float setpoint, float current, float &integral, float kp, float ki) {
  float dt = 0.05;
  float error = setpoint - current;
  float integralCandidate = integral + error * dt;
  float outputCandidate = kp * error + ki * integralCandidate;

  float output;
  if (abs(outputCandidate) > PWM_MAX) {
    output = PWM_MAX * ((outputCandidate > 0) ? 1 : -1);
  } else {
    integral = integralCandidate;
    output = outputCandidate;
  }
  return output;
}

// RESET VARIABLES
void resetControllers() {
  totalR = totalL = dS_total = theta = 0;
  total_ech_l = total_ech_r = 0;
}

// TURN FUNCTION
void dour(float targetAngleDeg, float speed) {
  resetControllers();
  float targetRad = targetAngleDeg * PI / 180.0;
  float targetDistance = (entreaxe * targetRad) / 2.0; // each wheel travels half of arc length

  while (abs(theta) < abs(targetRad)) {
    int sens = (targetRad > 0) ? 1 : -1;

    // simple open-loop speed for rotation
    PWM_R = speed * sens;
    PWM_L = -speed * sens;

    PWM_R = constrain(PWM_R, -PWM_MAX, PWM_MAX);
    PWM_L = constrain(PWM_L, -PWM_MAX, PWM_MAX);

    run();
    delay(10);
  }
  stopmotors();
}

// SETUP
void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(interruptPinRA, INPUT_PULLUP);
  pinMode(interruptPinRB, INPUT_PULLUP);
  pinMode(interruptPinLA, INPUT_PULLUP);
  pinMode(interruptPinLB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(interruptPinRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinLA), interruptL, CHANGE);

  Timer1.initialize(100000); // 100ms
  Timer1.attachInterrupt(updateOdometrie);

  // Rotate 180 degrees
  dour(180, 150);
}

void loop() {
  // You can call moveDistance() or other functions here later
}
