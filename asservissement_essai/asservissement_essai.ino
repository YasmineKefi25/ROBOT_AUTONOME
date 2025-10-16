#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// =========================
// ===== ROBOT PINS =======
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

#define ENC_L_A 3
#define ENC_L_B 2
#define ENC_R_A 19
#define ENC_R_B 18

// =========================
// ===== ROBOT GEOMETRY =====
const float WHEEL_RADIUS = 4;   // cm
const float WHEEL_BASE   = 33.5; // cm
const int TICKS_PER_REV  = 800;  // encoder ticks per wheel rev
const float DT           = 0.05; // PID update period (s)

// =========================
// ===== MOTION PROFILE =====
float vmax = 20.0;
float accel = 5.0;
float d_acc = 0.0;
float d_dec = 0.0;
float d_const = 0.0;
float vpeak = 0.0;
int triangular = 0;

// =========================
// ===== ENCODERS =====
volatile long leftTicks = 0;
volatile long rightTicks = 0;
long prevLeftTicks = 0;
long prevRightTicks = 0;

// =========================
// ===== ODOMETRY =====
float distanceLeft = 0.0;
float distanceRight = 0.0;
float linearDistance = 0.0;
float theta = 0.0;
float speedLeft = 0.0;
float speedRight = 0.0;
float linearSpeed = 0.0;
float angularSpeed = 0.0;

// =========================
// ===== PID =====
float Kp_L = 1.0, Ki_L = 0.1;
float Kp_R = 1.0, Ki_R = 0.1;
float integral_L = 0.0, integral_R = 0.0;
float minPWM = 0.0, maxPWM = 255.0;

// =========================
// ===== TARGET SPEEDS =====
float targetSpeedL = 0.0;
float targetSpeedR = 0.0;

// =========================
// ===== SERIAL OPTIMIZATION =====
unsigned long serialCounter = 0;
const int serialSkip = 4; // print every 4 loops (~200ms)

// =========================
// ===== MOTION CONTROL =====
bool moving = false;
float moveTargetDistance = 0.0;
float moveDirection = 1.0;
float rotateTargetDistance = 0.0;
float rotateDirection = 1.0;
bool rotating = false;

// =========================
// ===== FUNCTION DECLARATIONS =====
void updateOdometry();
void calculateDistance();
void calculateSpeed();
void prepareMotionProfile(float dist);
float calculateCommandedSpeed(float traveledDist);
float computePI(float setpoint, float current, float Kp, float Ki, float &integral);
void runMotors(float pwmL, float pwmR);
void stopMotors();
float clamp(float val, float minVal, float maxVal);
void resetControllers();
void startMove(float distance, float speed);
void startRotate(float angle, float speed);
void processMotion();

// =========================
// ===== ENCODER INTERRUPTS =====
void ISR_leftA()  { bool A = digitalRead(ENC_L_A); bool B = digitalRead(ENC_L_B); leftTicks += (A==B)?1:-1; }
void ISR_leftB()  { bool A = digitalRead(ENC_L_A); bool B = digitalRead(ENC_L_B); leftTicks += (A!=B)?1:-1; }
void ISR_rightA() { bool A = digitalRead(ENC_R_A); bool B = digitalRead(ENC_R_B); rightTicks += (A==B)?1:-1; }
void ISR_rightB() { bool A = digitalRead(ENC_R_A); bool B = digitalRead(ENC_R_B); rightTicks += (A!=B)?1:-1; }

// =========================
// ===== TIMER ISR (Odometry) =====
void ISR_Timer() {
    updateOdometry();
}

// =========================
// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), ISR_leftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), ISR_leftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), ISR_rightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), ISR_rightB, CHANGE);

  resetControllers();

  // Setup TimerOne for precise 50ms updates
  Timer1.initialize(50000); // 50 ms = 50000 us
  Timer1.attachInterrupt(ISR_Timer);

  // Example: Start a move
  startMove(50.0, 15.0);
  // Example: Rotate 90 degrees
  // startRotate(90.0, 10.0);
}

// =========================
// ===== LOOP =====
void loop() {
    // Process motion non-blocking
    processMotion();
}

// =========================
// ===== FUNCTIONS =====
void resetControllers() {
  integral_L = 0.0; integral_R = 0.0;
  prevLeftTicks = leftTicks; prevRightTicks = rightTicks;
  distanceLeft = 0.0; distanceRight = 0.0;
  linearDistance = 0.0; theta = 0.0;
  speedLeft = 0.0; speedRight = 0.0;
}

void calculateDistance() {
  long dL = leftTicks - prevLeftTicks;
  long dR = rightTicks - prevRightTicks;
  prevLeftTicks = leftTicks; prevRightTicks = rightTicks;

  distanceLeft  = 2.0*PI*WHEEL_RADIUS*dL/(TICKS_PER_REV*4.0);
  distanceRight = 2.0*PI*WHEEL_RADIUS*dR/(TICKS_PER_REV*4.0);
}

void calculateSpeed() {
  speedLeft  = distanceLeft / DT;
  speedRight = distanceRight / DT;
  linearSpeed  = (speedLeft + speedRight)/2.0;
  angularSpeed = (speedRight - speedLeft)/WHEEL_BASE;
}

void updateOdometry() {
  calculateDistance();
  linearDistance += (distanceLeft + distanceRight)/2.0;
  theta += (distanceRight - distanceLeft)/WHEEL_BASE;
  calculateSpeed();
}

void prepareMotionProfile(float dist) {
  float accDist = (vmax*vmax)/(2.0*accel);
  if (2.0*accDist > dist) {
    triangular = 1;
    vpeak = sqrt(accel*dist);
    d_acc = dist/2.0; d_dec = dist/2.0; d_const = 0.0;
  } else {
    triangular = 0;
    vpeak = vmax;
    d_acc = accDist; d_dec = accDist; d_const = dist - 2.0*accDist;
  }
}

float calculateCommandedSpeed(float traveledDist) {
  if (traveledDist < d_acc) return sqrt(2.0*accel*traveledDist);
  if (traveledDist < d_acc + d_const) return vpeak;
  if (traveledDist < d_acc + d_const + d_dec) return sqrt(vpeak*vpeak - 2.0*accel*(traveledDist-d_acc-d_const));
  return 0.0;
}

float computePI(float setpoint, float current, float Kp, float Ki, float &integral) {
  float error = setpoint - current;
  float integralCandidate = integral + error*DT;
  float output;
  float outputCandidate = Kp*error + Ki*integralCandidate;
  if (outputCandidate > maxPWM) { output = maxPWM; if (error<0) integral=integralCandidate; }
  else if (outputCandidate < minPWM) { output = minPWM; if (error>0) integral=integralCandidate; }
  else { integral = integralCandidate; output = outputCandidate; }
  return output;
}

float clamp(float val, float minVal, float maxVal) {
  if (val>maxVal) return maxVal;
  if (val<minVal) return minVal;
  return val;
}

void runMotors(float pwmL, float pwmR) {
  pwmL = clamp(pwmL, -255, 255);
  pwmR = clamp(pwmR, -255, 255);

  if(pwmL >= 0) { analogWrite(IN1, pwmL); analogWrite(IN2, 0); }
  else { analogWrite(IN1, 0); analogWrite(IN2, -pwmL); }

  if(pwmR >= 0) { analogWrite(IN3, pwmR); analogWrite(IN4, 0); }
  else { analogWrite(IN3, 0); analogWrite(IN4, -pwmR); }
}

void stopMotors() {
  analogWrite(IN1, 0); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, 0);
}

// =========================
// ===== MOTION CONTROL =====
void startMove(float distance, float speed){
  resetControllers();
  vmax = speed;
  prepareMotionProfile(distance);
  moveTargetDistance = distance;
  moveDirection = 1.0;
  moving = true;
}

void startRotate(float angle, float speed){
  resetControllers();
  float radDistance = PI*WHEEL_BASE*abs(angle)/180.0;
  prepareMotionProfile(radDistance);
  rotateTargetDistance = radDistance;
  rotateDirection = (angle>0)?1.0:-1.0;
  rotating = true;
}

void processMotion(){
  if(moving){
    float traveled = linearDistance;
    float targetSpeed = calculateCommandedSpeed(traveled);
    targetSpeedL = targetSpeed; targetSpeedR = targetSpeed;

    float pwmL = computePI(targetSpeedL, speedLeft, Kp_L, Ki_L, integral_L);
    float pwmR = computePI(targetSpeedR, speedRight, Kp_R, Ki_R, integral_R);

    runMotors(pwmL,pwmR);

    serialCounter++;
    if(serialCounter>=serialSkip){
      serialCounter=0;
      Serial.print(traveled); Serial.print(",");
      Serial.print(targetSpeedL); Serial.print(",");
      Serial.print(speedLeft); Serial.print(",");
      Serial.println(speedRight);
    }

    if(traveled >= moveTargetDistance){
      stopMotors();
      moving = false;
    }
  }

  if(rotating){
    float traveled = abs((distanceRight - distanceLeft)/2.0);
    float targetSpeed = calculateCommandedSpeed(traveled);
    targetSpeedL = rotateDirection*targetSpeed;
    targetSpeedR = -rotateDirection*targetSpeed;

    float pwmL = computePI(targetSpeedL, speedLeft, Kp_L, Ki_L, integral_L);
    float pwmR = computePI(targetSpeedR, speedRight, Kp_R, Ki_R, integral_R);

    runMotors(pwmL,pwmR);

    serialCounter++;
    if(serialCounter>=serialSkip){
      serialCounter=0;
      Serial.print(traveled); Serial.print(",");
      Serial.print(targetSpeedL); Serial.print(",");
      Serial.print(speedLeft); Serial.print(",");
      Serial.println(speedRight);
    }

    if(traveled >= rotateTargetDistance){
      stopMotors();
      rotating = false;
    }
  }
}
