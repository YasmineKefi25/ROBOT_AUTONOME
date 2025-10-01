#include <TimerOne.h>

// Motor pins
#define IN1 3
#define IN2 2
#define IN3 4
#define IN4 5

// Encoder pins
#define interruptPinRA 18
#define interruptPinRB 19
#define interruptPinLA 20
#define interruptPinLB 21

// Robot parameters
#define wheel_radius 39.55
#define entreaxe 305
#define nb_ticks 800
#define PI 3.14159265358979323846

// PID and control
float kp_left = 0.1,
float ki_left = 0.05;
float kp_right = 0.12,
float ki_right = 0.06;
float kTheta = 2;
float k_position = 0.5;

float PWM_MAX = 180;
float PWM_MIN = 70;

// Odometry
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

float dsL = 0, dsR = 0, dS_total = 0;
float totalL = 0, totalR = 0;
float theta = 0;

// Speed
float currentvelocityLeft = 0, currentvelocityRight = 0;
float total_ech_l = 0, total_ech_r = 0;
int speed_ech = 10;

// PID integrals
float i_left_erreur = 0, i_right_erreur = 0;

// PWM commands
float PWM_L = 0, PWM_R = 0;

// ======= INTERRUPTS =======
void interruptL() {
  if (digitalRead(interruptPinLA) == digitalRead(interruptPinLB)) encoderLeftCount++;
  else encoderLeftCount--;
}

void interruptR() {
  if (digitalRead(interruptPinRA) == digitalRead(interruptPinRB)) encoderRightCount--;
  else encoderRightCount++;
}

// ======= DISTANCE CALC =======
float calculDistance(long deltaLeftCount, long deltaRightCount, float wheel_radius, int nb_ticks) {
  dsL = (deltaLeftCount / (float)nb_ticks) * 2 * PI * wheel_radius;
  dsR = (deltaRightCount / (float)nb_ticks) * 2 * PI * wheel_radius;
  return (dsL + dsR) / 2.0;
}

// ======= SPEED CALC =======
void speed_calcul() {
  currentvelocityRight = total_ech_r / (speed_ech * 0.015);
  currentvelocityLeft = total_ech_l / (speed_ech * 0.015);
  total_ech_r = 0;
  total_ech_l = 0;
}

// ======= ODOMETRY =======
long lastEncoderLeftCount = 0, lastEncoderRightCount = 0;
void updateOdometrie() {
  long deltaLeft = encoderLeftCount - lastEncoderLeftCount;
  long deltaRight = encoderRightCount - lastEncoderRightCount;

  lastEncoderLeftCount = encoderLeftCount;
  lastEncoderRightCount = encoderRightCount;

  float dS = calculDistance(deltaLeft, deltaRight, wheel_radius, nb_ticks);
  dS_total += dS;
  totalL += dsL;
  totalR += dsR;
  total_ech_l += dsL;
  total_ech_r += dsR;

  float dTheta = (dsR - dsL) / entreaxe;
  theta += dTheta;

  static int t = 0;
  t++;
  if (t % speed_ech == 0) speed_calcul();
}

// ======= MOTOR STOP =======
void stopmotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0); 
  analogWrite(IN4, 0);
}

// ======= MOVE DISTANCE =======
void moveDistance(float distance, float speed) {
  totalL = totalR = dS_total = i_left_erreur = i_right_erreur = 0;
  float accel = 0.25 * distance;
  float decel = 0.5 * distance;
  int sens = 1;

  while (abs(dS_total - distance) > 5) {
    sens = (dS_total < distance) ? 1 : -1;

    // Ramp speed
    float current_speed = sens * speed;
    if (abs(dS_total) < accel) current_speed = sens * (speed / accel) * abs(dS_total);
    else if (distance - abs(dS_total) < decel)
      current_speed = sens * ((speed / -decel) * abs(dS_total) + speed - ((distance - decel) * (speed / -decel)));

    // Right motor PID
    float errorR = current_speed - currentvelocityRight;
    i_right_erreur += errorR;
    PWM_R = kp_right * errorR + ki_right * i_right_erreur;

    // Anti-windup
    if (PWM_R > PWM_MAX) { PWM_R = PWM_MAX; i_right_erreur -= errorR; }
    else if (PWM_R < -PWM_MAX) { PWM_R = -PWM_MAX; i_right_erreur -= errorR; }

    // Left motor PID
    float errorL = current_speed - currentvelocityLeft;
    i_left_erreur += errorL;
    PWM_L = kp_left * errorL + ki_left * i_left_erreur;

    if (PWM_L > PWM_MAX) { PWM_L = PWM_MAX; i_left_erreur -= errorL; }
    else if (PWM_L < -PWM_MAX) { PWM_L = -PWM_MAX; i_left_erreur -= errorL; }

    // Orientation correction
    float theta_err = totalR - totalL;
    float theta_correction = kTheta * theta_err;
    PWM_R -= theta_correction; PWM_L += theta_correction;

    // Position correction
    float pos_err = k_position * (totalR + totalL);
    PWM_R += pos_err; PWM_L -= pos_err;

    // Clamp PWM after all corrections
    PWM_R = constrain(PWM_R, -PWM_MAX, PWM_MAX);
    PWM_L = constrain(PWM_L, -PWM_MAX, PWM_MAX);

    // Run motors
    if (PWM_R >= 0) { 
      analogWrite(IN1, PWM_R);
      analogWrite(IN2, 0); }
    else { 
      analogWrite(IN1, 0);
      analogWrite(IN2, -PWM_R); }

    if (PWM_L >= 0) { 
      analogWrite(IN3, PWM_L); 
      analogWrite(IN4, 0); }
    else { 
      analogWrite(IN3, 0); 
      analogWrite(IN4, -PWM_L); }

    // Debug
    Serial.print("dS_total: "); Serial.print(dS_total);
    Serial.print("\tPWM_L: "); Serial.print(PWM_L);
    Serial.print("\tPWM_R: "); Serial.println(PWM_R);
  }

  stopmotors();
}

// ======= ROTATION =======
void dour(float angleDeg, float vmax, float accel, bool stopAtEnd) {
    // Reset all counters and integrals
    totalL = totalR = dS_total = i_left_erreur = i_right_erreur = 0;

    float dist = angleDeg * PI * entreaxe / 180.0;  // convert angle to distance
    float d_acc, d_dec, d_const, vpeak;
    int triangular;

    // --- Motion Profile Calculation ---
    float acc_dist = (vmax * vmax) / (2.0 * accel);
    if (2.0 * acc_dist > dist) { // Triangular profile
        triangular = 1;
        vpeak = sqrt(accel * dist);
        d_acc = dist / 2.0;
        d_dec = dist / 2.0;
        d_const = 0.0;
    } else { // Trapezoidal profile
        triangular = 0;
        vpeak = vmax;
        d_acc = acc_dist;
        d_dec = acc_dist;
        d_const = dist - 2.0 * acc_dist;
    }

    int sens = 1;

    while (abs((theta * 180.0 / PI) - angleDeg) > 2) {
        sens = ((totalR - totalL) < dist) ? 1 : -1;

        // --- Determine current speed based on motion profile ---
        float traveled = abs(totalR - totalL);
        float current_speed;
        if (traveled < d_acc)
            current_speed = sens * sqrt(2.0 * accel * traveled);          // accelerating
        else if (traveled < d_acc + d_const)
            current_speed = sens * vpeak;                                  // constant speed
        else
            current_speed = sens * sqrt(2.0 * accel * (dist - traveled));  // decelerating

        // --- Right PID ---
        float errorR = current_speed - currentvelocityRight;
        i_right_erreur += errorR;
        PWM_R = kp_right * errorR + ki_right * i_right_erreur;

        // Anti-windup
        if (PWM_R > PWM_MAX) { PWM_R = PWM_MAX; i_right_erreur -= errorR; }
        else if (PWM_R < -PWM_MAX) { PWM_R = -PWM_MAX; i_right_erreur -= errorR; }

        // --- Left PID ---
        float errorL = -current_speed - currentvelocityLeft;
        i_left_erreur += errorL;
        PWM_L = kp_left * errorL + ki_left * i_left_erreur;

        if (PWM_L > PWM_MAX) { PWM_L = PWM_MAX; i_left_erreur -= errorL; }
        else if (PWM_L < -PWM_MAX) { PWM_L = -PWM_MAX; i_left_erreur -= errorL; }

        // --- Position & orientation correction ---
        float pos_err = k_position * (totalR + totalL);
        float theta_err = totalR - totalL;
        float theta_correction = kTheta * theta_err;

        PWM_R = constrain(PWM_R + pos_err - theta_correction, -PWM_MAX, PWM_MAX);
        PWM_L = constrain(PWM_L - pos_err + theta_correction, -PWM_MAX, PWM_MAX);

        // --- Run motors ---
        if (PWM_R >= 0) { analogWrite(IN1, PWM_R); analogWrite(IN2, 0); }
        else { analogWrite(IN1, 0); analogWrite(IN2, -PWM_R); }

        if (PWM_L >= 0) { analogWrite(IN3, PWM_L); analogWrite(IN4, 0); }
        else { analogWrite(IN3, 0); analogWrite(IN4, -PWM_L); }

        // --- Debug ---
        Serial.print("Theta: "); Serial.print(theta * 180.0 / PI);
        Serial.print("\tPWM_L: "); Serial.print(PWM_L);
        Serial.print("\tPWM_R: "); Serial.println(PWM_R);
    }

    if (stopAtEnd) stopmotors();
}


// ======= SETUP =======
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

  Timer1.initialize(5000);
  Timer1.attachInterrupt(updateOdometrie);

  // Example movements
  moveDistance(1000, 150);
  delay(1000);
  dour(90, 120, true);
  delay(1000);
  moveDistance(-500, 150);
}

void loop() {
  // Nothing needed here; movement controlled by functions
}
