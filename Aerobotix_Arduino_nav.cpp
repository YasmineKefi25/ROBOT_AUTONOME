#include "Aerobotix_Arduino_nav.h"
#include <TimerOne.h>
#include <math.h> 

// ===== STATIC INSTANCE POINTER =====
Aerobotix_Arduino_nav* Aerobotix_Arduino_nav::instance = &aerobotix_arduino_nav;

// Create global instance
Aerobotix_Arduino_nav aerobotix_arduino_nav;

// ===== CONSTRUCTOR =====
Aerobotix_Arduino_nav::Aerobotix_Arduino_nav() {
    // Initialization done in header with default values
}

// ===== INITIALIZATION =====
void Aerobotix_Arduino_nav::begin() {
    Serial.begin(9600);

    // Motor pins
    pinMode(_IN1, OUTPUT);
    pinMode(_IN2, OUTPUT);
    pinMode(_IN3, OUTPUT);
    pinMode(_IN4, OUTPUT);

    // Encoder pins
    pinMode(_interruptPinRA, INPUT_PULLUP);
    pinMode(_interruptPinRB, INPUT_PULLUP);
    pinMode(_interruptPinLA, INPUT_PULLUP);
    pinMode(_interruptPinLB, INPUT_PULLUP);

    // Attach interrupts using static wrappers
    attachInterrupt(digitalPinToInterrupt(_interruptPinRA), interruptR_static, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_interruptPinLA), interruptL_static, CHANGE);

    // Timer for odometry
    Timer1.initialize(5000);  // 5ms
    Timer1.attachInterrupt(updateOdometrie_static);

    _previousMillis = millis();

    Serial.println("Aerobotix_Arduino_nav initialized");
}

// ===== STATIC INTERRUPT WRAPPERS =====
void Aerobotix_Arduino_nav::interruptR_static() {
    if (instance) instance->handleRightInterrupt();
}

void Aerobotix_Arduino_nav::interruptL_static() {
    if (instance) instance->handleLeftInterrupt();
}

void Aerobotix_Arduino_nav::updateOdometrie_static() {
    if (instance) instance->updateOdometrie();
}

// ===== ACTUAL INTERRUPT HANDLERS =====
void Aerobotix_Arduino_nav::handleRightInterrupt() {
    if (digitalRead(_interruptPinRA) == digitalRead(_interruptPinRB))
        _encoderRightCount--;
    else
        _encoderRightCount++;
}

void Aerobotix_Arduino_nav::handleLeftInterrupt() {
    if (digitalRead(_interruptPinLA) == digitalRead(_interruptPinLB))
        _encoderLeftCount++;
    else
        _encoderLeftCount--;
}

// ===== MAIN NAVIGATION FUNCTIONS =====
void Aerobotix_Arduino_nav::moveDistance(float distance, float speed) {
    iniiit();

    // --- prepare motion profile ---
    // distance  = distance (cm)
    // speed     = desired vmax (cm/s)    <-- make sure 'speed' unit matches expected units
    // accelVal  = acceleration in cm/s^2 (choose a realistic value for your robot)
    float accelVal = 5.0f;   // <-- adjust this to suit your robot (example: 5 cm/s^2)
    setMotionProfileParams(distance, speed, accelVal);
    prepareMotionProfile();

    // debug prints (optional)
    // Serial.print("d_acc: "); Serial.println(_mp_d_acc);
    // Serial.print("d_const: "); Serial.println(_mp_d_const);
    // Serial.print("d_dec: "); Serial.println(_mp_d_dec);
    // Serial.print("vpeak: "); Serial.println(_mp_vpeak);

    while (abs(_dS_total - distance) > 5) {
        _sens = (_dS_total - distance < 0) ? 1 : -1;

        // IMPORTANT: your existing acceleration() expects accel and decel as *distances*,
        // so we pass the computed distances _mp_d_acc and _mp_d_dec.
        float current_speed = _sens * acceleration(speed, abs(distance), fabs(_mp_d_acc), fabs(_mp_d_dec));
        ...

        // Right PID
        _right_erreur = current_speed - _currentvelocityRight;
        _i_right_erreur += _right_erreur;
        _PWM_R = _kp_right * _right_erreur + _ki_right * _i_right_erreur;


        // Left PID
        _left_erreur = current_speed - _currentvelocityLeft;
        _i_left_erreur += _left_erreur;
        _PWM_L = _kp_left * _left_erreur + _ki_left * _i_left_erreur;


        // Orientation correction
        float Theta_correction = _kTheta * (_totalR - _totalL);
        _PWM_R -= Theta_correction;
        _PWM_L += Theta_correction;

        if (_sens == 1) {
            _PWM_R = erreur(_PWM_R, _PWM_MIN, _PWM_MAX);
            _PWM_L = erreur(_PWM_L, _PWM_MIN, _PWM_MAX);
        } else {
            _PWM_R = erreur(_PWM_R, -_PWM_MAX, -_PWM_MIN);
            _PWM_L = erreur(_PWM_L, -_PWM_MAX, -_PWM_MIN);
        }

        // Swap PWM if needed
        float PWM_test = _PWM_R;
        _PWM_R = _PWM_L;
        _PWM_L = PWM_test;

        run();

        Serial.print("Distance: "); Serial.print(_dS_total);
        Serial.print(" / "); Serial.print(distance);
        Serial.print(" PWM_R: "); Serial.print(_PWM_R);
        Serial.print(" PWM_L: "); Serial.println(_PWM_L);

        delay(10);
    }
    stopmotors();
    Serial.print("Move completed. Total distance: "); Serial.println(_dS_total);
}

void Aerobotix_Arduino_nav::dour(float angle, float speed, bool stop) {
    iniiit();
    float distance = angle * _PI * _entreaxe / 180.0;
    float accel = 0.25 * distance;
    float decel = 0.5 * distance;

    while (abs((_theta * 180.0 / _PI) - angle) > 2.0) {
        _sens = ((_totalR - _totalL) - distance < 0) ? 1 : -1;
        if (_sens == -1 && stop) break;

        float current_speed = _sens * acceleration_dour(speed, abs(distance), abs(accel), abs(decel));

        // Right PID
        _right_erreur = current_speed - _currentvelocityRight;
        _i_right_erreur += _right_erreur;
        _PWM_R = _kp_dour * _right_erreur;
        _PWM_R = (_sens == 1) ? erreur(_PWM_R, _PWM_MIN_DOURA, _PWM_MAX_DOURA) : erreur(_PWM_R, -_PWM_MAX_DOURA, -_PWM_MIN_DOURA);

        // Left PID
        _left_erreur = -current_speed - _currentvelocityLeft;
        _i_left_erreur += _left_erreur;
        _PWM_L = _kp_dour * _left_erreur;
        _PWM_L = (_sens == 1) ? erreur(_PWM_L, -_PWM_MAX_DOURA, -_PWM_MIN_DOURA) : erreur(_PWM_L, _PWM_MIN_DOURA, _PWM_MAX_DOURA);

        // Position correction
        float pos_corr = _k_position * (_totalR + _totalL);
        _PWM_R += pos_corr;
        _PWM_L -= pos_corr;

        if (_sens == 1) {
            _PWM_L = erreur(_PWM_L, -_PWM_MAX_DOURA, -_PWM_MIN_DOURA);
            _PWM_R = erreur(_PWM_R, _PWM_MIN_DOURA, _PWM_MAX_DOURA);
        } else {
            _PWM_L = erreur(_PWM_L, _PWM_MIN_DOURA, _PWM_MAX_DOURA);
            _PWM_R = erreur(_PWM_R, -_PWM_MAX_DOURA, -_PWM_MIN_DOURA);
        }

        float PWM_test = _PWM_R;
        _PWM_R = _PWM_L;
        _PWM_L = PWM_test;

        run();

        Serial.print("Angle: "); Serial.print(_theta * 180.0 / _PI);
        Serial.print(" / "); Serial.print(angle);
        Serial.print(" PWM_R: "); Serial.print(_PWM_R);
        Serial.print(" PWM_L: "); Serial.println(_PWM_L);

        delay(10);
    }
    stopmotors();
}

void Aerobotix_Arduino_nav::rotate(float angle, float speed) {
    dour(angle, speed, true);
}

void Aerobotix_Arduino_nav::go(float targetX, float targetY, float speed) {
    float deltaX = targetX - (_dS_total * cos(_theta));
    float deltaY = targetY - (_dS_total * sin(_theta));
    float targetAngle = atan2(deltaY, deltaX);
    float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    targetAngle = RadToDeg(targetAngle);
    while (targetAngle < 0) targetAngle += 360;
    while (targetAngle >= 360) targetAngle -= 360;

    rotate(targetAngle, speed * 0.8);
    delay(500);
    moveDistance(distance, speed);
}

void Aerobotix_Arduino_nav::stopmotors() {
    digitalWrite(_IN1, LOW);
    digitalWrite(_IN2, LOW);
    digitalWrite(_IN3, LOW);
    digitalWrite(_IN4, LOW);
}

// ===== ODOMETRY =====
void Aerobotix_Arduino_nav::updateOdometrie() {
    _t++;

    long deltaL = _encoderLeftCount - _lastEncoderLeftCount;
    long deltaR = _encoderRightCount - _lastEncoderRightCount;

    _lastEncoderLeftCount = _encoderLeftCount;
    _lastEncoderRightCount = _encoderRightCount;

    _dS = calculDistance(deltaL, deltaR, _wheel_radius, _nb_ticks);
    _totalL += _dsL;
    _totalR += _dsR;
    _dS_total += _dS;

    _total_ech_l += _dsL;
    _total_ech_r += _dsR;

    _dTheta = (_dsR - _dsL) / _entreaxe;
    _theta += _dTheta;

    if (_theta > _PI) _theta -= 2.0 * _PI;
    else if (_theta < -_PI) _theta += 2.0 * _PI;

    if (_t % _speed_ech == 0) {
        speed_calcul();
        _total_ech_l = 0;
        _total_ech_r = 0;
    }

    _dsR = 0;
    _dsL = 0;
}

// ===== SPEED CALCULATION =====
void Aerobotix_Arduino_nav::speed_calcul() {
    float right_speed = float(_total_ech_r / (float(_speed_ech) * 15.0 / 1000.0));
    float left_speed = float(_total_ech_l / (float(_speed_ech) * 15.0 / 1000.0));
    float alpha_speed = (float(_total_ech_r - _total_ech_l)) / (float(_speed_ech) * 15.0 / 1000.0);

    _currentvelocityRight = (right_speed + left_speed) / 2.0 + alpha_speed * _wheel_radius / 2.0;
    _currentvelocityLeft = (right_speed + left_speed) / 2.0 - alpha_speed * _wheel_radius / 2.0;
}

// ===== UTILITY =====
float Aerobotix_Arduino_nav::RadToDeg(float radians) { return radians * (180.0 / _PI); }
float Aerobotix_Arduino_nav::DegToRad(float degrees) { return degrees * (_PI / 180.0); }
float Aerobotix_Arduino_nav::calculDistance(long deltaL, long deltaR, float radius, int nb_ticks) {
    _dsL = (deltaL / (float)nb_ticks) * 2.0 * _PI * radius;
    _dsR = (deltaR / (float)nb_ticks) * 2.0 * _PI * radius;
    return (_dsL + _dsR) / 2.0;
}




// Set parameters for the motion profile (call this before prepareMotionProfile)
void Aerobotix_Arduino_nav::setMotionProfileParams(float dist, float vmax, float accel) {
    _mp_dist  = dist;
    _mp_vmax  = vmax;
    _mp_accel = accel;
}

// Compute the distances (d_acc, d_dec, d_const) and peak speed (vpeak)
// based on the input parameters. Uses clamped/triangular vs trapezoidal logic.
void Aerobotix_Arduino_nav::prepareMotionProfile() {
    // safety guard: avoid divide-by-zero (if user forgot to set accel)
    if (_mp_accel <= 0.0f) {
        // choose a small default acceleration so nothing explodes,
        // but you should set a realistic accel in setMotionProfileParams.
        _mp_accel = 1.0f;
    }

    // Distance required to accelerate from 0 to vmax: d = v^2 / (2*a)
    float acc_dist = (_mp_vmax * _mp_vmax) / (2.0f * _mp_accel);

    // If we don't have enough distance to accelerate and decelerate to vmax:
    if (2.0f * acc_dist > _mp_dist) {
        // Triangular profile (no constant-speed phase)
        _mp_triangular = true;

        // If we split distance in two (acc + dec) with equal accel,
        // the reachable peak speed vpeak satisfies: vpeak^2 =  a * dist
        _mp_vpeak   = sqrtf(_mp_accel * _mp_dist);
        _mp_d_acc   = _mp_dist / 2.0f;
        _mp_d_dec   = _mp_dist / 2.0f;
        _mp_d_const = 0.0f;
    } else {
        // Trapezoidal profile (reach vmax, then constant, then decel)
        _mp_triangular = false;

        _mp_vpeak   = _mp_vmax;
        _mp_d_acc   = acc_dist;
        _mp_d_dec   = acc_dist;
        _mp_d_const = _mp_dist - 2.0f * acc_dist;
    }
}

// ===== PRIVATE CONTROL =====
void Aerobotix_Arduino_nav::applyMotorCommand(float cmdR, float cmdL) {
    digitalWrite(_IN1, cmdR);
    digitalWrite(_IN2, cmdR);
    digitalWrite(_IN3, cmdL);
    digitalWrite(_IN4, cmdL);
}

void Aerobotix_Arduino_nav::run() {
    if (_PWM_R > 0) { analogWrite(_IN1, _PWM_R); analogWrite(_IN2, 0); }
    else { analogWrite(_IN1, 0); analogWrite(_IN2, -_PWM_R); }

    if (_PWM_L > 0) { analogWrite(_IN3, _PWM_L); analogWrite(_IN4, 0); }
    else { analogWrite(_IN3, 0); analogWrite(_IN4, -_PWM_L); }
}

void Aerobotix_Arduino_nav::iniiit() {
    _totalR = _totalL = _dS_total = 0;
    _i_right_erreur = _i_left_erreur = _right_erreur = _left_erreur = 0;
    _position_erreur = _orientation_erreur = 0;
}

float Aerobotix_Arduino_nav::erreur(float PWM, float min, float max) {
    if (PWM < min) return min;
    if (PWM > max) return max;
    return PWM;
}

float Aerobotix_Arduino_nav::acceleration(float speed, float distance, float accel, float decel) {
    float current_speed;
    if (abs(_dS_total) < accel) current_speed = (speed / accel) * abs(_dS_total);
    else if (distance - abs(_dS_total) < decel) current_speed = (speed / -decel) * abs(_dS_total) + speed - ((distance - decel) * (speed / -decel));
    else current_speed = speed;
    return current_speed;
}

float Aerobotix_Arduino_nav::acceleration_dour(float speed, float distance, float accel, float decel) {
    float delta = _totalR - _totalL;
    float current_speed;
    if (delta < accel) current_speed = (speed / accel) * delta;
    else if (distance - delta < decel) current_speed = (speed / -decel) * delta + speed - ((distance - decel) * (speed / -decel));
    else current_speed = speed;
    return erreur(current_speed, _PWM_MIN, speed);
}

int Aerobotix_Arduino_nav::constraint(float a, int min, int max) { if (a < min) return min; if (a > max) return max; return a; }
float Aerobotix_Arduino_nav::getcurrentVelocity(float dist, float t) { return dist / t; }
float Aerobotix_Arduino_nav::angleToDistance(float angleRad, float radius) { return radius * RadToDeg(angleRad); }
float Aerobotix_Arduino_nav::ramp(int time) { return time * 0.6; }
