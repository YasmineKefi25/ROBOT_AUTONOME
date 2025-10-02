#include "Aerobotix_Arduino_nav.h"
#include <TimerOne.h>

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

/*void Aerobotix_Arduino_nav::Aerobotix_Arduino_nav(){

}*/


// ===== STATIC INTERRUPT WRAPPERS =====
void Aerobotix_Arduino_nav::interruptR_static() { if (instance) instance->handleRightInterrupt(); }
void Aerobotix_Arduino_nav::interruptL_static() { if (instance) instance->handleLeftInterrupt(); }
void Aerobotix_Arduino_nav::updateOdometrie_static() { if (instance) instance->updateOdometrie(); }

// ===== ACTUAL INTERRUPT HANDLERS =====
void Aerobotix_Arduino_nav::handleRightInterrupt() {
    if (digitalRead(_interruptPinRA) == digitalRead(_interruptPinRB)) _encoderRightCount--;
    else _encoderRightCount++;
}

void Aerobotix_Arduino_nav::handleLeftInterrupt() {
    if (digitalRead(_interruptPinLA) == digitalRead(_interruptPinLB)) _encoderLeftCount++;
    else _encoderLeftCount--;
}

// ===== MAIN NAVIGATION FUNCTIONS =====
void Aerobotix_Arduino_nav::moveDistance(float distance, float speed) {
    resetControllers();

    float accelVal = 5.0f;   // cm/s^2, adjust per robot
    setMotionProfileParams(distance, speed, accelVal);
    prepareMotionProfile();

    while (abs(_dS_total - distance) > 5) {
        _sens = (_dS_total - distance < 0) ? 1 : -1;

        float current_speed = _sens * getProfileSpeed(fabs(_dS_total));
        _PWM_R = compute(current_speed, _currentvelocityRight,_i_right_erreur,_kpr,_kir);
        _PWM_L = compute(current_speed, _currentvelocityLeft,_i_left_erreur,_kpl,_kil);

        // Orientation correction
        float Theta_correction = _kTheta * (_totalR - _totalL);
        _PWM_R -= Theta_correction;
        _PWM_L += Theta_correction;

        if (_sens == 1) {
            _PWM_R = clamp(_PWM_R, _PWM_MIN, _PWM_MAX);
            _PWM_L = clamp(_PWM_L, _PWM_MIN, _PWM_MAX);
        } else {
            _PWM_R = clamp(_PWM_R, -_PWM_MAX, -_PWM_MIN);
            _PWM_L = clamp(_PWM_L, -_PWM_MAX, -_PWM_MIN);
        }

    
        


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

void Aerobotix_Arduino_nav::dour(float angle, float speed){
    resetControllers();

    float distance = angle * _PI * _entreaxe / 180.0;
    float accel = 0.25f * distance;
    float decel = 0.5f * distance;

    while (abs((_theta * 180.0 / _PI) - angle) > 2.0) {
        _sens = ((_totalR - _totalL) - distance < 0) ? 1 : -1;
       // if (_sens == -1 && stop) break;

        float current_speed = _sens * acceleration_dour(speed, fabs(distance), fabs(accel), fabs(decel));
        _PWM_R = compute(current_speed, _currentvelocityRight,_i_right_erreur,_kpr,_kir);
        _PWM_L = compute(current_speed, _currentvelocityRight,_i_left_erreur,_kpl,_kil);
        // Position correction
        float pos_corr = _k_position * (_totalR + _totalL);
        _PWM_R += pos_corr;
        _PWM_L -= pos_corr;

        if (_sens == 1) {
            _PWM_L = clamp(_PWM_L, -_PWM_MAX_DOURA, -_PWM_MIN_DOURA);
            _PWM_R = clamp(_PWM_R, _PWM_MIN_DOURA, _PWM_MAX_DOURA);
        } else {
            _PWM_L = clamp(_PWM_L, _PWM_MIN_DOURA, _PWM_MAX_DOURA);
            _PWM_R = clamp(_PWM_R, -_PWM_MAX_DOURA, -_PWM_MIN_DOURA);
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

//void Aerobotix_Arduino_nav::rotate(float angle, float speed)
   // { dour(angle, speed, true); }

void Aerobotix_Arduino_nav::go(float targetX, float targetY, float speed) {
    float deltaX = targetX - _posX;
    float deltaY = targetY - _posY;
    float targetAngle_rad = atan2(deltaY, deltaX);
    float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    float relative_angle_rad = targetAngle_rad - _theta;
    while (relative_angle_rad > _PI) relative_angle_rad -= 2.0f * _PI;
    while (relative_angle_rad < -_PI) relative_angle_rad += 2.0f * _PI;

    float relative_angle_deg = RadToDeg(relative_angle_rad);
    dour(relative_angle_deg, speed * 0.8f);
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
    if (_theta > _PI) _theta -= 2.0f * _PI;
    else if (_theta < -_PI) _theta += 2.0f * _PI;

    _posX += _dS * cosf(_theta);
    _posY += _dS * sinf(_theta);

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
    float time_interval = float(_speed_ech) * 5.0f / 1000.0f;  // 5ms per tick
    float right_speed = float(_total_ech_r) / time_interval;
    float left_speed  = float(_total_ech_l) / time_interval;
    float alpha_speed = float(_total_ech_r - _total_ech_l) / time_interval;

    _currentvelocityRight = (right_speed + left_speed) / 2.0f + alpha_speed * _wheel_radius / 2.0f;
    _currentvelocityLeft  = (right_speed + left_speed) / 2.0f - alpha_speed * _wheel_radius / 2.0f;
}

// ===== UTILITY =====
float Aerobotix_Arduino_nav::RadToDeg(float radians) { return radians * (180.0f / _PI); }
float Aerobotix_Arduino_nav::DegToRad(float degrees) { return degrees * (_PI / 180.0f); }

float Aerobotix_Arduino_nav::calculDistance(long deltaL, long deltaR, float radius, int nb_ticks) {
    _dsL = (deltaL / (float)nb_ticks) * 2.0f * _PI * radius;
    _dsR = (deltaR / (float)nb_ticks) * 2.0f * _PI * radius;
    return (_dsL + _dsR) / 2.0f;
}

void Aerobotix_Arduino_nav::setMotionProfileParams(float dist, float vmax, float accel) {
    profileDistance = fabs(dist);
    targetSpeed = vmax;
    acceleration = accel;
}

void Aerobotix_Arduino_nav::prepareMotionProfile() {
    float acc_dist = (targetSpeed * targetSpeed) / (2.0f * acceleration);
    if (2.0f * acc_dist > profileDistance) {
        isTriangular = true;
        peakSpeed = sqrtf(acceleration * profileDistance);
        accelDistance = profileDistance / 2.0f;
        decelDistance = profileDistance / 2.0f;
        cruiseDistance = 0.0f;
    } else {
        isTriangular = false;
        peakSpeed = targetSpeed;
        accelDistance = acc_dist;
        decelDistance = acc_dist;
        cruiseDistance = profileDistance - 2.0f * acc_dist;
    }
}

float Aerobotix_Arduino_nav::getProfileSpeed(float traveled) {
    float v;
    if (!isTriangular) {
        if (traveled < accelDistance) v = sqrtf(2.0f * acceleration * traveled);
        else if (traveled < accelDistance + cruiseDistance) v = peakSpeed;
        else {
            float decelTraveled = traveled - (accelDistance + cruiseDistance);
            v = sqrtf(fmaxf(0.0f, peakSpeed * peakSpeed - 2.0f * acceleration * decelTraveled));
        }
    } else {
        if (traveled < accelDistance) v = sqrtf(2.0f * acceleration * traveled);
        else {
            float decelTraveled = traveled - accelDistance;
            v = sqrtf(fmaxf(0.0f, peakSpeed * peakSpeed - 2.0f * acceleration * decelTraveled));
        }
    }
    return v;
}

// ===== PRIVATE CONTROL =====
void Aerobotix_Arduino_nav::run() {
    if (_PWM_R > 0) { analogWrite(_IN1, _PWM_R); analogWrite(_IN2, 0); }
    else { analogWrite(_IN1, 0); analogWrite(_IN2, -_PWM_R); }

    if (_PWM_L > 0) { analogWrite(_IN3, _PWM_L); analogWrite(_IN4, 0); }
    else { analogWrite(_IN3, 0); analogWrite(_IN4, -_PWM_L); }
}

void Aerobotix_Arduino_nav::resetControllers() {
    _totalR = _totalL = _dS_total = 0;
    _i_right_erreur = _i_left_erreur = _right_erreur = _left_erreur = 0;
    _position_erreur = _orientation_erreur = 0;
    
}

float Aerobotix_Arduino_nav::clamp(float PWM, float min, float max) {
    if (PWM < min) return min;
    if (PWM > max) return max;
    return PWM;
}

float Aerobotix_Arduino_nav::acceleration_dour(float speed, float distance, float accel, float decel) {
    float delta = _totalR - _totalL;
    float current_speed;

    if (delta < accel) current_speed = (speed / accel) * delta;
    else if (distance - delta < decel) current_speed = (speed / -decel) * delta + speed - ((distance - decel) * (speed / -decel));
    else current_speed = speed;

    return clamp(current_speed, 0, speed);
}

int Aerobotix_Arduino_nav::constraint(float a, int min, int max) {
    if (a < min) return min;
    if (a > max) return max;
    return (int)a;
}

float Aerobotix_Arduino_nav::getcurrentVelocity(float dist, float t) { return dist / t; }
float Aerobotix_Arduino_nav::angleToDistance(float angleRad, float radius) { return radius * RadToDeg(angleRad); }
float Aerobotix_Arduino_nav::ramp(int time) { return time * 0.6f; }