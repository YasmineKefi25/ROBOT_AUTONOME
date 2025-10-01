
#ifndef AEROBOTIX_ARDUINO_NAV_H
#define AEROBOTIX_ARDUINO_NAV_H

#include <Arduino.h>
#include <TimerOne.h>

class Aerobotix_Arduino_nav {
public:
    // ===== CONSTRUCTOR =====
    Aerobotix_Arduino_nav();

    // ===== INITIALIZATION =====
    void begin();

    // ===== MAIN NAVIGATION FUNCTIONS =====
    void moveDistance(float distance, float speed);
    void dour(float angle, float speed, bool stop = false);
    void rotate(float angle, float speed);
    void go(float targetX, float targetY, float speed);
    void stopmotors();
    void updateOdometrie();

    // ===== UTILITY FUNCTIONS =====
    float RadToDeg(float radians);
    float DegToRad(float degrees);
    float calculDistance(long deltaLeftCount, long deltaRightCount, float wheel_radius, int nb_ticks);
    void speed_calcul();

        // ===== GETTER FUNCTIONS =====
    // Motor pins
    uint8_t getIN1() { return _IN1; }
    uint8_t getIN2() { return _IN2; }
    uint8_t getIN3() { return _IN3; }
    uint8_t getIN4() { return _IN4; }

    // Encoder pins
    uint8_t getInterruptPinRA() { return _interruptPinRA; }
    uint8_t getInterruptPinRB() { return _interruptPinRB; }
    uint8_t getInterruptPinLA() { return _interruptPinLA; }
    uint8_t getInterruptPinLB() { return _interruptPinLB; }

    // Physical parameters
    float getWheelRadius() { return _wheel_radius; }
    float getEntreaxe() { return _entreaxe; }
    int getNbTicks() { return _nb_ticks; }

    // Ticks conversion
    float getTickcmR() { return _tickcmR; }
    float getTickcmL() { return _tickcmL; }
    int getTickZR_P() { return _tickZR_P; }
    int getTickZL_N() { return _tickZL_N; }
    int getTickZL_P() { return _tickZL_P; }
    int getTickZR_N() { return _tickZR_N; }

    // Control parameters
    int getMaxSpeed() { return _maxSpeed; }
    int getMinSpeed() { return _minSpeed; }
    int getMaxAcc() { return _maxAcc; }
    float getPI() { return _PI; }

    // PID parameters
    float getKp() { return _kp; }
    float getKi() { return _ki; }
    float getKTheta() { return _kTheta; }
    float getKpDour() { return _kp_dour; }
    float getKPosition() { return _k_position; }

    // PWM limits
    float getPWMMin() { return _PWM_MIN; }
    float getPWMMax() { return _PWM_MAX; }
    float getPWMMinDoura() { return _PWM_MIN_DOURA; }
    float getPWMMaxDoura() { return _PWM_MAX_DOURA; }

    // Speed calculation
    int getSpeedEch() { return _speed_ech; }

    // Navigation state
    float getCurrentVelocityRight() { return _currentvelocityRight; }
    float getCurrentVelocityLeft() { return _currentvelocityLeft; }
    long getEncoderLeftCount() { return _encoderLeftCount; }
    long getEncoderRightCount() { return _encoderRightCount; }
    float getTheta() { return _theta; }
    float getDSTotal() { return _dS_total; }
    float getPWM_R() { return _PWM_R; }
    float getPWM_L() { return _PWM_L; }
    int getSens() { return _sens; }

    // ===== SETTER FUNCTIONS =====
    // Motor pins
    void setIN1(uint8_t pin) { _IN1 = pin; }
    void setIN2(uint8_t pin) { _IN2 = pin; }
    void setIN3(uint8_t pin) { _IN3 = pin; }
    void setIN4(uint8_t pin) { _IN4 = pin; }

    // Encoder pins
    void setInterruptPinRA(uint8_t pin) { _interruptPinRA = pin; }
    void setInterruptPinRB(uint8_t pin) { _interruptPinRB = pin; }
    void setInterruptPinLA(uint8_t pin) { _interruptPinLA = pin; }
    void setInterruptPinLB(uint8_t pin) { _interruptPinLB = pin; }

    // Physical parameters
    void setWheelRadius(float radius) { _wheel_radius = radius; }
    void setEntreaxe(float distance) { _entreaxe = distance; }
    void setNbTicks(int ticks) { _nb_ticks = ticks; }

    // Ticks conversion
    void setTickcmR(float ticks) { _tickcmR = ticks; }
    void setTickcmL(float ticks) { _tickcmL = ticks; }
    void setTickZR_P(int ticks) { _tickZR_P = ticks; }
    void setTickZL_N(int ticks) { _tickZL_N = ticks; }
    void setTickZL_P(int ticks) { _tickZL_P = ticks; }
    void setTickZR_N(int ticks) { _tickZR_N = ticks; }

    // Control parameters
    void setMaxSpeed(int speed) { _maxSpeed = speed; }
    void setMinSpeed(int speed) { _minSpeed = speed; }
    void setMaxAcc(int acc) { _maxAcc = acc; }
    void setPI(float pi) { _PI = pi; }

    // PID parameters
    void setKp_right(float kpr) { _kpr = kpr; }
    void setKp_left(float kpl) { _kpl = kpl; }
    void setKi_right(float kir) { _kir = kir; }
    void setKi_left(float kil) { _kil = kil; }

    void setKTheta(float ktheta) { _kTheta = ktheta; }
    void setKpDour(float kp_dour) { _kp_dour = kp_dour; }
    void setKPosition(float k_position) { _k_position = k_position; }

    // PWM limits
    void setPWMMin(float min) { _PWM_MIN = min; }
    void setPWMMax(float max) { _PWM_MAX = max; }
    void setPWMMinDoura(float min) { _PWM_MIN_DOURA = min; }
    void setPWMMaxDoura(float max) { _PWM_MAX_DOURA = max; }

    // Speed calculation
    void setSpeedEch(int ech) { _speed_ech = ech; }

    // Navigation state (use with caution)
    void setCurrentVelocityRight(float vel) { _currentvelocityRight = vel; }
    void setCurrentVelocityLeft(float vel) { _currentvelocityLeft = vel; }
    void setEncoderLeftCount(long count) { _encoderLeftCount = count; }
    void setEncoderRightCount(long count) { _encoderRightCount = count; }
    void setTheta(float theta) { _theta = theta; }
    void setDSTotal(float dist) { _dS_total = dist; }
    void setPWM_R(float pwm) { _PWM_R = pwm; }
    void setPWM_L(float pwm) { _PWM_L = pwm; }
    void setSens(int sens) { _sens = sens; }

private:
    // ===== PRIVATE VARIABLES =====
    // Motor pins
    uint8_t _IN1 = 3, _IN2 = 2, _IN3 = 4, _IN4 = 5;

    // Encoder pins
    uint8_t _interruptPinRA = 18, _interruptPinRB = 19, _interruptPinLA = 20, _interruptPinLB = 21;

    // Physical parameters
    float _wheel_radius = 39.55, _entreaxe = 305;
    int _nb_ticks = 800;

    // Navigation & control variables
    float _currentvelocityRight = 0, _currentvelocityLeft = 0;
    long _encoderLeftCount = 0, _encoderRightCount = 0;
    float _theta = 0, _dS_total = 0;
    float _PWM_R = 0, _PWM_L = 0;
    int _sens = 1, _speed_ech = 10;

    // Error terms & odometry
    float _right_erreur = 0, _left_erreur = 0;
    float _i_right_erreur = 0, _i_left_erreur = 0;
    float _orientation_erreur = 0, _position_erreur = 0;
    long _lastEncoderLeftCount = 0, _lastEncoderRightCount = 0;
    float _totalL = 0, _totalR = 0;
    float _dsR = 0, _dsL = 0, _dS = 0, _dTheta = 0;
    float _total_ech_l = 0, _total_ech_r = 0;
    unsigned long _previousMillis = 0;
    long _t = 0;
    // ===== CONTROL PARAMETERS =====
    int _maxSpeed = 255;
    int _minSpeed = 0;
    int _maxAcc = 100;  
    float _PI = 3.14159265;

    // ===== PID PARAMETERS =====
    float _kpr = 1.0;
    float _kpl = 1.0;
    float _kir = 0.0;
    float _kil=0.0;
    float _kTheta = 1.0;
    float _kp_dour = 1.0;
    float _k_position = 1.0;

    // ===== TICKS CONVERSION =====
    float _tickcmR = 0, _tickcmL = 0;
    int _tickZR_P = 0, _tickZL_N = 0, _tickZL_P = 0, _tickZR_N = 0;

    // ===== PWM LIMITS =====
    float _PWM_MIN = 0, _PWM_MAX = 255;
    float _PWM_MIN_DOURA = 0, _PWM_MAX_DOURA = 255;


    // ===== STATIC ISR SUPPORT =====
    static Aerobotix_Arduino_nav* instance; // pointer for static ISRs
    static void interruptR_static();
    static void interruptL_static();
    static void updateOdometrie_static();

    // Actual handlers
    void handleRightInterrupt();
    void handleLeftInterrupt();

    // ===== PRIVATE METHODS =====
    void applyMotorCommand(float cmdPwmRight, float cmdPwmLeft);
    void run();
    void iniiit();
    float erreur(float PWM, float min, float max);
    float acceleration(float speed, float distance, float accel, float decel);
    float acceleration_dour(float speed, float distance, float accel, float decel);
    int constraint(float a, int min, int max);
    float getcurrentVelocity(float dist, float t);
    float angleToDistance(float angleRad, float radius);
    float ramp(int time);

    // ===== Motion profile (add to private section) =====
float _mp_dist = 0.0f;     // requested travel distance (cm)
float _mp_vmax = 0.0f;     // desired max speed (cm/s)
float _mp_accel = 0.0f;    // acceleration (cm/s^2)

float _mp_d_acc = 0.0f;    // computed accel distance (cm)
float _mp_d_dec = 0.0f;    // computed decel distance (cm)
float _mp_d_const = 0.0f;  // computed constant-speed distance (cm)
float _mp_vpeak = 0.0f;    // peak speed actually reached (cm/s)
bool  _mp_triangular = false; // true if triangular profile

// helpers
void setMotionProfileParams(float dist, float vmax, float accel);
void prepareMotionProfile();

};

// Global instance for ISR
extern Aerobotix_Arduino_nav aerobotix_arduino_nav;

#endif // AEROBOTIX_ARDUINO_NAV_H
