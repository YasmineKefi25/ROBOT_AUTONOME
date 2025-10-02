#ifndef AEROBOTIX_ARDUINO_NAV_H
#define AEROBOTIX_ARDUINO_NAV_H

#include <Arduino.h>
#include <math.h>

// ===== MAIN CLASS =====
class Aerobotix_Arduino_nav {
public:
    // constructor
    Aerobotix_Arduino_nav();

    // initialization
    void begin();

    // main navigation
    void moveDistance(float distance, float speed);
    void dour(float angle, float speed);
   // void rotate(float angle, float speed);
    void go(float targetX, float targetY, float speed);
    void stopmotors();

    // odometry / periodic
    void updateOdometrie();

    // utility
    float RadToDeg(float radians);
    float DegToRad(float degrees);
    float calculDistance(long deltaLeftCount, long deltaRightCount, float wheel_radius, int nb_ticks);
    void speed_calcul();

    // getters (examples kept)
    uint8_t getIN1() { return _IN1; }
    uint8_t getIN2() { return _IN2; }
    uint8_t getIN3() { return _IN3; }
    uint8_t getIN4() { return _IN4; }

    // ... (other getters/setters you had can remain, omitted here for brevity)
    // I'll include the critical ones you used in the .cpp:

    float getWheelRadius() { return _wheel_radius; }
    float getEntreaxe() { return _entreaxe; }
    int getNbTicks() { return _nb_ticks; }

    float getCurrentVelocityRight() { return _currentvelocityRight; }
    float getCurrentVelocityLeft()  { return _currentvelocityLeft; }
    long  getEncoderLeftCount()     { return _encoderLeftCount; }
    long  getEncoderRightCount()    { return _encoderRightCount; }
    float getTheta()                { return _theta; }
    float getDSTotal()              { return _dS_total; }
    float getPWM_R()                { return _PWM_R; }
    float getPWM_L()                { return _PWM_L; }

    // setters (keep if you need runtime changes)
    void setIN1(uint8_t pin) { _IN1 = pin; }
    void setIN2(uint8_t pin) { _IN2 = pin; }
    void setIN3(uint8_t pin) { _IN3 = pin; }
    void setIN4(uint8_t pin) { _IN4 = pin; }

    //setters for interruption pins
    setInterruptPinRA(uint8_t pin) { _interruptPinRA = pin; }
    setInterruptPinRB(uint8_t pin) { _interruptPinRB = pin; }
    setInterruptPinLA(uint8_t pin) { _interruptPinLA = pin; }
    setInterruptPinLB(uint8_t pin) { _interruptPinLB = pin; }

    void setWheelRadius(float radius) { _wheel_radius = radius; }
    void setEntreaxe(float distance)   { _entreaxe = distance; }
    void setNbTicks(int ticks)         { _nb_ticks = ticks; }
    void setCoefficientsRight(double kprValue, double kirValue) {
        _kpr = kprValue;
        _kir = kirValue;
    };


   void setCoefficientsLeft(double kplValue, double kilValue) {
        _kpl = kplValue;
        _kil = kilValue;
    };

   /* void setMotionProfileParams(float dist, float vmax, float accel);
    void prepareMotionProfile();*/


private:
    // hardware pins
    uint8_t _IN1 = 3, _IN2 = 2, _IN3 = 4, _IN4 = 5;
    uint8_t _interruptPinRA = 18, _interruptPinRB = 19, _interruptPinLA = 20, _interruptPinLB = 21;

    // physical parameters
    float _wheel_radius = 39.55f;    // mm or cm? keep consistent with your .cpp usage
    float _entreaxe     = 305.0f;
    int   _nb_ticks     = 800;

    // navigation state
    float _currentvelocityRight = 0.0f;
    float _currentvelocityLeft  = 0.0f;
    long  _encoderLeftCount  = 0;
    long  _encoderRightCount = 0;
    float _theta = 0.0f;
    float _dS_total = 0.0f;
    float _PWM_R = 0.0f;
    float _PWM_L = 0.0f;
    int   _sens = 1;
    int   _speed_ech = 10;

    float _posX = 0.0f, _posY = 0.0f;

    // odometry helpers
    float _right_erreur = 0.0f, _left_erreur = 0.0f;
    float _i_right_erreur = 0.0f, _i_left_erreur = 0.0f;
    float _orientation_erreur = 0.0f, _position_erreur = 0.0f;
    long  _lastEncoderLeftCount = 0, _lastEncoderRightCount = 0;
    float _totalL = 0.0f, _totalR = 0.0f;
    float _dsR = 0.0f, _dsL = 0.0f, _dS = 0.0f, _dTheta = 0.0f;
    float _total_ech_l = 0.0f, _total_ech_r = 0.0f;
    unsigned long _previousMillis = 0;
    long _t = 0;
    float dt=0.01f;



    // control params
    int _maxSpeed = 255;
    int _minSpeed = 0;
    int _maxAcc = 100;
    float _PI = 3.14159265f;

    // PID params
    float _kpr = 1.0f;
    float _kpl = 1.0f;
    float _kir = 0.5f;
    float _kil = 0.5f;
    float _kTheta = 1.0f;
    float _kp_dour = 1.0f;
    float _k_position = 1.0f;

    // tick conversions
    float _tickcmR = 0.0f, _tickcmL = 0.0f;
    int _tickZR_P = 0, _tickZL_N = 0, _tickZL_P = 0, _tickZR_N = 0;

    // PWM limits
    float _PWM_MIN = 0.0f, _PWM_MAX = 255.0f;
    float _PWM_MIN_DOURA = 0.0f, _PWM_MAX_DOURA = 255.0f;

    // motion profile
    float profileDistance = 0.0f;
    float targetSpeed = 0.0f;
    float acceleration = 0.0f;
    bool  isTriangular = false;
    float peakSpeed = 0.0f;
    float accelDistance = 0.0f;
    float cruiseDistance = 0.0f;
    float decelDistance = 0.0f;

    // ISR support
    static Aerobotix_Arduino_nav* instance;
    static void interruptR_static();
    static void interruptL_static();
    static void updateOdometrie_static();

    // actual ISR handlers
    void handleRightInterrupt();
    void handleLeftInterrupt();

    // private methods (names matched with cleaned .cpp)
void run();
    void resetControllers();
    float clamp(float PWM, float min, float max);
    float acceleration_dour(float speed, float distance, float accel, float decel);
    int constraint(float a, int min, int max);
    float getcurrentVelocity(float dist, float t);
    float angleToDistance(float angleRad, float radius);
    float ramp(int time);

    // motion profile helpers
    void setMotionProfileParams(float dist, float vmax, float accel);
    void prepareMotionProfile();
    float getProfileSpeed(float traveled);

    float compute(float setpoint, float current,float integral,float kpVal,float kiVal) {
        float Kp=kpVal;
        float Ki=kiVal;
        float error = setpoint - current;
        float integralCandidate = integral + error * dt;
        float outputCandidate = Kp * error + Ki * integralCandidate;
        float output;
        if (outputCandidate > _PWM_MAX) {
            output = _PWM_MAX;
            if (error < 0) integral = integralCandidate; // anti-windup unwind
        }
        else if (outputCandidate < _PWM_MIN ) {
            output = _PWM_MIN ;
            if (error > 0) integral = integralCandidate; // anti-windup unwind
        }
        else {
            integral = integralCandidate;
            output = outputCandidate;
        }
        return output;
    }
};



    
// global instance declaration (defined in the .cpp)
extern Aerobotix_Arduino_nav aerobotix_arduino_nav;

#endif // AEROBOTIX_ARDUINO_NAV_H