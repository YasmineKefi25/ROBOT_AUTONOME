#include <TimerOne.h>
//set pins for the motors
#define IN1 4  // Pin pour IN1 du moteur 1 LEFT
#define IN2 5  // Pin pour IN2 du moteur 1
#define IN3 6  // Pin pour IN3 du moteur 2 RIGHT
#define IN4 7  // Pin pour IN4 du moteur 2
//set pins for the encoders
    //right
#define interruptPinRA 2  
#define interruptPinRB 3
    //left
#define interruptPinLB 19  
#define interruptPinLA 18  


#define PI 3.14159265358979323846
//robot specification cm?mm?
#define nb_ticks 800
#define wheel_radius 4
#define entreaxe  33.5
float previousMillis=0;


//ODOMETRY---------------------------------------------------------------------
long encoderLeftCount = 0;
long encoderRightCount = 0;
float currentvelocityRight = 0.0;
float currentvelocityLeft = 0.0;
float x = 0.0;
float y = 0.0;
long t = 0;
int speed_ech = 10;
float total_ech_l = 0;
float total_ech_r = 0;
long lastEncoderLeftCount = 0;
long lastEncoderRightCount = 0;

float dS_total = 0;//
float dsR = 0;
float dsL = 0;
float dS;
float totalL = 0;
float totalR = 0;
float dTheta = 0.0;
float theta = 0.0;



//mettre à jour les encodeurs
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

float calculDistance(long deltaLeftCount, long deltaRightCount) {
  //calculer la distance parcourue par chaque roue
  dsL = (deltaLeftCount / (float)nb_ticks) * 2.0 * PI * wheel_radius;   //distance roue gauche
  dsR = (deltaRightCount / (float)nb_ticks) * 2.0 * PI * wheel_radius;  //distance roue droite
  //retourner la distance moyenne parcourue pae le robot
  return (dsL + dsR) / 2.0;  //DISTANCE MOYENNE
}

void speed_calcul(void) {

  float right_encoder_speed = 0.0;
  float left_encoder_speed = 0.0;
  float alpha_speed = 0.0;

  right_encoder_speed = float(total_ech_r / (float(speed_ech) * 15 / 1000));
  left_encoder_speed = float(total_ech_l / (float(speed_ech) * 15 / 1000));

  alpha_speed = (float(total_ech_r - total_ech_l)) / (float(speed_ech) * 15 / 1000);


  currentvelocityRight = (right_encoder_speed + left_encoder_speed) / 2 + alpha_speed * wheel_radius / 2;
  currentvelocityLeft = (right_encoder_speed + left_encoder_speed) / 2 - alpha_speed * wheel_radius / 2;

  total_ech_l = 0;
  total_ech_r = 0;
}

void updateOdometrie() {

  
  t++;
  
  long deltaLeftCount = encoderLeftCount - lastEncoderLeftCount;
  long deltaRightCount = encoderRightCount - lastEncoderRightCount;

  // n updaytiwhee bch matarje3ch li zero fi kol loop
  lastEncoderLeftCount = encoderLeftCount;
  lastEncoderRightCount = encoderRightCount;

  dS = calculDistance(deltaLeftCount, deltaRightCount);  // average mte3 position

  totalL += dsL;
  totalR += dsR;
  dS_total += dS;

  total_ech_l += dsL;
  total_ech_r += dsR;

  dTheta = (dsR - dsL) / entreaxe;  //average me3 orientation
  theta += dTheta;

  if (t % speed_ech == 0) {

    speed_calcul();
    total_ech_l = 0;
    total_ech_r = 0;
  }

  dsR = 0;
  dsL = 0;
}
//FIN ODOMETRY-----------------------------------------------------------------------------

//MOVE FUNCTIONS--------------------------------------------------------------------
#define motorCorr 0.935

#define PWM_MAX 250
#define PWM_MIN 100
//speeed
int PWM_L, PWM_R;
float accelVal = 5.0f;   // cm/s^2, adjust per robot
int sens = 1;


float profileDistance = 0.0;
int targetSpeed=0;
float acceleration = 5.0;
bool isTriangular = false;
float accelDistance = 0.0;
float decelDistance = 0.0;
float cruiseDistance = 0.0;
int peakSpeed = 0;

//erreur parametre
float i_right_erreur= 0, i_left_erreur= 0 , right_erreur= 0 ,left_erreur = 0;
float position_erreur = 0, orientation_erreur = 0;
float kTheta = 1.0;
float kir=0.0;
float kpr = 0.1;
float kil = 0.0;
float kpl = 0.1;

void setMotionProfile(float dist, float vmax, float accel){
  profileDistance = fabs(dist);
  targetSpeed = vmax;
  acceleration = accel;
}
void prepareMotionProfile(){
  
  float acc_dist = ((float)targetSpeed *(float) targetSpeed) / (2.0f * acceleration *100.0f);
 // Serial.print( acc_dist);
    if (2.0f * acc_dist >= profileDistance) {
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
        Serial.println(accelDistance);
        Serial.println(cruiseDistance);

    }
}

float getProfileSpeed(float traveled) {
    float v;
    if (!isTriangular) {
        if (traveled < accelDistance) v = sqrtf(2.0f * acceleration * traveled);
        else if (traveled < accelDistance + cruiseDistance){
          v = peakSpeed *100;
          }
        else {
            float decelTraveled = traveled - (accelDistance + cruiseDistance);
            v = sqrtf(fmaxf(0.0f, peakSpeed * peakSpeed - 2.0f * acceleration * decelTraveled))*100;
           
        }
    } else {
        if (traveled < accelDistance) v = sqrtf(2.0f * acceleration * traveled)*100;
    
        else {
            float decelTraveled = traveled - accelDistance;
            v = sqrtf(fmaxf(0.0f, peakSpeed * peakSpeed - 2.0f * acceleration * decelTraveled))*100;
        }
        Serial.print("v= ");Serial.print(v);
    }
    return v;
}
void run() {
    if (PWM_R > 0) { analogWrite(IN1, PWM_R    ); analogWrite(IN2, 0); }
    else { analogWrite(IN1, 0); analogWrite(IN2, -PWM_R ); }

    if (PWM_L > 0) { analogWrite(IN3, PWM_L); analogWrite(IN4, 0); }
    else { analogWrite(IN3, 0); analogWrite(IN4, -PWM_L); }
}
void resetControllers() {
    totalR = totalL = dS_total = 0;
    i_right_erreur = i_left_erreur = right_erreur = left_erreur = 0;
    position_erreur = orientation_erreur = 0;
    
}
float clamp(float PWM, float min, float max) {
    if (PWM < min) return min;
    if (PWM > max) return max;
    return PWM;
}
float RadToDeg(float radians) { return radians * (180.0f / PI); }
float DegToRad(float degrees) { return degrees * (PI / 180.0f); }
float getcurrentVelocity(float dist, float t) { return dist / t; }
float angleToDistance(float angleRad, float radius) { return radius * RadToDeg(angleRad); }
float ramp(int time) { return time * 0.6f; }
void stopmotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
float compute(float setpoint, float current,float& integral,float kpVal,float kiVal) {
        float Kp=kpVal;
        float dt=0.01f;
        float Ki=kiVal;
        float error = setpoint - current;
        float integralCandidate = integral + error * dt;
        float outputCandidate = Kp * error + Ki * integralCandidate;
        float output;
        if (outputCandidate > PWM_MAX) {
            output = PWM_MAX;
            if (error < 0) integral = integralCandidate; // anti-windup unwind
        }
        else if (outputCandidate < PWM_MIN ) {
            output =PWM_MIN ;
            if (error > 0) integral = integralCandidate; // anti-windup unwind
        }
        else {
            integral = integralCandidate;
            output = outputCandidate;
        }
        return output;
    }


void moveDistance(float distance, float speed) {
    resetControllers();

    
    setMotionProfile(distance, speed, accelVal);
    prepareMotionProfile();
    

    while (abs(dS_total - distance) > 5) {
        sens = (dS_total - distance < 0) ? 1 : -1;
        
        float current_speed = sens * getProfileSpeed(fabs(dS_total));
        PWM_R = compute(current_speed,currentvelocityRight,i_right_erreur,kpr,kir);
        PWM_L = compute(current_speed, currentvelocityLeft,i_left_erreur,kpl,kil);

        // Orientation correction
       /* float Theta_correction = kTheta * (totalR - totalL);
        PWM_R -= Theta_correction;
        PWM_L += Theta_correction;*/

        if (sens == 1) {
            PWM_R = clamp(PWM_R, PWM_MIN, PWM_MAX);
            PWM_L = clamp(PWM_L, PWM_MIN, PWM_MAX);
        } else {
            PWM_R = clamp(PWM_R, -PWM_MAX, -PWM_MIN);
            PWM_L = clamp(PWM_L, -PWM_MAX, -PWM_MIN);
        }

        run();

        Serial.print("Distance: "); Serial.print(dS_total);
        Serial.print(" / "); Serial.print(distance);
        Serial.print(" PWM_R: "); Serial.print(PWM_R);
        Serial.print(" PWM_L: "); Serial.println(PWM_L);
        delay(10);
    }

    stopmotors();
}


void setup() {
  
  
  Serial.begin(9600);

  // Set pin modes for the interrupt pins(mtaa les encodeurs)
  pinMode(interruptPinRA, INPUT_PULLUP);
  pinMode(interruptPinRB, INPUT_PULLUP);
  pinMode(interruptPinLA, INPUT_PULLUP);
  pinMode(interruptPinLB, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  


  // Attach interrupts to handle encoder signals
  attachInterrupt(digitalPinToInterrupt(interruptPinRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinLA), interruptL, CHANGE);

  // Initialisation du Timer
  Timer1.initialize(5000);  // 100ms (100000 microsecondes)
  Timer1.attachInterrupt(updateOdometrie);
  


  previousMillis = millis();

  moveDistance(100, 200);


}

void loop() {
  // put your main code here, to run repeatedly:

}
