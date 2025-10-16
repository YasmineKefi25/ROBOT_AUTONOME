#include <TimerOne.h>
//set pins for the motors
#define IN1 4  // Pin pour IN1 du moteur 1 LEFT
#define IN2 5  // Pin pour IN2 du moteur 1
#define IN3 6  // Pin pour IN3 du moteur 2 RIGHT
#define IN4 7  // Pin pour IN4 du moteur 2
//set pins for the encoders
    //right
#define interruptPinRA 18//19//18
#define interruptPinRB 19//18//19
    //left
#define interruptPinLB 2//3
#define interruptPinLA 3//3//2


#define PI 3.14159265358979323846
//robot specification cm
#define nb_ticks 800
#define wheel_radius 4
#define entreaxe  33.5
float previousMillis=0;


//ODOMETRY---------------------------------------------------------------------
long encoderLeftCount = 0;
long encoderRightCount = 0;
float currentvelocityRight;
float currentvelocityLeft;
float x = 0.0;
float y = 0.0;
long t = 0;
int speed_ech = 10;
float total_ech_l = 0;
float total_ech_r = 0;
long lastEncoderLeftCount = 0;
long lastEncoderRightCount = 0;

float dS_total = 0;
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
    currentvelocityRight=0.0;
    currentvelocityLeft=0.0;
  
  
    // Step 1: Compute linear velocity of the robot (average of both wheels)
    float linear_speed = (total_ech_r + total_ech_l) / (2.0 * speed_ech * 15 / 1000.0); // cm/s

    // Step 2: Compute angular velocity of the robot (difference of wheel distances / wheelbase)
    float alpha_speed = (total_ech_r - total_ech_l) / (entreaxe * speed_ech * 15 / 1000.0); // rad/s

    // Step 3: Compute individual wheel speeds
    currentvelocityRight = linear_speed + alpha_speed * (entreaxe / 2.0); // right wheel speed in cm/s
    currentvelocityLeft  = linear_speed - alpha_speed * (entreaxe / 2.0); // left wheel speed in cm/s

    // Step 4: Reset accumulated distances for the next calculation
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

  dS = calculDistance(deltaLeftCount, deltaRightCount);  // kadeh mchee distance

  totalL += dsL;
  totalR += dsR;
  dS_total += dS;

  total_ech_l += dsL;
  total_ech_r += dsR;

  dTheta = (dsR - dsL) / entreaxe;  // kadeh t3awej 
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
#define motorCorr 0.935//0.935

#define PWM_MAX 200
#define PWM_MIN 100

#define PWM_MAX_DOURA 200
#define PWM_MIN_DOURA 100
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
float kTheta = 12; //!!!!!!

//forward
float kir_f = 0.1;// 0.04
float kpr_f = 1;// 1.0
float kil_f = 0.1;// 0.087
float kpl_f = 1;//0.95



//backwards
float kir_b = 0.025;// 0.025//0.6
float kpr_b = 1.0;// 1.0
float kil_b = 0.049;// 0.049//0.087
float kpl_b = 0.95;//0.95

float kposition = 3.0;//1.2
//rotate///////////////////////////////////////////////////
float kir_rp = 0.01;// 0.025
float kpr_rp = 0.71;// 1.0 //057
float kil_rp = 0.01;// 0.049
float kpl_rp = 0.9;//0.95//0.3

float kir_rn = 0.01;// 0.025
float kpr_rn = 0.759;// 1.0
float kil_rn = 0.01;// 0.049
float kpl_rn = 0.930111;//0.95//0.3

float kir;
float kpr;
float kil;
float kpl;


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
        peakSpeed = sqrtf(acceleration * profileDistance *100);
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

float getProfileSpeed(float traveled) {
    float v;
    //Serial.print("peak :    ");Serial.println(peakSpeed);
    if (!isTriangular) {
        if (traveled < accelDistance) v = (traveled * targetSpeed)/accelDistance ;
        // v = sqrtf(2.0f * acceleration * traveled) *100;
        else if (traveled < accelDistance + cruiseDistance){
          v = targetSpeed ;
          }
          
        else {
            //float decelTraveled = traveled - (accelDistance + cruiseDistance);
             float decelTraveled = traveled - (accelDistance + cruiseDistance);
            float remaining = accelDistance - decelTraveled;
            v = (remaining * targetSpeed) / accelDistance;
            //v = (traveled * targetSpeed)/(accelDistance + cruiseDistance);
            //v = sqrtf(fmaxf(0.0f, peakSpeed * peakSpeed - 2.0f * acceleration * decelTraveled))*100;

        }
    } else {
        if (traveled < accelDistance) v = (traveled * peakSpeed)/accelDistance;

        else {
             float decelTraveled = traveled - (accelDistance + cruiseDistance);
            float remaining = accelDistance - decelTraveled;
            v = (remaining * peakSpeed) / accelDistance;
        }
      //  Serial.print("v= "); Serial.print(v);
    }
    return fmax(v,100.0);
}
void run() {
    if (PWM_R > 0) { analogWrite(IN1, PWM_R    ); analogWrite(IN2, 0); }
    else { analogWrite(IN1, 0); analogWrite(IN2, -PWM_R  ); }

    if (PWM_L > 0) { analogWrite(IN3, PWM_L); analogWrite(IN4, 0); }
    else { analogWrite(IN3, 0); analogWrite(IN4, -PWM_L); }
}
void resetControllers() {
    totalR = totalL = dS_total =theta=  0;
    i_right_erreur = i_left_erreur = right_erreur = left_erreur = 0;
    position_erreur = orientation_erreur = 0;

}
float clamp(float PWM, float min, float max) {
    if (PWM < min) return min;
    if (PWM > max) return max;
    return PWM;
}

float getcurrentVelocity(float dist, float t) { return dist / t; }

void stopmotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
float compute(float setpoint, float current,float& integral,float kpVal,float kiVal) {
        float Kp=kpVal;
        float dt=0.05f;
        float Ki=kiVal;
        float error = setpoint - current;
        float integralCandidate = integral + error * dt;
        float outputCandidate = Kp * error + Ki * integralCandidate;
     //   Serial.print("output condidate"); Serial.println(outputCandidate);
        float output;
        if (abs(outputCandidate) > PWM_MAX) {
            output = sens * PWM_MAX;
            if (error < 0) integral = integralCandidate; // anti-windup unwind
        }
        else if (abs(outputCandidate) < PWM_MIN ) {
            output =sens *PWM_MIN ;
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
    if(distance >= 0){
      kir = kir_f;
      kpr = kpr_f;
      kil = kil_f;
      kpl = kpl_f;
    }else{
      kir = kir_b;
      kpr = kpr_b;
      kil = kil_b;
      kpl = kpl_b;
    }


    setMotionProfile(distance, speed, accelVal);
    prepareMotionProfile();


    while (abs(dS_total - distance) > 2) {
        sens = (dS_total - distance < 0) ? 1 : -1;

        float current_speed = sens * getProfileSpeed(fabs(dS_total));
        
        Serial.print(current_speed);
        Serial.print("\t");
        Serial.print(currentvelocityRight);
        Serial.print("\t");
        Serial.println(currentvelocityLeft);

        PWM_R = compute(current_speed,currentvelocityRight,i_right_erreur,kpr,kir);
        
        PWM_L = compute(current_speed, currentvelocityLeft,i_left_erreur,kpl,kil);

        // Orientation correction
        float Theta_correction = kTheta * (totalR - totalL);
        PWM_R -= Theta_correction;
        PWM_L += Theta_correction;

        if (sens == 1) {
            PWM_R = clamp(PWM_R, PWM_MIN, PWM_MAX);
            PWM_L = clamp(PWM_L, PWM_MIN, PWM_MAX);
        } else {
            //Serial.println("here");
            PWM_R = clamp(PWM_R, -PWM_MAX, -PWM_MIN);
            PWM_L = clamp(PWM_L, -PWM_MAX, -PWM_MIN);
        }

        run();

        delay(10);
    
    }

    stopmotors();
}








void dour(float angle, float speed){
 if(angle>=0){
    kir = kir_rp;
    kpr = kpr_rp;
    kil = kil_rp;
    kpl = kpl_rp;
  }else{
    kir = kir_rn;
    kpr = kpr_rn;
    kil = kil_rn;
    kpl = kpl_rn;
  }

    sens = (angle >= 0) ? 1 : -1;
    resetControllers(); 
    float distance = angle * PI * entreaxe /( 180.0); 
    setMotionProfile(distance, speed, accelVal); 
    prepareMotionProfile();
  while (abs(abs(theta * 180.0 / PI) - abs(angle)) > 3.0) {

   
    float remaining = abs(angle) - abs(theta * 180.0 / PI);
    // Serial.print("remaining : ");Serial.println(remaining);
    int dir = (remaining >= 0) ? 1 : -1;
    //Serial.print("sens : ");Serial.println(sens);Serial.print("    ");Serial.println(dir);
    //Serial.print("   dir ");Serial.println(dir);
    float current_speed = getProfileSpeed(abs(theta) * entreaxe);
   /* Serial.print("distance : ");Serial.print(abs(totalR) + abs(totalL));Serial.print("  / ");Serial.print(distance);*/
 //   Serial.print("   current speed : ");Serial.println(current_speed);
    PWM_R = computedoura(dir*sens * current_speed, currentvelocityRight, i_right_erreur, kpr, kir);
    PWM_L = computedoura(-dir*sens * current_speed, currentvelocityLeft, i_left_erreur, kpl, kil);
   // Serial.print(" PWM right ");Serial.print(PWM_R); Serial.print("             PWM left ");Serial.println(PWM_L);
///////////////////////////////////////////////////////////////////////
  /* float avg_abs = (abs(totalR) + abs(totalL)) / 2.0;

    float pos_corr_R = kposition * (avg_abs - abs(totalR));
    float pos_corr_L = kposition * (avg_abs - abs(totalL));

    PWM_R += dir * pos_corr_R;  // boost if lagging, reduce if overshoot
    PWM_L += dir *pos_corr_L;  // same*/
   // Serial.print(" distance right ");Serial.print(totalR); Serial.print("             distance left ");Serial.println(totalL);
        //Serial.print(" corr right ");Serial.print(pos_corr_R); Serial.print("             corr left ");Serial.println(pos_corr_L);

////////////////////////////////////////////////////////////
       float corr_R = kposition * (abs(totalL) - abs(totalR)); // boost/slow based on other wheel
        float corr_L = kposition * (abs(totalR) - abs(totalL));

        PWM_R += dir*sens *corr_R;
        PWM_L += -dir*sens* corr_L;

    //  Serial.print(" PWM right ");Serial.print(PWM_R); Serial.print("             PWM left ");Serial.println(PWM_L);

    ////////////////////////////////////////////////////////// Clamping based on direction
  /*  if (dir == 1) {
      if(sens = 1){
        PWM_R = clamp(PWM_R, PWM_MIN_DOURA, PWM_MAX_DOURA);
        PWM_L = clamp(PWM_L, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
      }
      else{
        // Clockwise rotation: Right wheel forward, Left wheel backward
        PWM_L = clamp(PWM_L, PWM_MIN_DOURA, PWM_MAX_DOURA);
        PWM_R = clamp(PWM_R, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
      }
    } else {
        // Counter-clockwise rotation: Right wheel backward, Left wheel forward  
        if(sens = -1){
        PWM_R = clamp(PWM_R, PWM_MIN_DOURA, PWM_MAX_DOURA);
        PWM_L = clamp(PWM_L, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
      }
      else{
        // Clockwise rotation: Right wheel forward, Left wheel backward
        PWM_L = clamp(PWM_L, PWM_MIN_DOURA, PWM_MAX_DOURA);
        PWM_R = clamp(PWM_R, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
      }
    }
    */
    if(dir* sens == 1){
      PWM_R = clamp(PWM_R, PWM_MIN_DOURA, PWM_MAX_DOURA);
        PWM_L = clamp(PWM_L, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
    }
    else{
      PWM_L = clamp(PWM_L, PWM_MIN_DOURA, PWM_MAX_DOURA);
        PWM_R = clamp(PWM_R, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
    }
    /*
    PWM_R = clamp(PWM_R, -PWM_MAX_DOURA, PWM_MAX_DOURA);
    PWM_L = clamp(PWM_L, -PWM_MAX_DOURA, PWM_MAX_DOURA);

    // Apply deadzone for minimum power
    if (PWM_R > 0 && PWM_R < PWM_MIN_DOURA) {
        PWM_R = PWM_MIN_DOURA;
    } else if (PWM_R < 0 && PWM_R > -PWM_MIN_DOURA) {
        PWM_R = -PWM_MIN_DOURA;
    }
    

    if (PWM_L > 0 && PWM_L < PWM_MIN_DOURA) {
        PWM_L = PWM_MIN_DOURA;
    } else if (PWM_L < 0 && PWM_L > -PWM_MIN_DOURA) {
        PWM_L = -PWM_MIN_DOURA;
    }*/
   /* Serial.print(" PWM right ");Serial.print(PWM_R); Serial.print("             PWM left ");Serial.println(PWM_L);*/
     Serial.print("angle : ");Serial.print((theta * 180.0 / PI));Serial.print("  / ");Serial.println(angle);
    
    run();
   delay(10);
}

    stopmotors();
}

/*void dour_bor(float angle, float speed)
{
    resetControllers(); 
    float distance = angle * PI * entreaxe /( 180.0); 
    setMotionProfile(distance, speed, accelVal); 
    prepareMotionProfile();

    while ((abs((theta*180)/PI - angle)>2.0))
    {
        if (((totalR - totalL) - distance) < 0)
        {
            sens = 1;
        }
        else
        {
          
            sens = -1;

        }
        
        float current_speed = sens * getProfileSpeed(abs(totalR - totalL));

        // right pid
        right_erreur = current_speed - currentvelocityRight;
        i_right_erreur += right_erreur;
        PWM_R = kp_dour * right_erreur;

        //  PWM_R=erreur(PWM_R,PWM_MIN,PWM_MAX);
        
        
        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN_DOURA, PWM_MAX_DOURA);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
        }

        



        // left pid
        left_erreur = -current_speed - currentvelocityLeft;
        i_left_erreur += left_erreur;
        PWM_L = kp_dour * left_erreur;
        // PWM_L=erreur(PWM_L,-PWM_MAX,-PWM_MIN);
        if (sens == 1)
        {
            PWM_L = erreur(PWM_L, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
        }
        else
        {
            PWM_L = erreur(PWM_L, PWM_MIN_DOURA, PWM_MAX_DOURA);
        }

        // position pid
        position_erreur = k_position * (totalR + totalL);
        Serial.print(position_erreur);
        Serial.print("     ");
        // Theta_correction=kTheta*orientation_erreur;

        PWM_R += position_erreur;
        PWM_L -= position_erreur;

        if (sens == 1)
        {
            PWM_L = erreur(PWM_L, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
        }
        else
        {
            PWM_L = erreur(PWM_L, PWM_MIN_DOURA, PWM_MAX_DOURA);
        }
        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN_DOURA, PWM_MAX_DOURA);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
        }
        float PWM_test = PWM_R;
        PWM_R = PWM_L;
        PWM_L = PWM_test;
        Serial.print(theta);
        Serial.println("     ");
        run();
    }
    stopmotors();
   
}*/


float computedoura(float setpoint, float current,float& integral,float kpVal,float kiVal) {
        float Kp=kpVal;
        float dt=0.05f;
        float Ki=kiVal;
        float error = setpoint - current;
        
        float integralCandidate = integral + error * dt;
       
        float outputCandidate = Kp * error + Ki * integralCandidate;
//Serial.print("output condidate : "); Serial.print(outputCandidate);
        float output;
        /*****************************if (abs(outputCandidate) > PWM_MAX) {
            output = PWM_MAX;
            if (error < 0) integral = integralCandidate; // anti-windup unwind
        }
        else if (abs(outputCandidate) < PWM_MIN ) {
            output =PWM_MIN ;
            if (error > 0) integral = integralCandidate; // anti-windup unwind
        }
        else {
            integral = integralCandidate;
            output = outputCandidate;
        }********************************/
              //  Serial.print("output :"); Serial.println(output);
        if (outputCandidate > PWM_MAX) {
            output = PWM_MAX;
            if (error < 0) integral = integralCandidate; // anti-windup unwind
        }
        else if (outputCandidate < -PWM_MAX) {
            output = -PWM_MAX;
            if (error > 0) integral = integralCandidate; // anti-windup unwind
        }
        else if (fabs(outputCandidate) < PWM_MIN) {
            // Optional: deadzone or minimum drive
            output = (outputCandidate > 0 ? PWM_MIN : -PWM_MIN);
            integral = integralCandidate;
        }
        else {
            integral = integralCandidate;
            output = outputCandidate;
        }
                return output;
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
  Timer1.initialize(5000);  // 5 ms
  Timer1.attachInterrupt(updateOdometrie);



  previousMillis = millis();
  //Serial.print("lll");
  //delay(4000);
  /*dour(-180, 120);
  delay(1000);
  dour(180,120);*/
  moveDistance(70,120);
}

void loop() {
  // put your main code here, to run repeatedly:

}
