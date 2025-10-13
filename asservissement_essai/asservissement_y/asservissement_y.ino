#include <TimerOne.h>
//set pins for the motors
#define IN1 4  // Pin pour IN1 du moteur 1 LEFT
#define IN2 5  // Pin pour IN2 du moteur 1
#define IN3 6  // Pin pour IN3 du moteur 2 RIGHT
#define IN4 7  // Pin pour IN4 du moteur 2
//set pins for the encoders
    //right
#define interruptPinRA 19
#define interruptPinRB 18
    //left
#define interruptPinLB 2
#define interruptPinLA 3


#define PI 3.14159265358979323846
//robot specification cm
#define nb_ticks 800
#define wheel_radius 4
#define entreaxe  33.5
float previousMillis=0;


//ODOMETRY---------------------------------------------------------------------
long encoderLeftCount = 0;
long encoderRightCount = 0;
float currentvelocityRight=0.0;
float currentvelocityLeft=0.0;
float x = 0.0;
float y = 0.0;
long t = 0;
int speed_ech = 5;
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
    
    // Step 1: Compute linear velocity of the robot (average of both wheels)
    float linear_speed = (total_ech_r + total_ech_l) / (2.0 * speed_ech * 10 / 1000.0); // cm/s

    // Step 2: Compute angular velocity of the robot (difference of wheel distances / wheelbase)
    float alpha_speed = (total_ech_r - total_ech_l) / (entreaxe * speed_ech * 10 / 1000.0); // rad/s

    // Step 3: Compute individual wheel speeds
    currentvelocityRight = linear_speed + alpha_speed * (entreaxe / 2.0); // right wheel speed in cm/s
    currentvelocityLeft  = linear_speed - alpha_speed * (entreaxe / 2.0); // left wheel speed in cm/s

    // Step 5: Reset accumulated distances for the next calculation
    total_ech_l = 0;
    total_ech_r = 0;
}

void updateOdometrie() {
  t++;

  long deltaLeftCount = encoderLeftCount - lastEncoderLeftCount;
  long deltaRightCount = encoderRightCount - lastEncoderRightCount;

  lastEncoderLeftCount = encoderLeftCount;
  lastEncoderRightCount = encoderRightCount;

  dS = calculDistance(deltaLeftCount, deltaRightCount);

  totalL += dsL;
  totalR += dsR;
  dS_total += dS;

  total_ech_l += dsL;
  total_ech_r += dsR;

  dTheta = (dsR - dsL) / entreaxe;
  theta += dTheta;

  if (t % speed_ech == 0) {
    speed_calcul();
    total_ech_l = 0;
    total_ech_r = 0;
  }

  dsR = 0;
  dsL = 0;
}

#define motorCorr 0.935

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
float kTheta = 5;

//forward
float kir_f = 0.0415;// 0.04
float kpr_f = 1.0;// 1.0
float kil_f = 0.087;// 0.087
float kpl_f = 0.95;//0.95

float kposition = 1.20;

//backwards
float kir_b = 0.6;// 0.025
float kpr_b = 1.0;// 1.0
float kil_b = 0.087;// 0.049
float kpl_b = 0.95;//0.95

float kir;// 0.025
float kpr;// 1.0
float kil;// 0.049
float kpl;//0.95


void setMotionProfile(float dist, float vmax, float accel){
  profileDistance = fabs(dist);
  targetSpeed = vmax;
  acceleration = accel;
}

void prepareMotionProfile() { //li bch taamlelna trapeze
    // Distance nécessaire pour atteindre la vitesse cible à l'accélération donnée
    float acc_dist = (targetSpeed * targetSpeed) / (2.0f * acceleration);

    if (2.0f * acc_dist >= profileDistance) {
        // Profil triangulaire : trop court pour atteindre targetSpeed
        isTriangular = true;
        peakSpeed = sqrt(acceleration * profileDistance); // vitesse maximale possible
        accelDistance = profileDistance / 2.0f;
        decelDistance = profileDistance / 2.0f;
        cruiseDistance = 0.0f;
    } else {
        // Profil trapézoïdal : accélération, vitesse constante, décélération
        isTriangular = false;
        peakSpeed = targetSpeed;
        accelDistance = acc_dist;
        decelDistance = acc_dist;
        cruiseDistance = profileDistance - 2.0f * acc_dist;
    }
}

float getProfileSpeed(float traveled) { //bch taatina consigne vitesse
    float v;

    if (!isTriangular) {
        // Profil trapézoïdal
        if (traveled < accelDistance) {
            // Accélération
            v = (traveled / accelDistance) * targetSpeed;
        } else if (traveled < accelDistance + cruiseDistance) {
            // Vitesse constante
            v = targetSpeed;
        } else {
            // Décélération
            float decelTraveled = traveled - (accelDistance + cruiseDistance);
            float remaining = accelDistance - decelTraveled;
            v = (remaining > 0) ? (remaining / accelDistance) * targetSpeed : 0.0f;
        }
    } else {
        // Profil triangulaire
        if (traveled < accelDistance) {
            // Accélération
            v = (traveled / accelDistance) * peakSpeed;
        } else {
            // Décélération
            float decelTraveled = traveled - accelDistance;
            float remaining = accelDistance - decelTraveled;
            v = (remaining > 0) ? (remaining / accelDistance) * peakSpeed : 0.0f;
        }
    }

    return v;
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
void stopmotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

float anti_windup(float setpoint, float current, float& integral, float kpVal, float kiVal) {
    float Kp = kpVal;
    float Ki = kiVal;
    float dt = 0.05f;  // Intervalle PID (s)
    
    // Calcul de l'erreur
    float error = setpoint - current;

    // Calcul de l'intégrale "proposée"
    float integralCandidate = integral + error * dt;

    // Sortie candidate PID (sans limitation)
    float outputCandidate = Kp * error + Ki * integralCandidate;

    // Limitation avec anti-windup
    float output;

    if (outputCandidate > PWM_MAX) {
        output = PWM_MAX;
        // Anti-windup : on n'intègre que si cela réduirait la sortie
        if (error < 0) integral = integralCandidate;
    } 
    else if (outputCandidate < PWM_MIN) {
        output = PWM_MIN;
        if (error > 0) integral = integralCandidate;
    } 
    else {
        // Pas de saturation : intégrale normale
        integral = integralCandidate;
        output = outputCandidate;
    }

    // Appliquer le sens (direction moteur)
    return sens * output;
}

float anti_windup_doura(float setpoint, float current, float& integral, float kpVal, float kiVal) {
    float Kp = kpVal;
    float Ki = kiVal;
    float dt = 0.05f;  // Intervalle PID (s)

    // Calcul de l'erreur
    float error = setpoint - current;

    // Calcul de l'intégrale "proposée"
    float integralCandidate = integral + error * dt;

    // Sortie candidate PID (sans limitation)
    float outputCandidate = Kp * error + Ki * integralCandidate;

    // Limitation avec anti-windup
    float output;

    if (outputCandidate > PWM_MAX) {
        output = PWM_MAX;
        // Anti-windup : on n'intègre que si cela réduirait la sortie
        if (error < 0) integral = integralCandidate;
    } 
    else if (outputCandidate < PWM_MIN) {
        output = PWM_MIN;
        if (error > 0) integral = integralCandidate;
    } 
    else {
        // Pas de saturation : intégrale normale
        integral = integralCandidate;
        output = outputCandidate;
    }

    // Debug : affiche la sortie PID
    Serial.print("output :"); Serial.println(output);

    // Appliquer le sens (direction moteur)
    return sens * output;
}

void moveDistance(float distance, float speed) {
    resetControllers();

    // Choix des gains selon le sens
    if(distance >= 0){
        kir = kir_f; kpr = kpr_f;
        kil = kil_f; kpl = kpl_f;
    } else {
        kir = kir_b; kpr = kpr_b;
        kil = kil_b; kpl = kpl_b;
    }

    // Préparer le profil de mouvement
    setMotionProfile(distance, speed, accelVal);
    prepareMotionProfile();

    while (abs(dS_total - distance) > 2.0) {  // Tolérance 2 cm
        sens = (dS_total - distance < 0) ? 1 : -1;

        // Vitesse théorique selon profil
        float current_speed = getProfileSpeed(fabs(dS_total));

        // PID pour chaque roue
        PWM_R = anti_windup(current_speed, currentvelocityRight, i_right_erreur, kpr, kir);
        PWM_L = anti_windup(current_speed, currentvelocityLeft, i_left_erreur, kpl, kil);

        // Correction orientation
        float Theta_correction = kTheta * (totalR - totalL);
        PWM_R -= Theta_correction;
        PWM_L += Theta_correction;

        // Clamp PWM
        PWM_R = clamp(PWM_R, PWM_MIN, PWM_MAX);
        PWM_L = clamp(PWM_L, PWM_MIN, PWM_MAX);

        run();
        delay(5);

    }

    stopmotors();
}

void dour(float angle, float speed) {
    resetControllers();

    // Distance que chaque roue doit parcourir pour tourner l'angle donné
    float distance = (angle * PI * entreaxe) / 360.0; // angle en degrés, entreaxe en cm

    // Définir les gains PID selon le sens de rotation
    if (angle >= 0) {
        kir = kir_b; kpr = kpr_b;
        kil = kil_f; kpl = kpl_f;
    } else {
        kir = kir_f; kpr = kpr_f;
        kil = kil_b; kpl = kpl_b;
    }

    // Préparer le profil de rotation
    setMotionProfile(distance, speed, accelVal);
    prepareMotionProfile();
    float targetTheta = angle * PI / 180.0;
    while (fabs(theta - targetTheta) > (4.0 * PI / 180.0)) { // Tolérance 4°
        sens = (theta < targetTheta) ? 1 : -1;

        float current_speed = getProfileSpeed(fabs(theta * PI / 180));

        // PID pour chaque roue : droite + gauche opposée
        PWM_R = anti_windup_doura(current_speed, currentvelocityRight, i_right_erreur, kpr, kir);
        PWM_L = anti_windup_doura(current_speed, currentvelocityLeft, i_left_erreur, kpl, kil);

        // Correction position pour rotation centrée
        float pos_corr = kposition * (totalR + totalL);
        PWM_R += pos_corr;
        PWM_L -= pos_corr;

        // Clamp PWM
        PWM_R = clamp(PWM_R, PWM_MIN_DOURA, PWM_MAX_DOURA);
        PWM_L = clamp(PWM_L, PWM_MIN_DOURA, PWM_MAX_DOURA);

        run();
        delay(5);

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
  Timer1.initialize(5000);  // 5 ms
  Timer1.attachInterrupt(updateOdometrie);



  previousMillis = millis();
  //Serial.print("lll");
  dour(180,160); // 
  //delay(4000);
  //moveDistance(100,150);

}

void loop() {
  // put your main code here, to run repeatedly:

}
