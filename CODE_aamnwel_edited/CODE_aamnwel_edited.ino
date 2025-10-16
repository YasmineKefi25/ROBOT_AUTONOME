bool code1 = true;
bool code2 = false;

#include <TimerOne.h>
//set pins for the motors
#define IN1 4  // Pin pour IN1 du moteur 1 RIGHT
#define IN2 5  // Pin pour IN2 du moteur 1
#define IN3 6  // Pin pour IN3 du moteur 2 LEFT
#define IN4 7  // Pin pour IN4 du moteur 2


//right
#define interruptPinRA 19  
#define interruptPinRB 18
//left
#define interruptPinLA 3 
#define interruptPinLB 2  
 

// tick / cm
#define tickcmR 58.6  //bch nbadlouhom
#define tickcmL 58.9

//tick / rad horaire
#define tickZR_P 882
#define tickZL_N 886

//tick / rad trigo
#define tickZL_P 887
#define tickZR_N 882
//#define currentvelocityLeft;
//#define currentvelocityRight;

// parametre K
/*#define KRap 1.3
#define KRai 0.4

#define Kap 1
#define Kai 0

#define kdp1 1
#define Kdp2 1
#define Kdi 0.2
*/
// vitesse max
#define maxSpeed 200
#define minSpeed 10
#define maxAcc 10
#define PI 3.14159265358979323846
// compte codeur

long prevCountR = 0;
long prevCountL = 0;
int nb_ticks = 800;
//float Ez=0;//erreur proportionnelle liée à l'angle
//float dT = 0;
//float IdT = 0;
//float zT = 0;
//float IzT = 0;
float cmdV = 0;
float dsR = 0;
float dsL = 0;
float dS;
float totalL = 0;
float totalR = 0;
float dTheta = 0.0;
float theta = 0.0;

float cmdPwmRight = 0;
float cmdPwmLeft = 0;
float wheel_radius = 40.4;//39.55;
float entreaxe = 334;


/*char cmd;
int cmdR;
int cmdL;
int prevCmdR;
int prevCmdL;
char streamChar[32] ;*/
int i;
int incomingByte = 0;  // for incoming serial data

unsigned long previousMillis;
unsigned long chrono;
long encoderLeftCount = 0;
long encoderRightCount = 0;
float currentvelocityRight = 0.0;
float currentvelocityLeft = 0.0;

long t = 0;
int speed_ech = 10;

float total_ech_l = 0;
float total_ech_r = 0;

long lastEncoderLeftCount = 0;
long lastEncoderRightCount = 0;

float dS_total = 0;


float kp_r = 0.09; //0.09
float kp_l=0.0715; //0.0715
float ki_r = 0.03;//0.03
float ki_l=0.03;//0.03

//float kTheta = 2;

float kp_dour = 0.001;
float k_position =0.5;


float PWM_R = 0;
float PWM_L = 0;

float right_erreur = 0;
float left_erreur = 0;
float i_right_erreur = 0;
float i_left_erreur = 0;

float orientation = 0;
float i_orientation = 0;
float orientation_erreur = 0;
float i_orientation_erreur = 0;
float Theta_correction = 0;
float position_erreur = 0;

int sens = 1;

float PWM_MIN = 125;
float PWM_MAX = 180;

float PWM_MIN_DOURA = 85;
float PWM_MAX_DOURA = 150;


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


float calculDistance(long deltaLeftCount, long deltaRightCount, float wheel_radius, int nb_ticks) {
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
  // float right_speed=0.0;
  // float left_speed=0.0;
  right_encoder_speed = float(total_ech_r / (float(speed_ech) * 15 / 1000));
  left_encoder_speed = float(total_ech_l / (float(speed_ech) * 15 / 1000));

  alpha_speed = (float(total_ech_r - total_ech_l)) / (float(speed_ech) * 15 / 1000);


  currentvelocityRight = (right_encoder_speed + left_encoder_speed) / 2 + alpha_speed * wheel_radius / 2;
  currentvelocityLeft = (right_encoder_speed + left_encoder_speed) / 2 - alpha_speed * wheel_radius / 2;

  total_ech_l = 0;
  total_ech_r = 0;
}


void updateOdometrie() {

  //int nb_ticks=400;
  t++;

  float currentvelocityRight = 0;
  float currentvelocityLeft = 0;


  float x = 0.0;
  float y = 0.0;




  // 3dad pulses fi kol update
  long deltaLeftCount = encoderLeftCount - lastEncoderLeftCount;
  long deltaRightCount = encoderRightCount - lastEncoderRightCount;

  // n updaytiwhee bch matarje3ch li zero fi kol loop
  lastEncoderLeftCount = encoderLeftCount;
  lastEncoderRightCount = encoderRightCount;

  dS = calculDistance(deltaLeftCount, deltaRightCount, wheel_radius, nb_ticks);  // average mte3 position

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
}

//  currentvelocityRight=dsR/(15*1000);//convertiha lel milliseconds
//  currentvelocityLeft=dsL/(15000);

//    // ntal3ou beha position w angle d'orientation jdida
//     x += dS * cos(theta + dTheta / 2.0);
//     y += dS * sin(theta + dTheta / 2.0);
//theta += dTheta;//

//     // nsal7ou beha angle bch nkhaliwhee entre pi et -pi
//     if (theta > PI) {
//       theta -= 2.0 * PI;
//     } else if (theta < -PI) {
//       theta += 2.0 * PI;
//     }

//     /*Serial.print("x: ");
//     Serial.print(x);
//     Serial.print(" m, y: ");
//     Serial.print(y);
//     Serial.print(" m, theta: ");
//     Serial.print(theta * 180.0 / PI);
//     Serial.println(" degrees");*/
//   }
//   lastTime = currentTime;
//  }
void setup() {
  // Initialize serial communication for debugging
  // Serial.begin(115200); //115200 est une valeur couramment utilisée pour le débogage avec l'ESP32.
  // while (!Serial);
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
  pinMode(8, OUTPUT);


  // Attach interrupts to handle encoder signals
  attachInterrupt(digitalPinToInterrupt(interruptPinRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinLA), interruptL, CHANGE);
  //chabeb fazet timerOne mch sur mnha rw kali aaliha marwen
  // Initialisation du Timer
  Timer1.initialize(5000);  // 100ms (100000 microsecondes)
  Timer1.attachInterrupt(updateOdometrie);
  // Set a time reference for your main loop

  // digitalWrite(8,HIGH);
  previousMillis = millis();
    /*analogWrite(IN3, 100);
    analogWrite(IN4, 0);*/
    moveDistance(400,150);
    doura(180,150);
    //delay(2000);
    //moveDistance(-500,150);
    //dour(180,50,false);

 

}

//fonction pour arreter les moteurs
void stopmotors() {
  digitalWrite(IN1, LOW);  //moteur 1
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);  //moteur 2
  digitalWrite(IN4, LOW);
}

float RadToDeg(float radians) {
  return radians * (180.0 / PI);
}
float DegToRad(float degrees) {
  return degrees * (PI / 180.0);
}

float ramp(int time) {
  return time * 0.6;  //constante
}


void applyMotorCommand(float cmdPwmRight, float cmdPwmLeft) {
  //if(cmdPwmRight>=0){
  digitalWrite(IN1, cmdPwmRight);
  digitalWrite(IN2, cmdPwmRight);
  // analogWrite(cmdPwmRight,abs(cmdPwm));

  ////else if(cmdPwmLeft>=0){
  digitalWrite(IN3, cmdPwmLeft);
  digitalWrite(IN4, cmdPwmLeft);
  //}else{
  // digitalWrite(IN3,LOW);
  //digitalWrite(IN4,LOW);
}
//analogWrite(cmdPwmLeft,abs(cmdPwm));


float angleToDistance(float angleRad, float radius) {
  float angleDeg = RadToDeg(angleRad);
  float distance = radius * angleDeg;

  return distance;
}

int constraint(float a, int min, int max) {
  if (a < min) {
    return min;
  } else if (a > max) {
    return max;
  }
  return a;
}

float getcurrentVelocity(float dist, float t) {
  return dist / t;
}

// int targeTtest(int targetdistance)
// { if (((abs(targetdistance))/(targetdistance))==1)
// return(1);
// else
// return(-1);}



void run() {

  if (PWM_R > 0) {
    // PWM_R=erreur(PWM_R,PWM_MIN,PWM_MAX);
    analogWrite(IN1, PWM_R);
    analogWrite(IN2, 0);
  } else {
    // PWM_R=erreur(PWM_R,PWM_MIN,PWM_MAX);
    analogWrite(IN1, 0);
    analogWrite(IN2, -PWM_R);
  }
  if (PWM_L > 0) {
    // PWM_L=erreur(PWM_L,PWM_MIN,PWM_MAX);

    analogWrite(IN3, PWM_L);
    analogWrite(IN4, 0);
  } else {
    // PWM_L=erreur(PWM_L,-PWM_MAX,-PWM_MIN);

    analogWrite(IN3, 0);
    analogWrite(IN4, -PWM_L);
  }
}

float erreur(float PWM, float min, float max) {
  if (PWM < min) {
    PWM = min;
  } else if (PWM > max) {
    PWM = max;
  }
  return PWM;
}
float acceleration(float speed, float distance, float accel, float decel) {
  float current_speed;

  if (abs(dS_total) < accel) {
    current_speed = (speed / (accel)) * abs(dS_total);
  } else if (distance - abs(dS_total) < decel) {
    current_speed = (speed / -decel) * abs(dS_total) + speed - ((distance - decel) * (speed / -decel));
  } else {
    current_speed = speed;
  }
  return current_speed;
}

void iniiit() {
  totalR = 0;
  totalL = 0;
  dS_total = 0;
  i_right_erreur = 0;
  i_left_erreur = 0;
  right_erreur = 0;
  left_erreur = 0;
  position_erreur = 0;
  orientation_erreur = 0;
}
float acceleration_dour(float speed, float distance, float accel, float decel)
{
    float current_speed;

    if ((totalR - totalL) < accel)
    {
        current_speed = (speed / (accel)) * (totalR - totalL);
    }
    else if (distance - (totalR - totalL) < decel)
    {
        current_speed = (speed / -decel) * (totalR - totalL) + speed - ((distance - decel) * (speed / -decel));
    }
    else
    {
        current_speed = speed;
    }

    current_speed = erreur(current_speed, PWM_MIN, speed);
    return current_speed;
}


void dour(float angle, float speed, bool stop )
{
    iniiit();

    float distance = angle * PI * entreaxe / 180;

    float accel = 0.25 * distance;
    float decel = 0.5 * distance;

    while ((abs((theta*180)/PI - angle)>2))
    {
        if (((totalR - totalL) - distance) < 0)
        {
            sens = 1;
        }
        else
        {
          if(stop){
            break;
          }
            sens = -1;

        }
        /*Serial.print(distance);
  Serial.print("   ");
        Serial.print(sens);
  Serial.print("   ");*/
        float current_speed = sens * acceleration_dour(speed, abs(distance), abs(accel), abs(decel));

        // right pid
        right_erreur = current_speed - currentvelocityRight;
        i_right_erreur += right_erreur;
        PWM_R = kp_dour * right_erreur;

        //  PWM_R=erreur(PWM_R,PWM_MIN,PWM_MAX);
        
        Serial.print(theta - angle);
        Serial.print("     ffff ");
        Serial.print(PWM_R);
        Serial.print("      ");
        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN_DOURA, PWM_MAX_DOURA);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
        }

        Serial.print(PWM_L);
        Serial.print("     ");
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
   
}


void moveDistance(float distance, float speed) {
  iniiit();
  float accel = 0.25 * distance;
  float decel = 0.5 * distance;

  while (abs(dS_total - distance) > 5) {
    if ((dS_total - distance) < 0)
      sens = 1;
    else
      sens = -1;

    float current_speed = sens * acceleration(speed, abs(distance), abs(accel), abs(decel));
    // right pid
    right_erreur = current_speed - currentvelocityRight;
    i_right_erreur += right_erreur;
    PWM_R = kp_r * right_erreur + ki_r * i_right_erreur;
    // Serial.print(PWM_R);
    // Serial.print("   BA3DHA HAJA   ");
    if (sens == 1) {
      PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
    } else {
      PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
    }
    // left pid
    left_erreur = current_speed - currentvelocityLeft;
    i_left_erreur += left_erreur;
    PWM_L = kp_l * left_erreur + ki_l * i_left_erreur;

    if (sens == 1) {
      PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
    } else {
      PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
    }
    // Serial.print(dS_total);
    // Serial.print("      ");
    
    /*Serial.print(right_erreur);
        Serial.print("    ");
        Serial.print(current_speed);
        Serial.print("   ");
        Serial.println(PWM_R);*/

    // orientation pid
    /*float orientation_erreur = totalR - totalL;
    float Theta_correction = kTheta * orientation_erreur;
    // Serial.print(Theta_correction);
    // Serial.println("      ");
    PWM_R -= Theta_correction;
    PWM_L += Theta_correction;*/
    if (sens == 1) {
      PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
    } else {
      PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
    }
    if (sens == 1) {
      PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
    } else {
      PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
    }
  
    /*Serial.print(PWM_R);
    Serial.print("      ");
    Serial.print(PWM_L);
    Serial.println("      ");*/
    run();
    Serial.print(current_speed);
    Serial.print("\t");
    Serial.print(currentvelocityRight);
    Serial.print("\t");
    Serial.println(currentvelocityLeft);
}
  stopmotors();
  Serial.print(dS_total);
}



/*void rotate(float targetAngle){

  unsigned long now,startTime,timevitesse;
  float moveDistanceRight=0;
  float moveDistanceLeft=0;



  float moveDistanceCenter=0;

  float accumulationError=0;
  float erroPrec=0;
  float error=0;
  float cmdPwmRight=0;
  float cmdPwmLeft=0;
  unsigned long elapsedTime=0;
  float targetdistance=angleToDistance(targetAngle);  
  startTime = millis();
  
  //float d=calculDistance(deltaLeftCount,deltaRightCount,wheel_radius,nb_ticks);//naseelou aaliha marweeeeeeeennnnnn
  while((moveDistanceCenter<(targetdistance-2))|| (moveDistanceCenter>(targetdistance+2))){
    now=millis(); 
    first=now;  
    error = targetdistance-moveDistanceCenter;
    //IdT+=dT;
    cmdV=Kdp*(error-erroPrec)+Kp1*error;//kp1 mtaa dist w kp2 mtaa l vitesse   
    erroPrec=error;
    cmdv=min(ramp(now-startTime),cmdv);//applique une rampe de montée 
    cmdv=constraint(cmdv,0,500);//limite la commande de vitesse 
    timevitesse=millis();///// malek : zit hedhi
    //float ds=calculDistance(deltaLeftCount,deltaRightCount,wheel_radius,nb_ticks);
    d+= ds;//accumule la distance parcourue
    //time=debut-now
    //calcul error  Error vitesse

    float errorvitesseRight=cmdv-currentvelocityRight;
    accumulationErrorRight+=errorVitesseRight;
    cmdDPwmRight=kp2*errorvitesseRight+kdi*accumulationErrorRight
    cmdDPwmRight=constraint(cmdDPwmRight,40,240);

    float errorvitesseLeft=cmdv-currentvelocityLeft;
    accumulationErrorLeft+=errorVitesseLeft;
    cmdDPwmLeft=kp2*errorvitesseRight+kdi*accumulationErrorLeft;
    cmdDPwmLeft=constraint(cmdDPwmLeft,40,240);

    applyMotorCommand(1,cmdPwmRight);
    applyMotorCommand(2,cmdPwmLeft);

    elapsedTime = millis() - now;
    delay(15-elapsedTime);// lazemha milli second

    }
  stopmotors();
}    */

void loop() {

  // reply only when you receive data:
  // if (Serial.available() > 0) {
  //   // read the incoming byte:
  //   incomingByte = Serial.read();
  //   streamChar[i]=(char)incomingByte;
  //   i++;
  // }

  // if (incomingByte == 10){
  //   incomingByte =0;
  //   decryptIncom();
  //   i = 0;
  // }
  Serial.print(encoderRightCount);
  Serial.print("        ");
  Serial.println(encoderLeftCount);
  
  //   moveDistance(500);
  // digitalWrite(IN1, 125);//moteur right
  //  digitalWrite(IN4, 0);
  //  digitalWrite(IN3, 125);//moteur 2
  //  digitalWrite(IN2, 0);

  //  Serial.print("   encoderLeftCount=");
  //  Serial.print(encoderLeftCount);
  // Serial.print("encoderRightCount=");
  // Serial.println(encoderRightCount);

  // Serial.println("deltaRightCount");
  // Serial.print(deltaRightCount);
  // Serial.print("Left Speed   ");
  // Serial.print(currentvelocityLeft);
  // Serial.print("Right Speed  ");
  // Serial.println(currentvelocityRight);

  // Serial.print("   ddddsLLLLL=");
  // Serial.print(dsL);
  // Serial.print("   ddddsRRRRRR=");
  // Serial.print(dsR);
  // Serial.print("   ddddssssss=   ");
  // Serial.println(dS);
  // Serial.print("  totalL ");
  // Serial.print(totalL);
  // Serial.print("  totalR ");
  // Serial.println(totalR);
  //Serial.println("     dist= 27 ");
  
  /*Serial.print("    deg    ");
Serial.print(RadToDeg(dTheta));
Serial.print("    rad    ");
Serial.print(DegToRad(dTheta));*/
  /*rotate(90);
Serial.print("         dTheta");
Serial.print(dTheta);*/
  //delay(1000);
}



//T le temps entre deux mises à jour des encodeurs
/*void ticks_to_distance(long ticks,float radius, int resolution,float precision){
}
//resolution nombres de ticks générés par l'encodeur pour un tour ocmplet de la roue
//preciison un facteur de correction pour ajuster la distance calculée
void update_position(void)
{  
  float totalR=0;
  float totalR=0;

  d_right=counterR-prevcountR;
  prevcountR=counterR;

	dR = ticks_to_distance(d_right,right_radius,right_resolution,right_precision);// d_right count - prevvount en 10 ms
	totalR += dR;
	//d_right_counter += dR;

  d_left=counterL-prevcountL;
  prevcountL=counterL;
	dL = ticks_to_distance(d_left,left_radius,left_resolution,left_precision);
	totalL += dL;

	//d_left_counter += dL;
	dC = (dR+dL)/2;
	total_centre+=dC;
	
	X += dC*cos(alpha);
	Y += dC*sin(alpha);
	alpha += ((dR-dL)/spacing_encoder);
	dalpha_counter += ((dR-dL)/spacing_encoder);
	while (alpha>PI)
	{
		alpha -= 2*PI;
	}
	while (alpha<-PI)
	{
		alpha += 2*PI;
	}
	alpha_deg = rad_to_deg(alpha);
}*/

//debut de chaque itération
//debut chaque iteration now=millis()
//calcul error  Error
//prevoius error
//mise à jour de l'intégrale des erreurs
//time=debut-now
//calcul error  Error vitesse
//tu repetes la meme logique pour le moteur 2
//vers la fin l while endIteration=millis() periode =15ms
// wait 15ms
//on plafone à la vitesse de commande


/*Initialisation du Timer
    Timer1.initialize(100000); // 100ms (100000 microsecondes)
    Timer1.attachInterrupt(updateOdometrie); // Appelle updateOdometrie() à chaque interruption    
    void updateOdometrie() {
    // Mettre à jour les compteurs des encodeur
    countR = digitalRead(interruptPinRA) ? countR + 1 : countR - 1;
    countL = digitalRead(interruptPinLA) ? countL + 1 : countL - 1;}*/



//estaaml bibliothéque TimerOne bch tconnecti code l odomtrie
//maa l code hetha kol 5ms secondes ynedilou wahou l code


//hot fi wesst boucle while
//aaml fonction mtaa capture d'ecran needilha
//aaml include
