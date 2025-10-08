// === TEST SIMPLE ENCODEURS ===
// Déclare les pins
#define ENCODER_R_A 3   // interruption droite A
#define ENCODER_R_B 2   // droite B
#define ENCODER_L_A 19  // interruption gauche A
#define ENCODER_L_B 18  // gauche B

// Compteurs (doivent être "volatile" car modifiés par interruptions)
volatile long encoderRightCount = 0;
volatile long encoderLeftCount  = 0;

// ISR pour encodeur droit
void encoderRightISR() {
  if (digitalRead(ENCODER_R_A) == digitalRead(ENCODER_R_B)) {
    encoderRightCount--;
  } else {
    encoderRightCount++;
  }
}

// ISR pour encodeur gauche
void encoderLeftISR() {
  if (digitalRead(ENCODER_L_A) == digitalRead(ENCODER_L_B)) {
    encoderLeftCount++;
  } else {
    encoderLeftCount--;
  }
}

void setup() {
  Serial.begin(9600);

  // Configurer les pins
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);

  // Attacher les interruptions sur les canaux A
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderRightISR, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLeftISR, CHANGE);

  Serial.println("=== Test encodeurs prêt ===");
}

void loop() {
  Serial.print("RA="); Serial.print(digitalRead(ENCODER_R_A));
  Serial.print(" RB="); Serial.print(digitalRead(ENCODER_R_B));
  Serial.print(" | LA="); Serial.print(digitalRead(ENCODER_L_A));
  Serial.print(" LB="); Serial.println(digitalRead(ENCODER_L_B));
  delay(100);
}


