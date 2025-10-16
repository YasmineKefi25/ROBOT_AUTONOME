#define ENC_L_A 3
#define ENC_L_B 2
#define ENC_R_A 19
#define ENC_R_B 18

volatile long leftCount = 0;
volatile long rightCount = 0;

void leftEncoder() {
  leftCount++;
}

void rightEncoder() {
  rightCount++;
}

void setup() {
  Serial.begin(9600);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoder, RISING);

  Serial.println("Rotate wheels by hand to test encoders...");
}

void loop() {
  Serial.print("Left encoder: ");
  Serial.print(leftCount);
  Serial.print("   Right encoder: ");
  Serial.println(rightCount);
  delay(500);
}
