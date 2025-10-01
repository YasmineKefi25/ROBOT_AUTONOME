#define encoder_L_A 19
#define encoder_L_B 3

volatile long nb_ticks_L = 0;

void ticks_L_A() {
  if (digitalRead(encoder_L_A) == digitalRead(encoder_L_B)) {
    nb_ticks_L++;
  } else {
    nb_ticks_L--;
  }
}

void ticks_L_B() {
  if (digitalRead(encoder_L_B) == digitalRead(encoder_L_A)) {
    nb_ticks_L--;
  } else {
    nb_ticks_L++;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(encoder_L_A, INPUT_PULLUP);
  pinMode(encoder_L_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_L_A), ticks_L_A, CHANGE);
 // attachInterrupt(digitalPinToInterrupt(encoder_L_B), ticks_L_B, CHANGE);
}

void loop() {
  static long last_value = 0;
  if (nb_ticks_L != last_value) {
    Serial.println(nb_ticks_L);
    last_value = nb_ticks_L;
  }
}
