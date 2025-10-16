#define IN1 4  // Left motor
#define IN2 5
#define IN3 6  // Right motor
#define IN4 7

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Motor test starting...");
  
  delay(2000);
  
  // --- Test LEFT motor ---
  Serial.println("Testing LEFT motor...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(2000);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(1000);
  
  // --- Test RIGHT motor ---
  Serial.println("Testing RIGHT motor...");
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(2000);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(1000);

  Serial.println("Test done!");
}

void loop() {}
