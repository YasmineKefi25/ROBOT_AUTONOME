#include "Aerobotix_Arduino_nav.h"

// Create robot instance
Aerobotix_Arduino_nav robot;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Aerobotix Arduino Navigation Example");

  // === 1. Set pins if different from defaults ===
  robot.setIN1(4);
  robot.setIN2(5);
  robot.setIN3(6);
  robot.setIN4(7);

  robot.setInterruptPinRA(3);
  robot.setInterruptPinRB(2);
  robot.setInterruptPinLA(19);
  robot.setInterruptPinLB(18);

  // === 2. Configure parameters ===
  robot.setKp(0.12);
  robot.setKi(0.06);
  robot.setWheelRadius(3.3);   // cm
  robot.setEntreaxe(14.5);     // cm distance between wheels
  robot.setPWMMax(200);
  robot.setPWMMin(50);
  robot.setMaxSpeed(120);

  // Initialize the robot
  robot.begin();

  Serial.println("Robot initialized!");

  // === 3. Simple movement demo ===
  Serial.println("Move forward 40cm...");
  robot.moveDistance(40, 100);   // distance cm, speed
  Serial.println(robot.)

  Serial.println(robot.getCurrentVelocityRight());
  Serial.println(",");
  Serial.println(robot.getCurrentVelocityLeft());
  delay(1000);

  Serial.println("Rotate 90 degrees...");
  robot.rotate(90, 80);          // angle degrees, speed

  Serial.println("Example finished!");
}

void loop() {
  // Nothing here, just one-shot demo
}