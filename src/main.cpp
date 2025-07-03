#include <Arduino.h>
#include <LibS3GRO.h>
#include "PID.h"

int MAGPIN = 2; //pin electroaimant
int POTPIN = 0; // pin potentiometre

uint8_t PWM_PIN_1 = 5;
uint8_t DIR_PIN_1 = 30;
uint8_t PWM_PIN_2 = 6;
uint8_t DIR_PIN_2 = 31;
ArduinoX AX;
PID pid;
float speed;

//variables PID
float kp = 50;
float ki = 0;
float kd = 80;
float setPoint = 9600;
float currentPosition;
long lastTime = 0;
float output;


void setup() {
  Serial.begin(9600);

  pinMode(MAGPIN, OUTPUT);
  pinMode(POTPIN, INPUT);
  AX.init();
}

void loop() {
  //electroaimant
  /*digitalWrite(MAGPIN, HIGH);
  delay(10000);
  digitalWrite(MAGPIN, LOW);
  delay(1000);*/

  //potentiometre
  // int valeur = analogRead(POTPIN);
  // Serial.println(valeur);

  speed = 0.5;
  AX.setMotorPWM(PWM_PIN_1, speed);

  currentPosition = AX.readEncoder(0);
  rouler(setPoint, currentPosition);
}

void rouler(float sp, float cp){
  long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  output = pid.calculate(sp, cp, dt);
  int motorSpeed = ((constrain(abs(output), 0, 255) / 255.0) * 2.0) - 1.0;
  AX.setMotorPWM(PWM_PIN_1, motorSpeed);
}