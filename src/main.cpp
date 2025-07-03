#include <Arduino.h>
#include <LibS3GRO.h>
#include "PIDurmom.h"

int MAGPIN = 2; //pin electroaimant
int POTPIN = 0; // pin potentiometre

int REAR = 0;
int FRONT = 1;

uint8_t PWM_PIN_1 = 5;
uint8_t DIR_PIN_1 = 30;
uint8_t PWM_PIN_2 = 6;
uint8_t DIR_PIN_2 = 31;
ArduinoX AX;
float speed;

//variables PID
float kp = 0.001;
float ki = 0;
float kd = 0;
int setPoint = 6000;
int currentPosition;
float output;

PIDurmom pid(kp, ki, kd);

void rouler(PIDurmom pid, float sp, float cp){
  output = pid.calculate(sp, cp);
  float speed = constrain(output, -1.0, 1.0);
  //Serial.println(speed);
  AX.setMotorPWM(REAR, speed);
}

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

  currentPosition = AX.readEncoder(REAR);
  Serial.println(currentPosition);
  rouler(pid, setPoint, currentPosition);

  if(currentPosition == setPoint){
    Serial.println("allo");
    delay(3000);
    AX.resetEncoder(0);
  }
}