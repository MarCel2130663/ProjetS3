#include <Arduino.h>
#include <LibS3GRO.h>

int MAGPIN = 2; //pin electroaimant
int POTPIN = 0; // pin potentiometre

uint8_t PWM_PIN_1 = 5;
uint8_t DIR_PIN_1 = 30;
uint8_t PWM_PIN_2 = 6;
uint8_t DIR_PIN_2 = 31;
ArduinoX motor1;
ArduinoX motor2;
float speed;


void setup() {
  Serial.begin(9600);

  pinMode(MAGPIN, OUTPUT);
  pinMode(POTPIN, INPUT);
  motor1.init();
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

  speed = 0.7;
  motor1.setMotorPWM(PWM_PIN_1, speed);
  Serial.println(speed);
  delay(1000);
  speed = 0.0;
  motor1.setMotorPWM(PWM_PIN_1, speed);
  Serial.println(speed);
  delay(1000);
}