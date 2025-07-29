#include <Arduino.h>
#include <LibS3GRO.h>
#include <string.h>
#include "PIDhihi.h"

//UI
bool START = true; // utilisateur

//PINS
int POTPIN = 11; //potentiometre
int MAGPIN1 = 8; //electroaimant
int MAGPIN2 = 9; //electroaimant
uint8_t PWM_PIN_1 = 5; //moteur arriere
uint8_t DIR_PIN_1 = 30; //moteur arriere
uint8_t PWM_PIN_2 = 6; //moteur avant
uint8_t DIR_PIN_2 = 31; //moteur avant
uint8_t PIN_CH1 = 2; //encodeur
uint8_t PIN_CH2 = 3; //encodeur

// MOTEURS
float speed;
int REAR = 0;
int FRONT = 1;

// PID
float kp = 0.001;
float ki = 0;
float kd = 0;
int currentPosition = 0;
float output;

// OBJETS
ArduinoX AX;
PIDhihi pid(kp, ki, kd);
//VexQuadEncoder encoder;

// POSITIONS CIBLES
int debut = 2700;
int obstacle = 5700;

// ANGLE CIBLE
int angleArriere = 730;

bool vibing;

float rouler(PIDhihi pid, float sp, float cp){
  output = pid.calculate(sp, cp);
  float speed = constrain(output, -0.55, 0.55);
  AX.setMotorPWM(FRONT, speed);
  AX.setMotorPWM(REAR, speed);
  return cp;
}

void arreter(){
  AX.setMotorPWM(FRONT, 0);
  AX.setMotorPWM(REAR, 0);
}

void setup() {
  Serial.begin(9600);

  vibing = true;
  pinMode(POTPIN, INPUT);
  pinMode(MAGPIN1, OUTPUT);
  pinMode(MAGPIN2, OUTPUT);
  AX.init();
  AX.resetEncoder(REAR);
}

void loop() {
  Serial.println("Debut loop");
  AX.resetEncoder(REAR);
  //activer electroaimant
  digitalWrite(MAGPIN1, HIGH);
  digitalWrite(MAGPIN2, HIGH);
  // mettre sapin (a enlever quand on a le ui)
  delay(9000);
  START = true;

  while(START){
    Serial.println("Debut START");
    vibing = true;
    // avancer la premiere fois jusqua obstacle1
    while(currentPosition < obstacle){
      currentPosition = rouler(pid, obstacle, AX.readEncoder(REAR));
    }
    arreter();
    delay(100);

    while(vibing){
      // reculer jusqua debut
      Serial.println("Debut vibing");
      while(currentPosition > debut && vibing){
        currentPosition = rouler(pid, debut, AX.readEncoder(REAR));
        if(analogRead(POTPIN) <= angleArriere && vibing){
          // desactiver electroaimant
          Serial.println("Angle atteint");
          while(currentPosition < obstacle && vibing){
            currentPosition = rouler(pid, obstacle, AX.readEncoder(REAR));
            Serial.println("Avance une derniere fois");
            delay(700);
            Serial.println("Lacher sapin");
            digitalWrite(MAGPIN1, LOW);
            digitalWrite(MAGPIN2, LOW);
            delay(100);
            Serial.println("Vibing false");
            vibing = false;
            START = false;
          }
        }
      }
      while(currentPosition < obstacle && vibing){
        currentPosition = rouler(pid, obstacle, AX.readEncoder(REAR));
      }
      arreter();
      delay(100);
    }
  }
  Serial.println("Retour au bout du rail");
  while(currentPosition > 0){
    currentPosition = rouler(pid, 0, AX.readEncoder(REAR));
  }
}