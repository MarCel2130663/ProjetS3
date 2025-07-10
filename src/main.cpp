#include <Arduino.h>
#include <LibS3GRO.h>
#include "PIDhihi.h"

//UI
bool START = true; // utilisateur

//PINS
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
VexQuadEncoder encoder;

// POSITIONS CIBLES
int debut = 0;
int obstacle1 = 9000;
int obstacle2 = 12000;
int fin = 15000;

// ANGLES CIBLES
int angleAller = -75;
int angleRetour = 75;
int angleDepot = 20;

float rouler(PIDhihi pid, float sp, float cp){
  output = pid.calculate(sp, cp);
  float speed = constrain(output, -0.8, 0.8);
  AX.setMotorPWM(FRONT, speed);
  AX.setMotorPWM(REAR, speed);
  return cp;
}


int ticksToDeg(int ticks) {return ticks * 4;}

void setup() {
  Serial.begin(9600);

  pinMode(MAGPIN1, OUTPUT);
  pinMode(MAGPIN2, OUTPUT);
  AX.init();
  encoder.init(PIN_CH1, PIN_CH2);
  attachInterrupt(encoder.getPinInt(), []{encoder.isr();}, FALLING);
  AX.resetEncoder(REAR);
}

void loop() {
  // mettre sapin
  delay(2000);

  // attendre ui
  if(START){
  //activer electroaimant
    digitalWrite(MAGPIN1, HIGH);
    digitalWrite(MAGPIN2, HIGH);

    // tant que angle > aller
    while(ticksToDeg(encoder.getCount()) > angleAller){
        // avancer jusqua obstacle1
        while(currentPosition < obstacle1){
          currentPosition = rouler(pid, obstacle1, AX.readEncoder(REAR));
        }
        delay(200);
        // reculer jusqua debut
        while(currentPosition > debut){
          currentPosition = rouler(pid, debut, AX.readEncoder(REAR));
        }
        delay(200);
    }

    // si angle <= aller
    if(ticksToDeg(encoder.getCount()) <= angleAller){
      // avancer at max speed jusqua fin
      while(currentPosition < fin){
        currentPosition = rouler(pid, fin, AX.readEncoder(REAR));
      }
      delay(200);
    }

    // si angle == depot
    if(ticksToDeg(encoder.getCount()) == angleDepot){
      //desactiver electroaimant
      digitalWrite(MAGPIN1, LOW);
      digitalWrite(MAGPIN2, LOW);
    }

    // tant que angle < angleRetour
    while(ticksToDeg(encoder.getCount()) < angleRetour){
      // reculer jusqua obstacle2
      while(currentPosition > obstacle2){
        currentPosition = rouler(pid, obstacle2, AX.readEncoder(REAR));
      }
      delay(200);
      // avancer jusqua fin
      while(currentPosition < fin){
        currentPosition = rouler(pid, fin, AX.readEncoder(REAR));
      }
      delay(200);
    }

    //si angle >= angleRetour
    if(ticksToDeg(encoder.getCount()) >= angleRetour){
      // reculer at max speed jusqua debut
      while(currentPosition > debut){
        currentPosition = rouler(pid, debut, AX.readEncoder(REAR));
      }
      delay(200);
    }
  }  
}