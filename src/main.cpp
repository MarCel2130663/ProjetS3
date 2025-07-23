#include <Arduino.h>
#include <LibS3GRO.h>
#include <string.h>
#include "PIDhihi.h"

//UI
bool START1 = false; // utilisateur
bool START2 = true; // yeet

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
int debut = 2500;
int obstacle1 = 5500;
int obstacle2 = 12000;
int fin = 15000;

// ANGLES CIBLES
int angleYeet = 810;
int angleArriere = 720;

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
  //activer electroaimant
  digitalWrite(MAGPIN1, HIGH);
  digitalWrite(MAGPIN2, HIGH);
  // mettre sapin (a enlever quand on a le ui)
  delay(4000);

  while(START2){
    vibing = true;
    // avancer la premiere fois jusqua obstacle1
    while(currentPosition < obstacle1){
      currentPosition = rouler(pid, obstacle1, AX.readEncoder(REAR));
    }
    arreter();
    delay(100);

    while(vibing){
      // reculer jusqua debut
      while(currentPosition > debut){
        currentPosition = rouler(pid, debut, AX.readEncoder(REAR));
        Serial.println(analogRead(POTPIN));
        if(analogRead(POTPIN) <= angleArriere){
          // desactiver electroaimant
          Serial.println("Angle atteint");
          // while(currentPosition < obstacle1){
          //   currentPosition = rouler(pid, obstacle1, AX.readEncoder(REAR));
          //   if(analogRead(POTPIN) >= angleArriere + 5){
              delay(700);
              digitalWrite(MAGPIN1, LOW);
              digitalWrite(MAGPIN2, LOW);
              delay(1000);
              START2 = false;
              vibing = false;
          //   }
          // }
        }
      }
      Serial.println("Retour au bout du rail");
      while(currentPosition < obstacle1){
        currentPosition = rouler(pid, obstacle1, AX.readEncoder(REAR));
      }
      arreter();
      delay(100);
    }
  }
  while(currentPosition < 0){
    currentPosition = rouler(pid, 0, AX.readEncoder(REAR));
  }
  // attendre ui
  // else if(START1){
  //   // tant que angle > aller
  //   while(digitalRead(POTPIN) > angleAller){
  //       // avancer jusqua obstacle1
  //       while(currentPosition < obstacle1){
  //         currentPosition = rouler(pid, obstacle1, AX.readEncoder(REAR));
  //       }
  //       delay(200);
  //       // reculer jusqua debut
  //       while(currentPosition > debut){
  //         currentPosition = rouler(pid, debut, AX.readEncoder(REAR));
  //       }
  //       delay(200);
  //   }

  //   // si angle <= aller
  //   if(lirePot(POTPIN) <= angleAller){
  //     // avancer at max speed jusqua fin
  //     while(currentPosition < fin){
  //       currentPosition = rouler(pid, fin, AX.readEncoder(REAR));
  //     }
  //     delay(200);
  //   }

  //   // si angle == depot
  //   if(lirePot(POTPIN) == angleDepot){
  //     //desactiver electroaimant
  //     digitalWrite(MAGPIN1, LOW);
  //     digitalWrite(MAGPIN2, LOW);
  //   }

  //   // tant que angle < angleRetour
  //   while(lirePot(POTPIN) < angleRetour){
  //     // reculer jusqua obstacle2
  //     while(currentPosition > obstacle2){
  //       currentPosition = rouler(pid, obstacle2, AX.readEncoder(REAR));
  //     }
  //     delay(200);
  //     // avancer jusqua fin
  //     while(currentPosition < fin){
  //       currentPosition = rouler(pid, fin, AX.readEncoder(REAR));
  //     }
  //     delay(200);
    // }

    // //si angle >= angleRetour
    // if(lirePot(POTPIN) >= angleRetour){
    //   // reculer at max speed jusqua debut
    //   while(currentPosition > debut){
    //     currentPosition = rouler(pid, debut, AX.readEncoder(REAR));
    //   }
    //   delay(200);
    // }
  // }
}