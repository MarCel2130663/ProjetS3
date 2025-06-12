#include <Arduino.h>
#include <LibS3GRO.h>

int MAGPIN = 2; //pin electroaimant
int POTPIN = 0; // pin potentiometre

void setup() {
  Serial.begin(9600);

  pinMode(MAGPIN, OUTPUT);
  pinMode(POTPIN, INPUT);
}

void loop() {
  //electroaimant
  /*digitalWrite(MAGPIN, HIGH);
  delay(10000);
  digitalWrite(MAGPIN, LOW);
  delay(1000);*/

  //potentiometre
  int valeur = analogRead(POTPIN);
  Serial.println(valeur);
}