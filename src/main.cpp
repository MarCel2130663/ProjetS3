#include <Arduino.h>
#include <LibS3GRO.h>

int MAGPIN = 2;

void setup() {
  pinMode(MAGPIN, OUTPUT);
}

void loop() {
  digitalWrite(MAGPIN, HIGH);
  delay(10000);
  digitalWrite(MAGPIN, LOW);
  delay(1000);
}