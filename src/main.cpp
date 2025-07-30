#include <Arduino.h>
#include <LibS3GRO.h>
#include <string.h>
#include <ArduinoJson.h>
#include "PIDhihi.h"

// CONSTANTES
#define UPDATE_PERIOD 100 // ms

//UI
bool START = false; // utilisateur
float courant;
float tension;
float position;
float pos_membre_all;
volatile bool shouldRead_;
volatile bool shouldSend_;
SoftTimer timerSendMessage_;
SoftTimer lightTimer_;
int lightState_ = 0;

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
int lastPosition = 0;
int currentPosition = 0;
float output;

// OBJETS
ArduinoX AX;
PIDhihi pid(kp, ki, kd);

// POSITIONS CIBLES
int home = 200;
int debut = 3500;
int obstacle = debut + 2200; // obstacle = debut + oscillation = 5700

// ANGLE CIBLE
int angleArriere = 730;
int angleAvant = 810;

bool oscille;

void timerCallback(){shouldSend_ = true;}

void serialEvent(){shouldRead_ = true;}

void turnOnLight(){
  digitalWrite(LED_BUILTIN, HIGH);
  lightState_ = 1;
}
 
// Éteindre la lumière
void turnOffLight(){
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  lightTimer_.disable();
  lightState_ = 0;
}

void sendMessage(){
  /* Envoi du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  
  // Elements du message
  doc["tension"] = tension;
  doc["courant"] = courant;
  doc["position"] = position;
  doc["AngleYeet"] = pos_membre_all;
  
  // Serialisation
  serializeJson(doc, Serial);
  
  // Envoi
  Serial.println();
  shouldSend_ = false;
}

void readMessage(){
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;
 
  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;
 
  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
 
  // Analyse des éléments du message message
  // Si on doit allumer la lumière
  parse_msg = doc["StartCode"];
  if(!parse_msg.isNull()){
    START = doc["StartCode"].as<int>();
    turnOnLight();
    lightTimer_.setDelay(10);
    lightTimer_.enable();
  }
 
  // Si on doit faire un echo du message recu
  parse_msg = doc["userMsg"];
  if(!parse_msg.isNull()){
    Serial.println(doc["userMsg"].as<String>());
  }
}

void sendPosition(){
  position += abs(currentPosition-lastPosition) * 0.44 / 3200;
  lastPosition = currentPosition;
}

void rouler(PIDhihi pid, float sp, float cp){
  output = pid.calculate(sp, cp);
  float speed = constrain(output, -0.57, 0.57);
  AX.setMotorPWM(FRONT, speed);
  AX.setMotorPWM(REAR, speed);

  tension = AX.getVoltage();
  courant = AX.getCurrent();
  currentPosition = cp;
  sendPosition();
  pos_membre_all = analogRead(POTPIN)/(223/85)-305;
  if(shouldSend_){
    sendMessage();
  }
  timerSendMessage_.update();
}

void arreter(){
  AX.setMotorPWM(FRONT, 0);
  AX.setMotorPWM(REAR, 0);
}

void setup() {
  Serial.begin(115200);

  oscille = true;
  pinMode(POTPIN, INPUT);
  pinMode(MAGPIN1, OUTPUT);
  pinMode(MAGPIN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  AX.init();

  timerSendMessage_.setDelay(UPDATE_PERIOD);
  timerSendMessage_.setCallback(timerCallback);
  timerSendMessage_.enable();

  lightTimer_.setCallback(turnOffLight);
  lightTimer_.disable();
}

void loop() {
  Serial.println("Debut loop");
  AX.resetEncoder(REAR);
  //activer electroaimant
  digitalWrite(MAGPIN1, HIGH);
  digitalWrite(MAGPIN2, HIGH);

  pos_membre_all = analogRead(POTPIN)/(223/85)-305;
  if(shouldSend_){
    sendMessage();
  }
  // mettre sapin
  if(shouldRead_){
    readMessage();
  }
  timerSendMessage_.update();
  lightTimer_.update();
  
  while(START){
    Serial.println("Debut du cycle");
    oscille = true;
    // avancer la premiere fois jusqua obstacle1
    while(currentPosition < obstacle){
      rouler(pid, obstacle, AX.readEncoder(REAR));
    }
    arreter();
    delay(100);
    // reculer jusqua debut
    Serial.println("Reculer");
    while(currentPosition > debut && oscille){
      rouler(pid, debut, AX.readEncoder(REAR));
      if(analogRead(POTPIN) <= angleArriere && oscille){
        Serial.println("Angle arriere atteint");
        while(currentPosition < obstacle && oscille){
          rouler(pid, obstacle, AX.readEncoder(REAR));
          if(analogRead(POTPIN) >= angleAvant && oscille){
            Serial.println("Lacher sapin");
            // desactiver electroaimant
            digitalWrite(MAGPIN1, LOW);
            digitalWrite(MAGPIN2, LOW);
            delay(100);
            Serial.println("Oscille false");
            oscille = false;
            START = false;
          }
        }
      }
    }
  }
  Serial.println("Retour au bout du rail");
  while(currentPosition > home){
    rouler(pid, home, AX.readEncoder(REAR));
  }
}