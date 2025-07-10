#include "PIDhihi.h"
#include <Arduino.h>

PIDhihi::PIDhihi(float p, float i, float d){
    kp = p;
    ki = i;
    kd = d;
    integral = 0;
    lastError = 0;
    lastTime = 0;
}

float PIDhihi::calculate(int setPoint, int currentPosition){
    long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    int error = setPoint - currentPosition;
    integral += error * dt;
    float derivative = (error - lastError) / dt;

    float output = (kp * error) + (ki * integral) + (kd * derivative);
    lastError = error;
    return output;
}