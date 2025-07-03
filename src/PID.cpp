#include "PID.h"

PID::PID(float p, float i, float d){
    kp = p;
    ki = i;
    kd = d;
    integral = 0;
    lastError = 0;
}

float PID::calculate(float setPoint, float currentPosition, float dt){
    float error = setPoint - currentPosition;
    integral += error * dt;
    float derivative = (error - lastError) / dt;

    float output = (kp * error) + (ki * integral) + (kd * derivative);
    lastError = error;
    return output;
}