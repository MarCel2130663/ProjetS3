#ifndef PID_H
#define PID_H

class PID{
public:
    PID(float p, float i, float d);
    float calculate(float setPoint, float currentPosition, float dt);
private:
    float kp, ki, kd;
    float integral, lastError;
};

#endif