#ifndef PIDURMOM_H
#define PIDURMOM_H

class PIDurmom{
    public:
        PIDurmom(float p, float i, float d);
        float calculate(int setPoint, int currentPosition);
    private:
        float kp, ki, kd;
        float integral, lastError, lastTime;
};

#endif