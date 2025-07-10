#ifndef PIDHIHI_H
#define PIDHIHI_H

class PIDhihi{
    public:
        PIDhihi(float p, float i, float d);
        float calculate(int setPoint, int currentPosition);
    private:
        float kp, ki, kd;
        float integral, lastError, lastTime;
};

#endif