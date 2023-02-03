#pragma once

#ifndef MOUSE_H
#define MOUSE_H

#ifndef STDINT_H
#include <stdint.h>
#endif

class mouse
{
private:
    

public:
    float R_gE;
    float R_gD;
    bool R_dir = true;
    float L_gE;
    float L_gD;
    bool L_dir = true;
    // pwm info
    int R_slice;
    int L_slice;
    float MouseSpeed=0;



    // direct motion
    mouse();
    void move(float speedL, float speedR);
    void move(float speed);
    void stop();
    void pwmInit();
    void straighten(float speed, uint16_t left, uint16_t right);
    int moveStraight(float speed, int &inticksL, int &inticksR);
    void rampUp();
    void rampDown();
    //turnLeft();
    //turnRight();

    // smart motion
    //moveStraightDistance();
    //void moveStraightEncoder(float speed,int ticksR, int ticksL);
    //moveStraight();

};
#endif