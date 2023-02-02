#include "mouse.h"

#include "define.h"

#ifndef STDINT_H
#include <stdint.h>
#endif

#ifndef CMATH
#include <cmath>
#endif

mouse::mouse()
{
    this->R_g = 1;
    this->R_dir = true;
    this->L_g = 1;
    this->L_dir = true;
    this->R_slice = pwm_gpio_to_slice_num(R_F);
    this->L_slice = pwm_gpio_to_slice_num(L_F);
}

void mouse::move(float speedL, float speedR)
{
    uint16_t L_level = std::floor((L_g * abs(speedL) / float(100)) * float(65535));
    uint16_t R_level = std::floor((R_g * abs(speedR) / float(100)) * float(65535));
    L_dir = speedL >= 0;
    R_dir = speedR >= 0;
    pwm_set_both_levels(L_slice, L_dir * (L_level), !L_dir * (L_level));
    pwm_set_both_levels(R_slice, R_dir * (R_level), !R_dir * (R_level));
    pwm_set_enabled(L_slice, true);
    pwm_set_enabled(R_slice, true);
}

void mouse::move(float speed)
{
    uint16_t L_level = std::floor((L_g * speed / 100) * float(65535));
    uint16_t R_level = std::floor((R_g * speed / 100) * float(65535));
    L_dir = L_level >= 0;
    R_dir = R_level >= 0;
    pwm_set_both_levels(L_slice, L_dir * (L_level), !L_dir * (L_level));
    pwm_set_both_levels(R_slice, R_dir * (R_level), !R_dir * (R_level));
    pwm_set_enabled(L_slice, true);
    pwm_set_enabled(R_slice, true);
}

void mouse::stop()
{
    // pwm_set_enabled(L_slice, false);
    // pwm_set_enabled(R_slice, false);
    move(0);
}

void mouse::pwmInit()
{
    // Enable PWM - Left
    // https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=1
    // https://www.etechnophiles.com/raspberry-pi-pico-pinout-specifications-datasheet-in-detail/
    gpio_set_function(L_F, GPIO_FUNC_PWM);
    uint8_t Left_slice = pwm_gpio_to_slice_num(L_F);
    uint8_t Left_F_channel = pwm_gpio_to_channel(L_F);
    gpio_set_function(L_R, GPIO_FUNC_PWM);
    uint8_t Left_R_channel = pwm_gpio_to_channel(L_R);
    // Set the levels
    // pwm_set_both_levels(Right_slice, 65465/2, 65465/2);
    pwm_set_both_levels(Left_slice, 0, 0);
    // pwm_set_chan_level(slice, channel, level out of 2^16)
    pwm_set_clkdiv_int_frac(Left_slice, 2, 8); //~1kHz
    pwm_set_wrap(Left_slice, 65465);
    // pwm_set_enabled(Right_slice, true);

    // Enable PWM - Right
    // https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=1
    // https://www.etechnophiles.com/raspberry-pi-pico-pinout-specifications-datasheet-in-detail/
    gpio_set_function(R_F, GPIO_FUNC_PWM);
    uint8_t Right_slice = pwm_gpio_to_slice_num(R_F);
    uint8_t Right_F_channel = pwm_gpio_to_channel(R_F);
    gpio_set_function(R_R, GPIO_FUNC_PWM);
    uint8_t Right_R_channel = pwm_gpio_to_channel(R_R);
    // Set the levels
    // pwm_set_both_levels(Right_slice, 65465/2, 65465/2);
    pwm_set_both_levels(Right_slice, 0, 0);
    // pwm_set_chan_level(slice, channel, level out of 2^16)
    pwm_set_clkdiv_int_frac(Right_slice, 30, 8); //~1kHz
    pwm_set_wrap(Right_slice, 65465);
}

void mouse::rampUp()
{
    for (int i = 0; i <= 10; i++)
    {
        move(10 * i);
        sleep_ms(1.5 * i * 10);
    }
}

void mouse::rampDown()
{
    for (int i = 10; i >= 0; i--)
    {
        move(10 * i);
        sleep_ms(i * 10);
    }
}

/*
void moveStraightEncoder(float speed,int ticksR,int ticksL){
        static uint8_t call = 0;
        static int initR

        call = call++;







}*/

void mouse::straighten(float speed, uint16_t left, uint16_t right)
{
    float c1 = .05;
    float m = .0001;
    // float m2 = 40;
    float lower = .1;
    int deltaDist = abs(left - right);
    int band = 50;
    if (left > right)
    {
        // decrease right
        // R_g = float((120-deltaDist)/120);
        L_g = 1 - (m * deltaDist * deltaDist);
        R_g = 1;
        if (L_g > 1)
        {
            L_g = 1;
        }
        if (L_g < 0)
        {
            L_g = 0;
        }
        /*
        if((R_g - c1)>=lower){
            R_g = (m*deltaDist*deltaDist);
            if(R_g>1){
                R_g = 1;
            }

            //R_g = exp(-m * (deltaUW * deltaUW))
            }
            */
    }
    else if (right > left)
    {
        //  R_g = float((120-deltaDist)/120);

        R_g = 1 - (m * deltaDist * deltaDist);
        L_g = 1;
        if (R_g > 1)
        {
            R_g = 1;
        }
        if (R_g < 0)
        {
            R_g = 0;
        }
        /*
        if ((L_g - c1) >= lower)
        {
        }
        */
    }
    /*
    else{
        L_g = 1;
        R_g = 1;
    }
    */
    move(speed);
}

int mouse::moveStraight(float inputSpeed, int &inticksL, int &inticksR)
{
    // always try and match to the right wheel.
    int prevTicksL = inticksL;
    int prevTicksR = inticksR;
    // wait
    sleep_ms(10);
    int newTicksL = inticksL;
    int newTicksR = inticksR;
    int dR = newTicksR - prevTicksR;
    int dL = newTicksL - prevTicksL;
    float e = dR - dL;
    float c1 = .05;

    
    if (e > 0)
    {
        // if left can speed up
        if ((L_g + c1) <= 1)
        {
            L_g = L_g + c1;
        }
        else
        {
            /*float rem = 1 - L_g;
            L_g = 1;
            R_g = R_g - rem;*/
            R_g = R_g - c1;

        }
    }
    else if (e < 0)
    {
        // if left can speed up
        if ((R_g + c1) <= 1)
        {
            R_g = R_g + c1;
        }
        else
        {
            /*
            float rem = 1 - R_g;
            R_g = 1;
            L_g = L_g - rem;*/
             L_g = L_g - c1;
        }
    }

    return c1;
}
