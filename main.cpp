#include "mouse/define.h"

#ifndef STDLIB_H
#include "pico/stdlib.h"
#endif

#ifndef I2C_H
#include "hardware/i2c.h"
#endif

#ifndef PWM_H
#include "hardware/pwm.h"
#endif

#ifndef STRING
#include <string>
#endif

#ifndef STDINT_H
#include <stdint.h>
#endif

#ifndef MATH_H
#include <math.h>
#endif

#include <algorithm>

// includes for OLED Display
#include "pico-ssd1306/ssd1306.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"
// Use the namespace for convenience
using namespace pico_ssd1306;

// includes for VL53L1X
#ifndef VL53L1X_TYPES_H
#include "pico/binary_info.h"
extern "C"
{
#include "VL53L1X/Library/public/include/VL53L1X_api.h"
#include "VL53L1X/Library/public/include/VL53L1X_types.h"
}
#endif

// Custom Library includes
#define SensorDefaultAddress 0x29
/*
void testScreen(SSD1306 display){
drawText(&display, font_5x8, "Screen Test", 0, 0);
        display.sendBuffer();
        sleep_ms(500);
        display.sendBuffer();
        display.clear();
        display.sendBuffer();
}*/

// Globally referenced variables
// -- Interrupts --
static int ticksR = 10;
static int ticksL = 10;
bool led = true;
// -- Pins --
// -- Sensor GPIO & SHUT --
// -- Left = 1 , Left Diagonal = 2, Front = 3, Right Diagonal = 4, Right = 5 --
static uint8_t SensorL_Shutdown = S1_SH;
static uint8_t SensorL_GPIO = S1_INT;
static uint8_t SensorL_address = S1_ADD;
static uint8_t SensorLD_Shutdown = S2_SH;
static uint8_t SensorLD_GPIO = S2_INT;
static uint8_t SensorLD_address = S2_ADD;
static uint8_t SensorF_Shutdown = S3_SH;
static uint8_t SensorF_GPIO = S3_INT;
static uint8_t SensorF_address = S3_ADD;
static uint8_t SensorRD_Shutdown = S4_SH;
static uint8_t SensorRD_GPIO = S4_INT;
static uint8_t SensorRD_address = S4_ADD;
static uint8_t SensorR_Shutdown = S5_SH;
static uint8_t SensorR_GPIO = S5_INT;
static uint8_t SensorR_address = S5_ADD;
// -- Right Motor --
static uint8_t Right_F = R_F;
static uint8_t Right_R = R_R;
// -- Left Motor --
static uint8_t Left_F = L_F;
static uint8_t Left_R = L_R;
// -- Encoder --
static uint8_t Encode_L = ENC_L;
static uint8_t Encode_R = ENC_R;
// -- VL53L1X --
// -- Left = 1 , Front = 2, Right = 3 --
static VL53L1X_Status_t statusL;
static VL53L1X_Result_t resultsL; // this is a structure with a lot of data
static VL53L1X_Status_t statusLD;
static VL53L1X_Result_t resultsLD; // this is a structure with a lot of data
static VL53L1X_Status_t statusF;
static VL53L1X_Result_t resultsF; // this is a structure with a lot of data
static VL53L1X_Status_t statusRD;
static VL53L1X_Result_t resultsRD; // this is a structure with a lot of data
static VL53L1X_Status_t statusR;
static VL53L1X_Result_t resultsR; // this is a structure with a lot of data

static uint8_t dataReady = 0;
// -- Mouse Characteristics --
// Model Parameters
static float W = 170;    // Width of Maze [mm]
static float a = 33.25;  // This is the distance from the sensor focal point for L & R [mm]
static float b = 33;     // This is the distance from the sensor focal point for DL & DR [mm]
static float c = 30;     // This is the distance from the sensor focal point for F [mm]
static float phi_D = 45; // Angle from 0deg (forwards) of DL & DR
static float d = 25;     // Distance from sensor focal point to center of rotation [mm]
static float R = 50;     // Radius of Wheel in [mm]
static float L = 40;     // Distance between each wheel [mm]

// Values to hold the averages which are to be used as the "clean" distances

#define filterSize 3
float L_hist[filterSize];
float L_sum = 0;
uint8_t L_idx_oldest = 0;
static float distL = 0;

float LD_hist[filterSize];
float LD_sum = 0;
uint8_t LD_idx_oldest = 0;
static float distLD = 0;

float F_hist[filterSize];
float F_sum = 0;
uint8_t F_idx_oldest = 0;
static float distF = 0;

float RD_hist[filterSize];
float RD_sum = 0;
uint8_t RD_idx_oldest = 0;
static float distRD = 0;

float R_hist[filterSize];
float R_sum = 0;
uint8_t R_idx_oldest = 0;
static float distR = 0;

void updateSensorFilter(uint16_t X_sensor, float X_hist[], float &X_sum, uint8_t &X_idx_oldest, float &distX)
{
    X_sum = X_sum - X_hist[X_idx_oldest];
    X_hist[X_idx_oldest] = X_sensor;
    X_idx_oldest = (X_idx_oldest + 1) % filterSize;
    X_sum = X_sum + X_sensor;
    distX = X_sum / filterSize;
}

// Calibration Constants
static float calL = 0;
static float calLD = 0;
static float calF = 0;
static float calRD = 0;
static float calR = 0;

// Frequently used conversions
static float DEG2RAD = .017453292519943296;
static float RAD2DEG = 57.2957795130823209;
// Old stuff
static float R_gE = 1;
static float R_gD = 1;
static bool R_dir = true;
static float L_gE = 1;
static float L_gD = 1;
static bool L_dir = true;
static int R_slice = pwm_gpio_to_slice_num(R_F);
static int L_slice = pwm_gpio_to_slice_num(L_F);
static float MouseSpeed = 75;
bool moving = false;
bool LEDon = false;

static int mouseW = 58;

// -- Convenient Model Variables

// END Globally reference variables

void setupSensor(uint16_t newAddress, SSD1306 display)
{
    // Initialize Pico's I2C and check for good address
    if (VL53L1X_I2C_Init(SensorDefaultAddress, i2c0) < 0)
    {
        drawText(&display, font_5x8, "Error init Sensor", 0, 0);
        display.sendBuffer();
        sleep_ms(500);
    }

    // Ensure the sensor has booted and print to display
    uint8_t sensorState;
    do
    {
        VL53L1X_BootState(SensorDefaultAddress, &sensorState);
        VL53L1X_WaitMs(SensorDefaultAddress, 2);
    } while (sensorState == 0);
    drawText(&display, font_5x8, "Booted", 0, 0);
    display.sendBuffer();
    sleep_ms(100);
    display.clear();
    display.sendBuffer();

    // Initialize and configure sensor
    VL53L1X_SensorInit(SensorDefaultAddress);
    VL53L1X_SetDistanceMode(SensorDefaultAddress, 1);
    VL53L1X_SetTimingBudgetInMs(SensorDefaultAddress, 100);
    VL53L1X_SetInterMeasurementInMs(SensorDefaultAddress, 100);
    VL53L1X_SetI2CAddress(SensorDefaultAddress, newAddress);
    drawText(&display, font_5x8, "Finished", 0, 0);
    display.sendBuffer();
    sleep_ms(100);
    display.clear();
    display.sendBuffer();
}

// Interrupt functions & setup

void updateEncoderR(void)
{
    gpio_acknowledge_irq(ENC_R, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    ticksR++;
}

void updateEncoderL(uint pin, uint32_t event)
{
    // gpio_acknowledge_irq(ENC_L, GPIO_IRQ_EDGE_RISE);
    ticksL++;
    ticksR--;
}

void move(float speedL, float speedR)
{
    uint16_t L_level = std::floor((L_gE * L_gD * abs(speedL) / float(100)) * float(65535));
    uint16_t R_level = std::floor((R_gE * R_gD * abs(speedR) / float(100)) * float(65535));
    L_dir = speedL >= 0;
    R_dir = speedR >= 0;
    pwm_set_both_levels(L_slice, L_dir * (L_level), !L_dir * (L_level));
    pwm_set_both_levels(R_slice, R_dir * (R_level), !R_dir * (R_level));
    pwm_set_enabled(L_slice, true);
    pwm_set_enabled(R_slice, true);
}

void move(float speed)
{
    MouseSpeed = speed;
    moving = true;
    uint16_t L_level = std::floor((L_gE * L_gD * speed / 100) * float(65535));
    uint16_t R_level = std::floor((R_gE * R_gD * speed / 100) * float(65535));
    L_dir = L_level >= 0;
    R_dir = R_level >= 0;
    pwm_set_both_levels(L_slice, L_dir * (L_level), !L_dir * (L_level));
    pwm_set_both_levels(R_slice, R_dir * (R_level), !R_dir * (R_level));
    pwm_set_enabled(L_slice, true);
    pwm_set_enabled(R_slice, true);
}

void stop()
{
    move(0);
    moving = false;
}

void pwmInit()
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

void rampUp()
{
    for (int i = 0; i <= 10; i++)
    {
        move(10 * i);
        sleep_ms(1.5 * i * 10);
    }
}

void rampDown()
{
    for (int i = 10; i >= 0; i--)
    {
        move(10 * i);
        sleep_ms(i * 10);
    }
}

void rampUp2()
{
    for (int i = 0; i <= 30; i++)
    {
        // float j = 4 * sqrtf(30*i)
        float j = .5 * pow(.5 * i, 2);
        if (j > 100)
        {
            j = 100;
        }
        move(j);
        sleep_ms(25);
    }
}

void rampDown2()
{
    for (int i = 0; i <= 30; i++)
    {
        // float j = 4 * sqrtf(30*i)
        float j = 100 - (.5 * pow(.5 * i, 2));
        if (j < 100)
        {
            j = 0;
        }
        move(j);
        sleep_ms(25);
    }
}

// Custom print float statement
void dispFloat(float inFloat, SSD1306 display, int duration)
{
    drawText(&display, font_12x16, (std::to_string(inFloat)).data(), 0, 0);
    display.sendBuffer();
    sleep_ms(duration * .75);
    display.clear();
    display.sendBuffer();
    sleep_ms(duration * .25);
}

void dispFloat(std::string inString, float inFloat, SSD1306 display, int duration)
{
    drawText(&display, font_8x8, (inString + std::to_string(inFloat)).data(), 0, 0);
    display.sendBuffer();
    sleep_ms(duration * .75);
    display.clear();
    display.sendBuffer();
    sleep_ms(duration * .25);
}

void dispDualFloat(std::string inString1, float inFloat1, std::string inString2, float inFloat2, SSD1306 display, int duration)
{
    drawText(&display, font_8x8, (inString1 + std::to_string(inFloat1)).data(), 0, 0);
    drawText(&display, font_8x8, (inString2 + std::to_string(inFloat2)).data(), 0, 9);
    display.sendBuffer();
    sleep_ms(duration * .75);
    display.clear();
    display.sendBuffer();
    sleep_ms(duration * .25);
}

// Example call: distL = measureSensor(SensorL_address);
void measureSensor(uint8_t sensorAddr)
{
    uint16_t measurement = 0;
    VL53L1X_CheckForDataReady(sensorAddr, &dataReady);
    if (dataReady == 1)
    {

        VL53L1X_GetDistance(sensorAddr, &measurement);
        switch (sensorAddr)
        {
        case S1_ADD:
            updateSensorFilter(measurement, L_hist, L_sum, L_idx_oldest, distL);
            // Put calibration contants after here if nessecary

            break;
        case S2_ADD:
            updateSensorFilter(measurement, LD_hist, LD_sum, LD_idx_oldest, distLD);
            // Put calibration contants after here if nessecary
            break;
        case S3_ADD:
            updateSensorFilter(measurement, F_hist, F_sum, F_idx_oldest, distF);
            // Put calibration contants after here if nessecary
            break;
        case S4_ADD:
            updateSensorFilter(measurement, RD_hist, RD_sum, RD_idx_oldest, distRD);
            // Put calibration contants after here if nessecary
            break;
        case S5_ADD:
            updateSensorFilter(measurement, R_hist, R_sum, R_idx_oldest, distR);
            // Put calibration contants after here if nessecary
            break;
        }
    }
    else
    {
    }
}

void updateAllSensors()
{
    measureSensor(SensorL_address);
    measureSensor(SensorLD_address);
    measureSensor(SensorF_address);
    measureSensor(SensorRD_address);
    measureSensor(SensorR_address);
}

float calculateHeading()
{
    float rat = W / (distL + distR + (2 * a));
    float theta_deg = 90 - (asinf(rat)) * RAD2DEG;
    float out = abs(theta_deg);

    // Check the diagonal sensors for polarity
    if (distLD < distRD)
    {
        out = -1 * out;
    }
    // Small angle correct
    if (abs(out) < 30)
    {
        // LHS Triangle
        float s1 = distL + a;
        float s2 = distLD + b;
        // RHS Triangle
        float s4 = distRD + b;
        float s5 = distR + a;
        // Triangle Projection on walls
        float L_AB = sqrtf(powf(s1, 2) + powf(s2, 2) - (2 * s1 * s2 * cosf(phi_D * DEG2RAD)));
        float L_ED = sqrtf(powf(s4, 2) + powf(s5, 2) - (2 * s4 * s5 * cosf(phi_D * DEG2RAD)));
        // Triangle upper angles
        float phi_s1 = asinf(s1 * sinf(phi_D * DEG2RAD) / L_AB) * RAD2DEG;
        float phi_s4 = asinf(s4 * sinf(phi_D * DEG2RAD) / L_ED) * RAD2DEG;
        // Solve for angles
        float theta_L = -90 + phi_s1 + phi_D;
        float theta_R = -90 + phi_s4 + phi_D;
        // Check for which side is accurate
        if (abs(theta_R - theta_L) > 4)
        {
            // out = MIN(theta_L, theta_R);
            out = 969696969;
        }
        else
        {
            out = (theta_L + theta_R) / 2.0;
            out = theta_L;
        }
        out = -1 * out;
    }
    out = -1 * out;
    return out;
}

float calculateHeadingModern()
{
    float rat = W / (distR + distL + 2 * a); // compares the width of the maze to the measured width of the maze
    float theta_degrees = 90 - asinf(rat) * RAD2DEG;
    float out = theta_degrees;
    //  LHS Triangle
    float s1 = distL + a;
    float s2 = distLD + b;
    // RHS Triangle
    float s4 = distRD + b;
    float s5 = distR + a;
    // Project onto walls
    float L_AB = sqrtf(pow(s1, 2) + powf(s2, 2) - 2 * s1 * s2 * cosf(phi_D * DEG2RAD));
    float L_ED = sqrtf(pow(s4, 2) + powf(s5, 2) - 2 * s4 * s5 * cosf(phi_D * DEG2RAD));
    // Upper sides
    float phi_s1 = RAD2DEG * asinf(s1 * sinf(phi_D * DEG2RAD) / L_AB);
    float phi_s5 = RAD2DEG * asinf(s5 * sinf(phi_D * DEG2RAD) / L_ED);

    if ((abs(out) < 25) || (abs(rat) > 1))
    {
        // Complements
        float phi_s1p = 180 - phi_s1;
        float phi_s5p = 180 - phi_s5;
        float thetaL;
        float thetaR;
        // Leftward facing
        if (phi_s1 < phi_s5)
        {
            thetaL = 90 - phi_s1p + phi_D;
            // float thetaR = 90 - phi_s5p + phi_D;
            thetaR = 45 - phi_s5;
        }
        else
        {
            thetaL = 45 - phi_s1;
            thetaR = 90 - phi_s5p + phi_D;
        }
        // Edge Guard
        out = -(thetaL + thetaR) / 2;
    }
    else
    {
    }
    if (phi_s1 > phi_s5)
    {
        out = -1 * out;
    }
    // out = -1 * out;
    return out;
}

float getX()
{
    float out = 0;
    float theta = calculateHeading();
    // Calculate the distances from the L-DL & R-DR sensors
    float xp1 = (distL + a) * cosf((180 + theta) * DEG2RAD);
    float xp2 = (distLD + b) * cosf((phi_D + theta) * DEG2RAD);
    float xp4 = W - (distRD + b) * cosf((90 - phi_D + theta) * DEG2RAD);
    float xp5 = W - (distR + a) * cosf((180 + theta) * DEG2RAD);
    out = (abs(xp1) + abs(xp2) + abs(xp4) + abs(xp5)) / float(4.0);
    return out;
}

float getX(float inHeading)
{
    float out = 0;
    float theta = inHeading;
    // Calculate the distances from the L-DL & R-DR sensors
    float xp1 = (distL + a) * cosf(theta * DEG2RAD);
    // float xp2 = (distLD + b) * cosf((phi_D + theta) * DEG2RAD);
    // float xp4 = W - (distRD + b) * cosf((90 - phi_D + theta) * DEG2RAD);
    float xp5 = W - (distR + a) * cosf(theta * DEG2RAD);
    // out = (abs(xp1) + abs(xp2) + abs(xp4) + abs(xp5)) / float(4.0);
    out = (xp1 + xp5) / 2.0;
    return out;
}
float A1 = 0.9;
float B1 = 0.1;
float C1 = 0.000002;
float D1 = .5;

float R_position_gain;
float L_position_gain;
float R_angle_gain;
float L_angle_gain;
float speedL;
float speedR;

void updateController(float heading, float xpos)
{
    // Map position to have little effect around W/2 and increasing at sides
    L_angle_gain = A1 + B1 * (xpos / (W / 2.0));
    R_angle_gain = (A1 + 2*B1) - B1 * (xpos / (W / 2.0));
    /*if (heading > 0)
    {
        R_angle_gain = C1 * pow(heading, 2) + 1;
        L_angle_gain = -C1 * pow(heading, 2) + 1;
    }
    else
    {
        R_angle_gain = -C1 * pow(heading, 2) + 1;
        L_angle_gain = C1 * pow(heading, 2) + 1;
    }
    if(R_angle_gain<D1){
        R_angle_gain = D1;
    }
    if(L_angle_gain<D1){
        L_angle_gain = D1;
    }
    */
    L_position_gain = -C1*powf(xpos-W/2.0,3)+1;
    R_position_gain = C1*powf(xpos-W/2.0,3)+1;

    speedL = MouseSpeed*L_angle_gain*L_position_gain;
    if(speedL>100){
        speedL = 100;
    }
    speedR = MouseSpeed*R_angle_gain*R_position_gain;
    if(speedR>100){
        speedR = 100;
    }
    if(moving){
        move(speedL,speedR);
    }
    else{
        rampUp2();
        moving = true;
        move(speedL,speedR);
    }
    
}


int main()
{
    // GPIO setup the LED
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    // GPIO Setup
    gpio_init(SensorL_GPIO);
    gpio_set_dir(SensorL_GPIO, GPIO_IN);
    gpio_init(SensorL_Shutdown);
    gpio_set_dir(SensorL_Shutdown, GPIO_OUT);
    // Shutdown is active low so set it high
    gpio_put(SensorL_Shutdown, 1);

    gpio_init(SensorLD_GPIO);
    gpio_set_dir(SensorLD_GPIO, GPIO_IN);
    gpio_init(SensorLD_Shutdown);
    gpio_set_dir(SensorLD_Shutdown, GPIO_OUT);
    // Shutdown is active low so set it high
    gpio_put(SensorLD_Shutdown, 1);

    gpio_init(SensorF_GPIO);
    gpio_set_dir(SensorF_GPIO, GPIO_IN);
    gpio_init(SensorF_Shutdown);
    gpio_set_dir(SensorF_Shutdown, GPIO_OUT);
    // Shutdown is active low so set it high
    gpio_put(SensorF_Shutdown, 1);

    gpio_init(SensorRD_GPIO);
    gpio_set_dir(SensorRD_GPIO, GPIO_IN);
    gpio_init(SensorRD_Shutdown);
    gpio_set_dir(SensorRD_Shutdown, GPIO_OUT);
    // Shutdown is active low so set it high
    gpio_put(SensorRD_Shutdown, 1);

    gpio_init(SensorR_GPIO);
    gpio_set_dir(SensorR_GPIO, GPIO_IN);
    gpio_init(SensorR_Shutdown);
    gpio_set_dir(SensorR_Shutdown, GPIO_OUT);
    // Shutdown is active low so set it high
    gpio_put(SensorR_Shutdown, 1);

    // Enable PWM - Left
    // https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=1
    // https://www.etechnophiles.com/raspberry-pi-pico-pinout-specifications-datasheet-in-detail/
    gpio_set_function(Left_F, GPIO_FUNC_PWM);
    static uint8_t Left_slice = pwm_gpio_to_slice_num(Left_F);
    static uint8_t Left_F_channel = pwm_gpio_to_channel(Left_F);
    gpio_set_function(Left_R, GPIO_FUNC_PWM);
    static uint8_t Left_R_channel = pwm_gpio_to_channel(Left_R);
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
    gpio_set_function(Right_F, GPIO_FUNC_PWM);
    static uint8_t Right_slice = pwm_gpio_to_slice_num(Right_F);
    static uint8_t Right_F_channel = pwm_gpio_to_channel(Right_F);
    gpio_set_function(Right_R, GPIO_FUNC_PWM);
    static uint8_t Right_R_channel = pwm_gpio_to_channel(Right_R);
    // Set the levels
    // pwm_set_both_levels(Right_slice, 65465/2, 65465/2);
    pwm_set_both_levels(Right_slice, 0, 0);
    // pwm_set_chan_level(slice, channel, level out of 2^16)
    pwm_set_clkdiv_int_frac(Right_slice, 30, 8); //~1kHz
    pwm_set_wrap(Right_slice, 65465);
    // pwm_set_enabled(Right_slice, true);

    // alternate the forward and backwards at 50% dc
    bool dir = true;
    int state = 0;
    pwm_set_both_levels(Right_slice, dir * (65465 / 2), !dir * (65465 / 2));
    // pwm_set_enabled(Right_slice, true);

    /* */

    // Init i2c0 controller
    i2c_init(i2c0, 1000000); // 1MHz
    // Set up pins GPIO 12 and 13 (***** NOT PIN NUMBER, GPIO NUMBER*****)
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);
    sleep_ms(250); // delay for display to boot up

    // Create a new display object at address 0x3C and size of 128x64
    // -- Display --
    static SSD1306 display = SSD1306(i2c0, 0x3C, Size::W128xH32);
    sleep_ms(500);

    // Here we rotate the display by 180 degrees, so that it's not upside down from my perspective
    // If your screen is upside down try setting it to 1 or 0
    display.setOrientation(1);
    display.clear();
    display.sendBuffer();
    sleep_ms(200);

    //  ------  Init Sensor L ------
    gpio_put(SensorL_Shutdown, 1);
    // turn off others
    gpio_put(SensorLD_Shutdown, 0);
    gpio_put(SensorF_Shutdown, 0);
    gpio_put(SensorRD_Shutdown, 0);
    gpio_put(SensorR_Shutdown, 0);
    sleep_ms(100);
    setupSensor(SensorL_address, display);

    //  ------  Init Sensor LD ------
    gpio_put(SensorLD_Shutdown, 1);
    sleep_ms(100);
    setupSensor(SensorLD_address, display);

    //  ------  Init Sensor F ------
    gpio_put(SensorF_Shutdown, 1);
    sleep_ms(100);
    setupSensor(SensorF_address, display);

    //  ------  Init Sensor RD ------
    gpio_put(SensorRD_Shutdown, 1);
    sleep_ms(100);
    setupSensor(SensorRD_address, display);

    //  ------  Init Sensor R ------
    gpio_put(SensorR_Shutdown, 1);
    sleep_ms(100);
    setupSensor(SensorR_address, display);

    // Enable all three sensors
    VL53L1X_StartRanging(SensorL_address);
    VL53L1X_StartRanging(SensorLD_address);
    VL53L1X_StartRanging(SensorF_address);
    VL53L1X_StartRanging(SensorRD_address);
    VL53L1X_StartRanging(SensorR_address);

    drawText(&display, font_5x8, "Ranging on", 0, 0);
    display.sendBuffer();
    sleep_ms(250);
    display.clear();
    display.sendBuffer();

    // Sensor calibration to "zero" them
    updateAllSensors();
    calL = (W / 2 - a) - distL;
    calR = (W / 2 - a) - distR;
    calLD = (W / 2) / cosf(phi_D * DEG2RAD) - b - distLD;
    calRD = (W / 2) / cosf(phi_D * DEG2RAD) - b - distRD;

    // Enable interrupts
    gpio_init(Encode_L);
    gpio_set_dir(Encode_L, GPIO_IN);
    gpio_disable_pulls(Encode_L);
    // gpio_add_raw_irq_handler(Encode_L, &updateEncoderL);
    // gpio_set_irq_enabled(Encode_L, GPIO_IRQ_EDGE_RISE, true);
    // gpio_set_irq_enabled_with_callback(Encode_L, GPIO_IRQ_EDGE_RISE, true, &updateEncoderL);
    //  gpio_add_raw_irq_handler(Encode_L, &updateEncoderL);
    gpio_set_irq_enabled_with_callback(Encode_L, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &updateEncoderL);
    // gpio_set_irq_enabled(Encode_L, GPIO_IRQ_EDGE_RISE, true);

    gpio_init(Encode_R);
    gpio_set_dir(Encode_R, GPIO_IN);
    gpio_disable_pulls(Encode_R);
    gpio_add_raw_irq_handler(Encode_R, &updateEncoderR);
    // gpio_set_irq_enabled_with_callback(Encode_R, GPIO_IRQ_EDGE_RISE, true, &updateEncoderR);
    gpio_set_irq_enabled(Encode_R, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // Test variables
    float heading = 0;
    float xpos = W / 2;
    updateAllSensors();
    updateAllSensors();
    updateAllSensors();
    MouseSpeed = 25;
    while (1)
    {
        gpio_put(25, true);
        updateAllSensors();
        if(distF<50){
            stop();
            while (distF<50)
            {
                updateAllSensors();
            }
            sleep_ms(1000);
        }
        gpio_put(25, false);
        heading = calculateHeadingModern();
        xpos = getX(heading);
        updateController(heading, xpos);

        /*
         dispFloat("DL:", distL, display, 1000);
         dispFloat("DLD:", distLD, display, 1000);
         dispFloat("DF:", distF, display, 1000);
         dispFloat("DRD:", distRD, display, 1000);
         dispFloat("DR:", distR, display, 1000);
         */

        
        //dispDualFloat("Heading:", heading, "X:", xpos, display, 500);
        // sleep_ms(100);
    }

    /* while (1)
     {
         gpio_put(25, LEDon);
         printf("Top of the Loop!");
         // LEDon = !LEDon;
         sleep_ms(200);
         // Run a loop ----------------------------------------------
         // Update the sensors
         updateAllSensors();
         // heading = measureSensor(SensorL_address);
         // heading = VL53L1X_CheckForDataReady(SensorL_address, &dataReady);
         float out = 999;
         float rat = W / (distL + distR + (2 * a));
         // LED off
         LEDon = false;
         if (rat <= 1)
         {

             float theta_deg = 90 - (asinf(rat)) * RAD2DEG;
             float out = abs(theta_deg);
             // Check the diagonal sensors for polarity
         }
         // Small angle correct

         if ((out == 999) || (abs(out) < 20))
         {
             // LED on
             LEDon = true;
             // LHS Triangle
             float s1 = distL + a;
             // dispFloat(s1, display, 500);
             float s2 = distLD + b;
             // dispFloat(s2, display, 500);
             //  RHS Triangle
             float s4 = distRD + b;
             // dispFloat(s4, display, 500);
             float s5 = distR + a;
             // dispFloat(s5, display, 500);
             //  Triangle Projection on walls
             float L_AB = sqrtf(powf(s1, 2) + powf(s2, 2) - (2 * s1 * s2 * cosf(phi_D * DEG2RAD)));
             // dispFloat(L_AB, display, 500);
             float L_ED = sqrtf(powf(s4, 2) + powf(s5, 2) - (2 * s4 * s5 * cosf(phi_D * DEG2RAD)));
             // dispFloat(L_ED, display, 500);
             //  Triangle upper angles
             float phi_s1 = asinf(s1 * sinf(phi_D * DEG2RAD) / L_AB) * RAD2DEG;
             // dispFloat(phi_s1, display, 500);
             float phi_s5 = asinf(s5 * sinf(phi_D * DEG2RAD) / L_ED) * RAD2DEG;
             // dispFloat(phi_s4, display, 500);
             //  Solve for angles
             float theta_L = -90 + phi_s1 + phi_D;
             // dispFloat(theta_L, display, 500);
             float theta_R = -90 + phi_s5 + phi_D;
             // dispFloat(theta_R, display, 500);
             //  Check for which side is accurate
             /*
             if (abs(theta_R - theta_L) > 4)
             {
                 //out = MIN(theta_L, theta_R);
                 out = (theta_L + theta_R) / 2.0;
                 // out = 969696969;
             }
             else
             {
                 out = (theta_L + theta_R) / 2.0;
                 // out = theta_L;
             }

             out = theta_R;
         }

         // Display heading
         dispFloat(out, display, 500);
     }*/
}