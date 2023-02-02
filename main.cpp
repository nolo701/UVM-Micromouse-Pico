#include "mouse/define.h"
#include "mouse/mouse.h"

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
    sleep_ms(500);
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
    sleep_ms(500);
    display.clear();
    display.sendBuffer();
}

// Interrupt functions & setup
static int ticksR = 10;
static int ticksL = 10;
bool led = true;

void updateEncoderR(void)
{
    gpio_acknowledge_irq(ENC_R, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    ticksR++;
    gpio_put(25, led);
    led = !led;
}

void updateEncoderL(uint pin, uint32_t event)
{
    // gpio_acknowledge_irq(ENC_L, GPIO_IRQ_EDGE_RISE);
    ticksL++;
    ticksR--;
    gpio_put(25, led);
    led = !led;
}

int main()
{
    // Pin & Variable Declarations

    uint8_t Sensor1_Shutdown = S1_SH;
    uint8_t Sensor1_GPIO = S1_INT;
    uint8_t Sensor1_address = S1_ADD;
    uint8_t Sensor2_Shutdown = S2_SH;
    uint8_t Sensor2_GPIO = S2_INT;
    uint8_t Sensor2_address = S2_ADD;
    uint8_t Sensor3_Shutdown = S3_SH;
    uint8_t Sensor3_GPIO = S3_INT;
    uint8_t Sensor3_address = S3_ADD;

    uint8_t Right_F = R_F;
    uint8_t Right_R = R_R;

    uint8_t Left_F = L_F;
    uint8_t Left_R = L_R;

    uint8_t Encode_L = ENC_L;
    uint8_t Encode_R = ENC_R;

    // GPIO Setup
    gpio_init(Sensor1_GPIO);
    gpio_set_dir(Sensor1_GPIO, GPIO_IN);
    gpio_init(Sensor1_Shutdown);
    gpio_set_dir(Sensor1_Shutdown, GPIO_OUT);
    // Shutdown is active low so set it high
    gpio_put(Sensor1_Shutdown, 1);

    gpio_init(Sensor2_GPIO);
    gpio_set_dir(Sensor2_GPIO, GPIO_IN);
    gpio_init(Sensor2_Shutdown);
    gpio_set_dir(Sensor2_Shutdown, GPIO_OUT);
    // Shutdown is active low so set it high
    gpio_put(Sensor2_Shutdown, 1);

    gpio_init(Sensor3_GPIO);
    gpio_set_dir(Sensor3_GPIO, GPIO_IN);
    gpio_init(Sensor3_Shutdown);
    gpio_set_dir(Sensor3_Shutdown, GPIO_OUT);
    // Shutdown is active low so set it high
    gpio_put(Sensor3_Shutdown, 1);

    // Enable PWM - Left
    // https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=1
    // https://www.etechnophiles.com/raspberry-pi-pico-pinout-specifications-datasheet-in-detail/
    gpio_set_function(Left_F, GPIO_FUNC_PWM);
    uint8_t Left_slice = pwm_gpio_to_slice_num(Left_F);
    uint8_t Left_F_channel = pwm_gpio_to_channel(Left_F);
    gpio_set_function(Left_R, GPIO_FUNC_PWM);
    uint8_t Left_R_channel = pwm_gpio_to_channel(Left_R);
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
    uint8_t Right_slice = pwm_gpio_to_slice_num(Right_F);
    uint8_t Right_F_channel = pwm_gpio_to_channel(Right_F);
    gpio_set_function(Right_R, GPIO_FUNC_PWM);
    uint8_t Right_R_channel = pwm_gpio_to_channel(Right_R);
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

    mouse Speedy;
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
    SSD1306 display = SSD1306(i2c0, 0x3C, Size::W128xH32);
    sleep_ms(500);

    // Here we rotate the display by 180 degrees, so that it's not upside down from my perspective
    // If your screen is upside down try setting it to 1 or 0
    display.setOrientation(1);
    display.clear();
    display.sendBuffer();
    sleep_ms(200);

    // Sensor Reading here
    VL53L1X_Status_t status1;
    VL53L1X_Result_t results1; // this is a structure with a lot of data
    VL53L1X_Status_t status2;
    VL53L1X_Result_t results2; // this is a structure with a lot of data
    VL53L1X_Status_t status3;
    VL53L1X_Result_t results3; // this is a structure with a lot of data

    //  ------  Init Sensor 1 ------
    gpio_put(Sensor1_Shutdown, 1);
    // turn off others
    gpio_put(Sensor2_Shutdown, 0);
    gpio_put(Sensor3_Shutdown, 0);
    sleep_ms(100);
    setupSensor(Sensor1_address, display);

    //  ------  Init Sensor 2 ------
    gpio_put(Sensor2_Shutdown, 1);
    sleep_ms(100);
    setupSensor(Sensor2_address, display);

    //  ------  Init Sensor 3 ------
    gpio_put(Sensor3_Shutdown, 1);
    sleep_ms(100);
    setupSensor(Sensor3_address, display);

    // Enable all three sensors
    VL53L1X_StartRanging(Sensor1_address);
    VL53L1X_StartRanging(Sensor2_address);
    VL53L1X_StartRanging(Sensor3_address);

    drawText(&display, font_5x8, "Ranging on", 0, 0);
    display.sendBuffer();
    sleep_ms(500);
    display.clear();
    display.sendBuffer();

    uint16_t dist1 = 0;
    uint16_t dist2 = 0;
    uint16_t dist3 = 0;

    uint8_t dataReady1;
    uint8_t dataReady2;
    uint8_t dataReady3;

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

    bool moving = false;
    Speedy.rampUp();
    while(1){
        Speedy.move(100);
        int a = Speedy.moveStraight(100, ticksL, ticksR);
        //drawText(&display, font_8x8, (std::to_string(a*1000)).data(), 0, 0);
        //display.sendBuffer();
        //sleep_ms(1000);
        //display.clear();
        //display.sendBuffer();
        //sleep_ms(1000);

    }







    // Test encoders
    while (1)
    {
        drawText(&display, font_8x8, (std::to_string(ticksL)).data(), 0, 0);
        drawText(&display, font_8x8, (std::to_string(ticksR)).data(), 0, 10);
        display.sendBuffer();
        sleep_ms(5);
        display.clear();
        display.sendBuffer();

        /* while((ticksL<900)&&(ticksR<900)){

                 Speedy.move(100);
         }

         Speedy.stop();*/
    }

    // main loop to drive forward
    while (1)
    {
        // update sensors
        VL53L1X_GetDistance(Sensor1_address, &dist1);
        VL53L1X_GetDistance(Sensor2_address, &dist2);
        VL53L1X_GetDistance(Sensor3_address, &dist3);
        drawText(&display, font_8x8, ("1: " + std::to_string(dist1)).data(), 0, 0);
        drawText(&display, font_8x8, ("2: " + std::to_string(dist2)).data(), 0, 8);
        drawText(&display, font_8x8, ("3: " + std::to_string(dist3)).data(), 8, 16);
        display.sendBuffer();
        // check to see if the distance forward is valid if so move forward
        if ((dist2 > 80) && (!moving))
        {
            // move forward
            drawText(&display, font_12x16, "Moving!", 0, 0);
            display.sendBuffer();
            sleep_ms(200);
            // Speedy.move(50, 50);

            moving = true;
        }
        else if ((dist2 > 80) && (moving))
        {
            // try to maintain straight
            Speedy.straighten(100, dist1, dist2);
        }
        else
        {
            // stop
            Speedy.stop();
            drawText(&display, font_12x16, "Stopping!", 0, 0);
            display.sendBuffer();
            sleep_ms(3000);
            moving = false;
        }

        VL53L1X_ClearInterrupt(Sensor1_address);
        VL53L1X_ClearInterrupt(Sensor2_address);
        VL53L1X_ClearInterrupt(Sensor3_address);
        display.clear();
        display.sendBuffer();
        // sleep_ms(1000);
    }

    while (1)
    {
        VL53L1X_CheckForDataReady(Sensor1_address, &dataReady1);
        VL53L1X_CheckForDataReady(Sensor2_address, &dataReady2);
        VL53L1X_CheckForDataReady(Sensor3_address, &dataReady3);
        sleep_ms(10);
        // check to see if sensor 1 has a value
        if (dataReady1 == 1)
        {
            VL53L1X_GetDistance(Sensor1_address, &dist1);
            drawText(&display, font_5x8, ("1: " + std::to_string(dist1)).data(), 0, 0);
        }
        if (dataReady2 == 1)
        {
            VL53L1X_GetDistance(Sensor2_address, &dist2);
            drawText(&display, font_5x8, ("2: " + std::to_string(dist2)).data(), 5, 5);
        }
        if (dataReady3 == 1)
        {
            VL53L1X_GetDistance(Sensor3_address, &dist3);
            drawText(&display, font_5x8, ("3: " + std::to_string(dist3)).data(), 10, 10);
        }
        display.sendBuffer();
        display.clear();
        VL53L1X_ClearInterrupt(Sensor1_address);
        VL53L1X_ClearInterrupt(Sensor2_address);
        VL53L1X_ClearInterrupt(Sensor3_address);
        sleep_ms(500);
        display.sendBuffer();
    }

    while (1)
    {
        // move forward on right for 1 second
        Speedy.move(0, 100);
        sleep_ms(1000);
        Speedy.move(0, -100);
        sleep_ms(1000);
        // pause
        Speedy.move(0);
        sleep_ms(500);
        // move forward on left for 1/2 second
        Speedy.move(100, 0);
        sleep_ms(500);
        Speedy.move(-100, 0);
        sleep_ms(500);
        // pause
        Speedy.move(0);
        sleep_ms(500);
    }

    // Measure and print continuously
    bool first_range = true;
    while (1)
    {
        // Wait until we have new data
        uint8_t dataReady1;
        do
        {
            status1 = VL53L1X_CheckForDataReady(Sensor1_address, &dataReady1);
            sleep_us(1);
        } while (dataReady1 == 0);

        // Read and display result
        status1 += VL53L1X_GetResult(Sensor1_address, &results1);
        // printf("Status = %2d, dist = %5d, Ambient = %2d, Signal = %5d, #ofSpads = %5d\n",
        //        results.status, results.distance, results.ambient, results.sigPerSPAD, results.numSPADs);
        drawText(&display, font_12x16, std::to_string(results1.distance).data(), 0, 0);
        display.sendBuffer();
        // Clear the sensor for a new measurement
        status1 += VL53L1X_ClearInterrupt(Sensor1_address);
        if (first_range)
        { // Clear twice on first measurement
            status1 += VL53L1X_ClearInterrupt(Sensor1_address);
            first_range = false;
        }
        sleep_ms(10);
        display.clear();
        // display.sendBuffer();
    }
}