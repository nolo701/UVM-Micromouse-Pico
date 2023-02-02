
// Left
#define S1_SH 8
#define S1_INT 9
#define S1_ADD 0x30

//Forward
#define S2_SH 0
#define S2_INT 1
#define S2_ADD 0x31

//Right
#define S3_SH 4
#define S3_INT 5
#define S3_ADD 0x32

#define R_F 3
#define R_R 2

#define L_F 7
#define L_R 6

#define ENC_R 10
#define ENC_L 11

#ifndef STDLIB_H
#include "pico/stdlib.h"
#endif

#ifndef PWM_H
#include "hardware/pwm.h"
#endif

// includes for VL53L1X
#ifndef VL53L1X_TYPES_H
#include "pico/binary_info.h"
extern "C"
{
#include "../VL53L1X/Library/public/include/VL53L1X_api.h"
#include "../VL53L1X/Library/public/include/VL53L1X_types.h"
}
#endif


