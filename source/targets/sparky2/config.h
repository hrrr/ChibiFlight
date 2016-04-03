/*
 * config.h
 *
 *  Created on: May 21, 2015
 *      Author: Pablo
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define SPARKY2         1
#define MOTOF3          2

#define BOARD           SPARKY2

/* GENERAL SETTINGS */

//Debug mode outputs some statistics over USB Serial
#define DEBUG_MODE       FALSE
#define AIRMODE          TRUE
// Calibrate OneShot ESC to 125-250 range
#define CALIBRATE_ESC    FALSE
// Log enables the code to save the flight log on the on board spi flash
#define LOG              TRUE
#define BUZZER           TRUE
#define TIME_METER       FALSE

#define BYTES_PER_ENTRY  sizeof(struct LogEntry_t) //(7+3+3*3+4)*2 = 46
#define PAGE_SIZE        256
#define PAGES_IN_LOG     2
#define ENTRIES_PER_PAGE (PAGE_SIZE/BYTES_PER_ENTRY)

#define FLASH_SIZE         (16*1024*1024/8)
#define LAST_FLASH_ADDRESS (FLASH_SIZE-1)

/* TIMER SETTINGS */

//CLK Per microsecond of the timers to generate the one shot pulses
//N CLKS per microsecond = N Mhz
#define CLK_PER_MICRO  12
#define COUNTER_FREQ   (CLK_PER_MICRO * 1000000)
#define COUNTER_WRAP   ((250+10)*CLK_PER_MICRO)


/* GYRO */
#define GYRO_LPF              GYRO_LPF_3600
// Gyro sampling rate.
#define GYRO_RATE             8000               // in herz
// This is the update rate to the motors
// Equivalent to loop time in Multiwii or cleanfligh
#define LOOP_RATE             2000
#define GYRO_RANGE            GYRO_RANGE_2000

#define SAMPLES_PER_LOOP    (GYRO_RATE/LOOP_RATE)

#define PID_R_P_0    18
#define PID_R_I_0    10
#define PID_R_D_0    15

#define PID_P_P_0    18
#define PID_P_I_0    10
#define PID_P_D_0    15

#define PID_Y_P_0    30
#define PID_Y_I_0    10
#define PID_Y_D_0     0

#define PID_R_P_1    18
#define PID_R_I_1    10
#define PID_R_D_1    18

#define PID_P_P_1    18
#define PID_P_I_1    10
#define PID_P_D_1    18

#define PID_Y_P_1    30
#define PID_Y_I_1    10
#define PID_Y_D_1     0

#define PID_R_P_2    20
#define PID_R_I_2    10
#define PID_R_D_2    22

#define PID_P_P_2    20
#define PID_P_I_2    10
#define PID_P_D_2    22

#define PID_Y_P_2    35
#define PID_Y_I_2    10
#define PID_Y_D_2     0
/* TARGET MAX RATES */
// THROTTLE_MIN_PID is the minimal throttle given to the motors when armed
#define THROTTLE_MIN_PID      (132 * CLK_PER_MICRO)
#define THROTTLE_MIN          (125 * CLK_PER_MICRO)
#define THROTTLE_MAX          (250 * CLK_PER_MICRO)

// Rates in degrees/second
#define ROLL_RATE             500
#define PITCH_RATE            500
#define YAW_RATE              400
// AUX RATE has no unit
#define AUX_RATE              1000

/* MACROS */

// Set the value to the timer to generate the oneshot pulse
#define SET_MOTOR(ptr, value)    (*ptr = (COUNTER_WRAP - value))

// Starts the timers to generate the oneshot pulses
// Timers set up in single pulse mode
#define START_MOTORS(En1,En2)   do { \
                                     *En1 = (1 << 0) | (1 << 2) | (1 << 3) | (1 << 7); \
                                     *En2 = (1 << 0) | (1 << 2) | (1 << 3) | (1 << 7); \
                                   } while (FALSE)



#endif /* CONFIG_H_ */
