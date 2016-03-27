/*
 * mpu9250.h
 *
 *  Created on: May 5, 2015
 *      Author: Pablo
 */

#ifndef MPU9250_H_
#define MPU9250_H_

/*
 * Register names according to the datasheet.
 * According to the InvenSense document
 * "MPU-9250 and MPU-9250 Register Map
 * and Descriptions Revision 3.2", there are no registers
 * at 0x02 ... 0x18, but according other information
 * the registers in that unknown area are for gain
 * and offsets.
 */

#define MPU9250_SMPLRT_DIV           25   // R/W
#define MPU9250_CONFIG               26   // R/W
#define MPU9250_GYRO_CONFIG          27   // R/W
#define MPU9250_ACCEL_CONFIG         28   // R/W

#define MPU9250_INT_PIN_CFG          55   // R/W
#define MPU9250_INT_ENABLE           56   // R/W

#define MPU9250_GYRO_XOUT_H          67   // R
#define MPU9250_GYRO_XOUT_L          68   // R
#define MPU9250_GYRO_YOUT_H          69   // R
#define MPU9250_GYRO_YOUT_L          70   // R
#define MPU9250_GYRO_ZOUT_H          71   // R
#define MPU9250_GYRO_ZOUT_L          72   // R

#define MPU9250_SIGNAL_PATH_RESET   104   // R/W
#define MPU9250_USER_CTRL           106   // R/W
#define MPU9250_PWR_MGMT_1          107   // R/W
#define MPU9250_WHO_AM_I            117   // R

/*
 * Values of the different configurations
 *
 */

#define GYRO_LPF_250          0
#define GYRO_LPF_184          1
#define GYRO_LPF_92           2
#define GYRO_LPF_41           3
#define GYRO_LPF_20           4
#define GYRO_LPF_10           5
#define GYRO_LPF_5            6
#define GYRO_LPF_3600         7

#define GYRO_RANGE_250        (0<<3)
#define GYRO_RANGE_500        (1<<3)
#define GYRO_RANGE_1000       (2<<3)
#define GYRO_RANGE_2000       (3<<3)

#if (GYRO_RANGE == GYRO_RANGE_2000)
#define GYRO_SCALE            (-1.0f / 16.4f)
#define GYRO_SCALE_INT        (-16)
#elif (GYRO_RANGE == GYRO_RANGE_1000)
#define GYRO_SCALE            (-1.0f / 32.8f)
#define GYRO_SCALE_INT        (-32)
#elif (GYRO_RANGE == GYRO_RANGE_500)
#define GYRO_SCALE            (-1.0f / 65.5f)
#define GYRO_SCALE_INT        (-64)
#elif (GYRO_RANGE == GYRO_RANGE_250)
#define GYRO_SCALE            (-1.0f / 131.0f)
#define GYRO_SCALE_INT        (-128)
#else
#error  Wrong Gyro range
#endif

#if (LOOP_RATE == 2000)
#define CYCLE_TIME            (0.00005f)
#define MICROS_PER_LOOP       500
#elif (LOOP_RATE == 1000)
#define CYCLE_TIME            (0.0001f)
#define MICROS_PER_LOOP       1000
#elif (LOOP_RATE == 500)
#define CYCLE_TIME            (0.0002f)
#define MICROS_PER_LOOP       2000
#elif (LOOP_RATE == 250)
#define CYCLE_TIME            (0.0004f)
#define MICROS_PER_LOOP       4000
#else
#error  Wrong Gyro sample rate
#endif


/*
 * According to the data sheet the device should return
 * 0x71 as reply to Who am I.
 */
#define MPU9250_WHO_AM_I_REPLY 0x71

#if CALIBRATE_ESC
#define CALIB_CYCLES   (4000*8)
#else
#define CALIB_CYCLES   (2048*8)
#endif

#define X_AXIS         0
#define Y_AXIS         1
#define Z_AXIS         2
#define ROLL           0
#define PITCH          1
#define YAW            2

struct SensorGyroData {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
};
int MPUInit(void);
bool GetMPUData(void);
void StartCalibration(void);
void ResetPID(void);
void ClearFilters(void);
void DisableMPUInt(void);
#endif /* MPU9250_H_ */
