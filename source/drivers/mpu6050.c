/*
 * mou6000.c
 *
 *  Created on: Apr 26, 2015
 *      Author: Pablo
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "config.h"
#include "mpu6050.h"
#include "filter.h"
#include "lowlevel.h"

/***********************************/
/* Global variables in this module */
/***********************************/

enum {
        IDX_GYRO_XOUT_H = 0,
        IDX_GYRO_XOUT_L,
        IDX_GYRO_YOUT_H,
        IDX_GYRO_YOUT_L,
        IDX_GYRO_ZOUT_H,
        IDX_GYRO_ZOUT_L,
        GYRO_DATA_SIZE,
    };

uint8_t GyroReadDataBuffer[GYRO_DATA_SIZE];


struct SensorGyroData RawGyroData;
int32_t CalibrationCycles = 0;
int32_t GyroBias[3];
int16_t GyroData[3] = {0,0,0};




/***********************************/
/* Routines                        */
/***********************************/

/* ReadGyroData:
 * Access SPI bus and read the gyro sensor
 * Data is stored in 'GyroReadDataBuffer'
 */

static int ReadGyroData(void)
  {
    uint8_t reg = MPU6050_GYRO_XOUT_H;
//    i2cAcquireBus(&I2CD2);
    (void)i2cMasterTransmitTimeout(&I2CD2, MPU6050_I2C_ADDR, &reg, 1, GyroReadDataBuffer, GYRO_DATA_SIZE, TIME_INFINITE);
//    i2cReleaseBus(&I2CD2);
    return 0;
  }

static uint8_t MPU6050ReadRegister(uint8_t reg)
{
    uint8_t data;
//    i2cAcquireBus(&I2CD2);
    (void)i2cMasterTransmitTimeout(&I2CD2, MPU6050_I2C_ADDR, &reg, 1, &data, 1, TIME_INFINITE);
//    i2cReleaseBus(&I2CD2);
    return data;
}
static void MPU6050WriteRegister(uint8_t reg, uint8_t val)
{
    uint8_t cmd[2] = {reg, val};
//    i2cAcquireBus(&I2CD2);
    (void)i2cMasterTransmitTimeout(&I2CD2, MPU6050_I2C_ADDR, cmd, 2, NULL, 0, TIME_INFINITE);
//    i2cReleaseBus(&I2CD2);
}

static void MPU6050Reset(void)
  {
    MPU6050WriteRegister(MPU6050_PWR_MGMT_1, 0x80);
    chThdSleepMilliseconds(100);
    MPU6050WriteRegister(MPU6050_SIGNAL_PATH_RESET, 0x7);
    chThdSleepMilliseconds(100);
  }

static uint8_t MPU6050GetWhoAmI(void)
  {
    if (MPU6050ReadRegister(MPU6050_WHO_AM_I) != MPU6050_WHO_AM_I_REPLY)
      {
        return -1;
      }
    return 0;
  }

static void MPU6050SetSampleRate(uint16_t samplerate_hz)
  {
    uint16_t filter_frequency = 8000;

    if (MPU6050ReadRegister(MPU6050_CONFIG) != 0)
      filter_frequency = 1000;

    // limit samplerate to filter frequency
    if (samplerate_hz > filter_frequency)
      samplerate_hz = filter_frequency;

    // calculate divisor, round to nearest integeter
    int32_t divisor = (int32_t) (((float) filter_frequency / samplerate_hz)
        + 0.5f) - 1;

    // limit resulting divisor to register value range
    if (divisor < 0)
      divisor = 0;

    if (divisor > 0xff)
      divisor = 0xff;

    MPU6050WriteRegister(MPU6050_SMPLRT_DIV, (uint8_t) divisor);
  }

int MPUInit(void)
  {
    uint8_t retval = 0;

    /*
     * Reset the device.
     */
    MPU6050Reset();

    retval = MPU6050GetWhoAmI();
    //  PLL with Z axis gyroscope reference
    MPU6050WriteRegister(MPU6050_PWR_MGMT_1, 0x3);
    // LPF 256
    MPU6050WriteRegister(MPU6050_CONFIG, GYRO_LPF);
    MPU6050SetSampleRate(GYRO_RATE*SAMPLES_PER_LOOP);
    // Gyro range 1000
    MPU6050WriteRegister(MPU6050_GYRO_CONFIG, GYRO_RANGE);
    // Accel Range 8G
    MPU6050WriteRegister(MPU6050_ACCEL_CONFIG, (2<<3));
    // Interrupt configuration
    MPU6050WriteRegister(MPU6050_INT_PIN_CFG, 0x10);
    // Interrupt configuration
    MPU6050WriteRegister(MPU6050_INT_ENABLE, 0x01);

     return retval;
  }

void DisableMPUInt(void)
{
  //MPU9250WriteRegister(MPU9250_INT_ENABLE, 0x00);
}

/* StartCalibration:
 * Satrts the Gyro calibration
 */

void StartCalibration(void)
  {
    ResetPID();
    if (CalibrationCycles == 0)
      CalibrationCycles = CALIB_CYCLES;
  }

float GyroFilterIns[2*3]  ={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float GyroFilterOuts[2*3] ={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};


/* GetMPUData:
 * Get raw data from sensor
 * substract Sensor bias, scale the data
 * and gives the new data in 'GyroData'
 */
uint8_t SampleCounter = 0;

bool GetMPUData(void)
{
#if TIME_METER
    {
      extern time_measurement_t FilterTime;
      chTMStartMeasurementX(&FilterTime);
    }
#endif
  ReadGyroData();
#if TIME_METER
    {
      extern time_measurement_t FilterTime;
      chTMStopMeasurementX(&FilterTime);
    }
#endif
#if TIME_METER
    {
      extern time_measurement_t IntTime;

      chSysLock();
      chTMStopMeasurementX(&IntTime);
      chSysUnlock();
    }
#endif
  RawGyroData.roll=(int16_t) (GyroReadDataBuffer[IDX_GYRO_XOUT_H]<<8
      |GyroReadDataBuffer[IDX_GYRO_XOUT_L]);
  RawGyroData.roll/= GYRO_SCALE_INT;
  RawGyroData.pitch=(int16_t) (GyroReadDataBuffer[IDX_GYRO_YOUT_H]<<8
      |GyroReadDataBuffer[IDX_GYRO_YOUT_L]);
  RawGyroData.pitch/= (-1*GYRO_SCALE_INT);
  RawGyroData.yaw=(int16_t) (GyroReadDataBuffer[IDX_GYRO_ZOUT_H]<<8
      |GyroReadDataBuffer[IDX_GYRO_ZOUT_L]);
  RawGyroData.yaw/= GYRO_SCALE_INT;
#if 0//TIME_METER
    {
      extern time_measurement_t FilterTime;
      chTMStartMeasurementX(&FilterTime);
    }
#endif
  GyroData[ROLL]=Filter2ndOrder(RawGyroData.roll,
                                &(GyroFilterIns[0]),
                                &(GyroFilterOuts[0]),
                                &(Butter2_16_ACoeffs[0]),
                                &(Butter2_16_BCoeffs[0]));
  GyroData[PITCH]=Filter2ndOrder(RawGyroData.pitch,
                                 &(GyroFilterIns[2]),
                                 &(GyroFilterOuts[2]),
                                 &(Butter2_16_ACoeffs[0]),
                                 &(Butter2_16_BCoeffs[0]));
  GyroData[YAW]=Filter2ndOrder(RawGyroData.yaw,
                               &(GyroFilterIns[4]),
                               &(GyroFilterOuts[4]),
                               &(Butter2_16_ACoeffs[0]),
                               &(Butter2_16_BCoeffs[0]));
#if 0//TIME_METER
    {
      extern time_measurement_t FilterTime;
      chTMStopMeasurementX(&FilterTime);
    }
#endif
  if (CalibrationCycles==0)
    {
      GyroData[ROLL]-=GyroBias[ROLL];
      GyroData[PITCH]-=GyroBias[PITCH];
      GyroData[YAW]-=GyroBias[YAW];
    }
  else
    if (CalibrationCycles==CALIB_CYCLES)
      {
        TURN_LED_ON();
        GyroBias[ROLL]=0;
        GyroBias[PITCH]=0;
        GyroBias[YAW]=0;
        CalibrationCycles--;
      }
    else // (CalibrationCycles > 0)
      {
        GyroBias[ROLL]+=GyroData[ROLL];
        GyroBias[PITCH]+=GyroData[PITCH];
        GyroBias[YAW]+=GyroData[YAW];
        if (CalibrationCycles==1)
          {
            TURN_LED_OFF();
            GyroBias[ROLL]/= CALIB_CYCLES;
            GyroBias[PITCH]/= CALIB_CYCLES;
            GyroBias[YAW]/= CALIB_CYCLES;
          }
        GyroData[ROLL]=0;
        GyroData[PITCH]=0;
        GyroData[YAW]=0;
        CalibrationCycles--;
      }
  SampleCounter++;
#if 0 //Used to log gyro data only
    {
      extern void AddGyroEntry(void);
      extern volatile bool Armed;
      if (Armed)
      AddGyroEntry();
    }
#endif
  if ((SampleCounter%SAMPLES_PER_LOOP)==0)
    return TRUE;
  else
    return FALSE;

}
