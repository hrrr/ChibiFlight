/*
 * mpu9250.c
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
#include "mpu9250.h"
#include "filter.h"

/***********************************/
/* Global variables in this module */
/***********************************/

enum {
        IDX_SPI_DUMMY_BYTE = 0,
        IDX_GYRO_XOUT_H,
        IDX_GYRO_XOUT_L,
        IDX_GYRO_YOUT_H,
        IDX_GYRO_YOUT_L,
        IDX_GYRO_ZOUT_H,
        IDX_GYRO_ZOUT_L,
        GYRO_DATA_SIZE,
    };

uint8_t GyroReadCommandBuffer[GYRO_DATA_SIZE] = { MPU9250_GYRO_XOUT_H | 0x80, 0, 0, 0, 0, 0, 0 };
uint8_t GyroReadDataBuffer[GYRO_DATA_SIZE];


struct SensorGyroData RawGyroData;
int32_t CalibrationCycles = 0;
int32_t GyroBias[3];
int16_t GyroData[3] = {0,0,0};

/*
 * Maximum speed SPI configuration (10.5MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig HSSpiConfig =
  {
  NULL,
  GPIOC,
  4,
  SPI_CR1_BR_1
  };

/*
 * Low speed SPI configuration (281.250kHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig LSSpiConfig =
  {
  NULL,
  GPIOC,
  4,
  SPI_CR1_BR_2 | SPI_CR1_BR_1
  };

/***********************************/
/* Routines                        */
/***********************************/

/* ReadGyroData:
 * Access SPI bus and read the gyro sensor
 * Data is stored in 'GyroReadDataBuffer'
 */

static int ReadGyroData(void)
  {
    GyroReadCommandBuffer[0] = MPU9250_GYRO_XOUT_H | 0x80;
    spiStart(&SPID1, &HSSpiConfig); /* Setup transfer parameters.       */
    spiSelect(&SPID1); /* Slave Select assertion.          */
    spiExchange(&SPID1, GYRO_DATA_SIZE, GyroReadCommandBuffer, GyroReadDataBuffer); /* Atomic transfer operations.      */
    spiUnselect(&SPID1); /* Slave Select de-assertion.       */
    return 0;
  }

/* MPU9250ReadRegister:
 * Access SPI bus and read a register of the MPU9250 sensor
 * Register address is given in 'Address'. Read value is returned.
 */

static uint8_t MPU9250ReadRegister(uint8_t Address)
  {
    uint8_t RxBuffer[3];
    uint8_t TxBuffer[3];

    TxBuffer[0] = Address | 0x80;
    TxBuffer[1] = 0;
    TxBuffer[2] = 0;
    spiStart(&SPID1, &LSSpiConfig); /* Setup transfer parameters.       */
    spiSelect(&SPID1); /* Slave Select assertion.          */
    spiExchange(&SPID1, 3, TxBuffer, RxBuffer); /* Atomic transfer operations.      */
    spiUnselect(&SPID1); /* Slave Select de-assertion.       */
    return RxBuffer[1];
  }

/* MPU9250WriteRegister:
 * Access SPI bus and writes a register of the MPU9250 sensor
 * Register address is given in 'Address'.
 * Value to be written to the register is given in 'Value'.
 */

static void MPU9250WriteRegister(uint8_t Address, uint8_t Value)
  {
    uint8_t TxBuffer[2];

    TxBuffer[0] = Address;
    TxBuffer[1] = Value; // 8kHz => 19 (dec) devider = 400Hz
    spiStart(&SPID1, &LSSpiConfig); /* Setup transfer parameters.       */
    spiSelect(&SPID1); /* Slave Select assertion.          */
    spiSend(&SPID1, 2, TxBuffer); /* Send command                     */
    spiUnselect(&SPID1); /* Slave Select de-assertion.       */
    spiStop(&SPID1);
  }

/* MPU9250Reset:
 * Resets the MPU9250 sensor
 */

static void MPU9250Reset(void)
  {
    MPU9250WriteRegister(MPU9250_PWR_MGMT_1, 0x80);
    chThdSleepMilliseconds(100);
    MPU9250WriteRegister(MPU9250_SIGNAL_PATH_RESET, 0x7);
    chThdSleepMilliseconds(100);
  }

/* MPU9250GetWhoAmI:
 * Reads the 'Who am I' register
 * Returns 0 if values is as expected
 * or -1 in case of error
 */

static uint8_t MPU9250GetWhoAmI(void)
  {
    if (MPU9250ReadRegister(MPU9250_WHO_AM_I) != MPU9250_WHO_AM_I_REPLY)
      {
        return -1;
      }
    return 0;
  }

/* MPU9250SetSampleRate:
 * Sets the sample rate of the MPU9250 sensor
 * This is going to be the sample rate of the
 * MPU9250 interrupt, and thus, the loop rate
 */

static void MPU9250SetSampleRate(uint16_t SampleRateHz)
  {
    uint16_t FilterFreq = 1000;
    int32_t Divisor;
    uint8_t Config;

    Config = MPU9250ReadRegister(MPU9250_CONFIG);
    if ((Config == GYRO_LPF_250) || (Config == GYRO_LPF_3600))
      return;

    // limit samplerate to filter frequency
    if (SampleRateHz > FilterFreq)
      SampleRateHz = FilterFreq;

    // calculate divisor, round to nearest integeter
    Divisor = (int32_t) (((float) FilterFreq / SampleRateHz)
        + 0.5f) - 1;

    // limit resulting divisor to register value range
    if (Divisor < 0)
      Divisor = 0;

    if (Divisor > 0xff)
      Divisor = 0xff;

    MPU9250WriteRegister(MPU9250_SMPLRT_DIV, (uint8_t) Divisor);
  }

/* MPU9250Init:
 * Initialization of the MPU600 sensor
 * Returns 0 if OK, -1 if error (sensor not found).
 */

int MPU9250Init(void)
  {
    uint8_t ReturnValue;

    /*
     * Reset the device.
     */
    MPU9250Reset();

    ReturnValue = MPU9250GetWhoAmI();
    if (ReturnValue != 0)
      return ReturnValue;
    //  Select clock source
    MPU9250WriteRegister(MPU9250_PWR_MGMT_1, 0x1);
    // Disable i2c
    MPU9250WriteRegister(MPU9250_USER_CTRL, 0x10);
    // LPF
    MPU9250WriteRegister(MPU9250_CONFIG, GYRO_LPF);
    // Sample rate
    MPU9250SetSampleRate(GYRO_RATE);
    // Gyro range
    MPU9250WriteRegister(MPU9250_GYRO_CONFIG, GYRO_RANGE);
    // Accel Range 8G (ACC not used)
    MPU9250WriteRegister(MPU9250_ACCEL_CONFIG, (2<<3));
    // Interrupt configuration
    // Active, rising edge interrupt
    MPU9250WriteRegister(MPU9250_INT_PIN_CFG, 0x10);
    // Interrupt configuration
    MPU9250WriteRegister(MPU9250_INT_ENABLE, 0x01);
    return 0;
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
  ReadGyroData();
  RawGyroData.roll=(int16_t) (GyroReadDataBuffer[IDX_GYRO_YOUT_H]<<8
      |GyroReadDataBuffer[IDX_GYRO_YOUT_L]);
  RawGyroData.roll/= GYRO_SCALE_INT;
  RawGyroData.pitch=(int16_t) (GyroReadDataBuffer[IDX_GYRO_XOUT_H]<<8
      |GyroReadDataBuffer[IDX_GYRO_XOUT_L]);
  RawGyroData.pitch/= GYRO_SCALE_INT;
  RawGyroData.yaw=(int16_t) (GyroReadDataBuffer[IDX_GYRO_ZOUT_H]<<8
      |GyroReadDataBuffer[IDX_GYRO_ZOUT_L]);
  RawGyroData.yaw/= GYRO_SCALE_INT;
#if TIME_METER
    {
      extern time_measurement_t FilterTime;
      chTMStartMeasurementX(&FilterTime);
    }
#endif
  GyroData[ROLL]=Filter2ndOrder(RawGyroData.roll,
                                &(GyroFilterIns[0]),
                                &(GyroFilterOuts[0]),
                                &(Butter2_04_ACoeffs[0]),
                                &(Butter2_04_BCoeffs[0]));
  GyroData[PITCH]=Filter2ndOrder(RawGyroData.pitch,
                                 &(GyroFilterIns[2]),
                                 &(GyroFilterOuts[2]),
                                 &(Butter2_04_ACoeffs[0]),
                                 &(Butter2_04_BCoeffs[0]));
  GyroData[YAW]=Filter2ndOrder(RawGyroData.yaw,
                               &(GyroFilterIns[4]),
                               &(GyroFilterOuts[4]),
                               &(Butter2_04_ACoeffs[0]),
                               &(Butter2_04_BCoeffs[0]));
#if TIME_METER
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
#if TIME_METER
    {
      extern time_measurement_t IntTime;

      chSysLock();
      chTMStopMeasurementX(&IntTime);
      chSysUnlock();
    }
#endif
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

