/*
  *  This file is part of ChibiFlight.

    ChibiFlight is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ChibiFlight is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ChibiFlight.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "config.h"
#include "mcuconf.h"
#include "chprintf.h"
#if BOARD == SPARKY2
#include "drivers/mpu9250.h"
#endif
#if BOARD == MOTOF3
#include "drivers/mpu6050.h"
#endif
#include "drivers/spektrum.h"
#include "flash.h"
#include "filter.h"
#include "lowlevel.h"
#include "usbcfg.h"

/***********************************/
/* Global variables in this module */
/***********************************/

#if LOG
uint16_t LogData[(PAGE_SIZE*PAGES_IN_LOG)/sizeof(uint16_t)]; // Buffer for the Log data
uint16_t *LogDataPtr;                                        // Pointer to log data
uint8_t CurrentLogPage;                                      // Current Log page
uint8_t EntryInPage;                                         // Current Entry in page

// A log entry has this structure:
struct LogEntry_t
  {
    int16_t RCTarget[NUMBER_OF_CHANNELS+1];
    int16_t GyroRoll;
    int16_t GyroPitch;
    int16_t GyroYaw;
    int16_t AxisPID_P[3];
    int16_t AxisPID_I[3];
    int16_t AxisPID_D[3];
    int16_t motor[4];
  } LogEntry;

#endif

static binary_semaphore_t MPUDataReady; /* Semaphore fro the MPU thread */

// PID Values
int16_t P[3]= {PID_R_P_0, PID_P_P_0, PID_Y_P_0};
int16_t I[3]= {PID_R_I_0, PID_P_I_0, PID_Y_I_0};
int16_t D[3]= {PID_R_D_0, PID_P_D_0, PID_Y_D_0};
int32_t ErrorI[3]= {0, 0, 0};
int32_t LastError[3]= {0, 0, 0};
int32_t axisPID[3];

// Motor/ESC outputs
int16_t motor[4];
#if DEBUG_MODE
int32_t axisPID_P[3];
int32_t axisPID_I[3];
int32_t axisPID_D[3];
#endif


typedef struct motorMixer_t
  {
    int32_t throttle;
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
  } motorMixer_t;

/* Mixer for QuadX
 * Motor numbering is teh same as in BaseFlight/CleanFlight:
 *      Motor 1: Rear Right
 *      Motor 2: Front Right
 *      Motor 3: Rear Left
 *      Motor 4: Front Left
 */

static const motorMixer_t mixerQuadX[]= {
                                         {1.0, -1.0, -1.0, -1.0},        // REAR_R
                                         {1.0, -1.0, 1.0, 1.0},          // FRONT_R
                                         {1.0, 1.0, -1.0, 1.0},          // REAR_L
                                         {1.0, 1.0, 1.0, -1.0},          // FRONT_L
                                        };

/***********************************/
/* Extern declared variables       */
/***********************************/
extern THD_WORKING_AREA(waReceiverThread, 1024);
extern THD_WORKING_AREA(LogThread_wa, 512);

#if DEBUG_MODE
extern uint8_t Frame[16];
extern uint16_t ReceiverData[14];
extern struct SensorGyroData RawGyroData;
#endif
extern struct SensorGyroData RawGyroData;
extern volatile int16_t RCTarget[NUMBER_OF_CHANNELS+1];
extern volatile bool Armed;
extern int16_t GyroData[];


/***********************************/
/* Routines                        */
/***********************************/

#if LOG

uint16_t GyroLogData[(PAGE_SIZE*PAGES_IN_LOG)/sizeof(uint16_t)]; // Buffer for the Log data
uint16_t *GyroLogDataPtr;                                        // Pointer to log data
uint8_t CurrentGyroLogPage=0;                                      // Current Log page
uint8_t GyroEntryInPage=0;                                         // Current Entry in page
#define GYRO_ENTRIES_PER_PAGE   (256/4)
void AddGyroEntry(void)
{
  uint8_t *Ptr;
  int16_t *Ptr16;

  Ptr=(uint8_t *) GyroLogData;
  uint8_t *PageToWrite;
  if (GyroEntryInPage==GYRO_ENTRIES_PER_PAGE)
    {
      PageToWrite=Ptr+CurrentLogPage*PAGE_SIZE;
      GyroEntryInPage=0;
      CurrentGyroLogPage++;
      CurrentGyroLogPage%= PAGES_IN_LOG;
      WritePage(PageToWrite);
    }
  Ptr+=CurrentGyroLogPage*PAGE_SIZE+GyroEntryInPage*4;
  Ptr16=(int16_t *)Ptr;
//  *Ptr16++=RawGyroData.roll;
//  *Ptr16++=RawGyroData.pitch;
  *Ptr16++=RawGyroData.yaw;
//  *Ptr16++=GyroData[ROLL];
//  *Ptr16++=GyroData[PITCH];
  *Ptr16++=GyroData[YAW];
  GyroEntryInPage++;
}
/* AddLogEntry:
 * Adds a log entry to the current LogPage
 * If the current page is full it will write it to flash
 */

static void AddLogEntry(void)
{
  uint8_t *Ptr;
  Ptr=(uint8_t *) LogData;
  uint8_t *PageToWrite;
  if (EntryInPage==ENTRIES_PER_PAGE)
    {
      PageToWrite=Ptr+CurrentLogPage*PAGE_SIZE;
      EntryInPage=0;
      CurrentLogPage++;
      CurrentLogPage%= PAGES_IN_LOG;
      WritePage(PageToWrite);
    }
  Ptr+=CurrentLogPage*PAGE_SIZE+EntryInPage*sizeof(struct LogEntry_t);
  memcpy((Ptr), &LogEntry, sizeof(struct LogEntry_t));
  EntryInPage++;
}
#endif
#if BUZZER
 binary_semaphore_t BuzzerSemaphore; /* Semaphore for the Buzzer thread */
/* LogThread:
 * Thread of the flash log.
 */

THD_WORKING_AREA(BuzzerThread_wa, 512);
THD_FUNCTION(BuzzerThread, arg)
  {
    (void) arg;
    chRegSetThreadName("Buzzer");
    extern volatile bool FailSafeState;

    chBSemObjectInit(&BuzzerSemaphore, TRUE); /* Semaphore initialization before use */

    while (TRUE)
      {
        chBSemWait(&BuzzerSemaphore);
        while((RCTarget[AUX2_CH]<-100) || FailSafeState)
          {
            BEEP_ON();
            chThdSleepMilliseconds(500);
            BEEP_OFF();
            chThdSleepMilliseconds(500);
          }
      }
  }

#endif
#if DEBUG_MODE
#if CH_DBG_THREADS_PROFILING
void cmd_threads(BaseSequentialStream *chp)
  {
    static const char *states[] = {CH_STATE_NAMES};
    thread_t *tp;
    uint32_t TotalTime;

    chprintf(chp, "%10s\t\t%10s\t%6s\t%6s\t%11s\t%7s\t%17s \r\n", "add",
        "stack", "prio", "refs", "state", "time", "name");
    tp = chRegFirstThread();
    TotalTime = 0;
    do
      {
        TotalTime += (uint32_t) tp->p_time;
        tp = chRegNextThread(tp);
      }
    while (tp != NULL);
    tp = chRegFirstThread();
    do
      {
        chprintf(
            chp,
            "%.10lx\t\t%.10lx\t%6lu\t%6lu\t%11s\t%7lu\t%17.15s\t%d\t%d\t%d \r\n",
            (uint32_t) tp, (uint32_t) tp->p_ctx.r13, (uint32_t) tp->p_prio,
            (uint32_t) (0) /*(tp->p_refs - 1)*/, states[tp->p_state],
            (uint32_t) tp->p_time, (uint32_t) tp->p_name, TotalTime,
            (uint32_t) ((((float) (tp->p_time)) / ((float) (TotalTime))) * 100),
            chVTGetSystemTime());
        tp = chRegNextThread(tp);
      }
    while (tp != NULL);
  }
#endif
static void outputmotors(BaseSequentialStream *chp)
  {
    chprintf(chp, "motor FL : %u FR: %u\r\n", motor[3]/CLK_PER_MICRO,motor[1]/CLK_PER_MICRO);
    chprintf(chp, "motor RL : %u RR : %u\r\n", motor[2]/CLK_PER_MICRO,motor[0]/CLK_PER_MICRO);
  }

static THD_WORKING_AREA(waThread3, 1024);
static void Thread3(void *arg)
  {
    (void) arg;
    chRegSetThreadName("Statistics");

    while (TRUE)
      {
        chThdSleepMilliseconds(1000);
        if (SDU1.config->usbp->state == USB_ACTIVE)
          {
            chprintf(
                (BaseSequentialStream*) &SDU1,
                "Ch1:  %04X Ch2:  %04X Ch3:  %04X Ch4:  %04X Ch5:  %04X Ch6:  %04X Ch7:  %04X \n",
                ReceiverData[0], ReceiverData[1], ReceiverData[2],
                ReceiverData[3], ReceiverData[4], ReceiverData[5],
                ReceiverData[6]);
            chprintf(
                (BaseSequentialStream*) &SDU1,
                "Ch8:  %04X Ch9:  %04X Ch10: %04X Ch11: %04X Ch12: %04X Ch13: %04X Ch14: %04X \n",
                ReceiverData[7], ReceiverData[8],
                ReceiverData[9], ReceiverData[10], ReceiverData[11],
                ReceiverData[12], ReceiverData[13]);
            chprintf(
                (BaseSequentialStream*) &SDU1,
                "TARGET Throttle: %i Roll: %i Pitch: %i Yaw: %i Aux1: %i Aux2: %i\n",
                RCTarget[0]/CLK_PER_MICRO, RCTarget[1], RCTarget[2],
                RCTarget[3], RCTarget[4], RCTarget[5]);
            chprintf((BaseSequentialStream*) &SDU1, "GYRO Roll: %d Pitch: %d Yaw: %d\n",
                     GyroData[ROLL], GyroData[PITCH], GyroData[YAW]);
            outputmotors((BaseSequentialStream*) &SDU1);
#if CH_DBG_THREADS_PROFILING
            cmd_threads((BaseSequentialStream*) &SDU1);
#endif
          }
      }
  }

#endif

/* MPUISR: Interrupt Service Routine for the Gyro
 * Sets the MPU semaphore
 */
#if TIME_METER
time_measurement_t IntTime;
time_measurement_t FilterTime;
time_measurement_t LoopTime;
time_measurement_t SampleTime;
#endif
#if TIME_METER
bool FirstInterrupt  = TRUE;
#endif
void MPUISR(EXTDriver *extp, expchannel_t channel)
{
  (void) extp;
  (void) channel;
#if TIME_METER
  if (FirstInterrupt)
    FirstInterrupt = FALSE;
  else
    chTMStopMeasurementX(&SampleTime);
  chTMStartMeasurementX(&IntTime);
#endif
  chSysLockFromISR();
  chBSemSignalI(&MPUDataReady);
  chSysUnlockFromISR();
#if TIME_METER
  chTMStartMeasurementX(&SampleTime);
#endif
}

/* ResetPID:
 * Restes all PID globals
 */

void ResetPID()
{
  ErrorI[0]=0;
  ErrorI[1]=0;
  ErrorI[2]=0;
  axisPID[0]=0;
  axisPID[1]=0;
  axisPID[2]=0;

  LastError[0]=0;
  LastError[1]=0;
  LastError[2]=0;
#if DEBUG_MODE
  axisPID_P[0] = 0;
  axisPID_P[1] = 0;
  axisPID_P[2] = 0;
  axisPID_I[0] = 0;
  axisPID_I[1] = 0;
  axisPID_I[2] = 0;
  axisPID_D[0] = 0;
  axisPID_D[1] = 0;
  axisPID_D[2] = 0;
#endif
}

/* Constrain:
 * Returns the 'Input' value constrained between 'Low' and 'High'
 */

static int32_t Constrain(int32_t Input, int32_t Low, int32_t High)
{
  if (Input<Low)
    return Low;
  else
    if (Input>High)
      return High;
    else
      return Input;
}



float DTermFilterIns[2*3]  ={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float DTermFilterOuts[2*3] ={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* SetPIDValues:
 * Selects one of the 3 PID sets depending on AUX1 Channel
 * Called only when arming. PIDs can not be changed if armed.
 */

void SetPIDValues(void)
{
  if (RCTarget[AUX1_CH]>100)
    {
      P[0]= PID_R_P_0;
      I[0]= PID_R_I_0;
      D[0]= PID_R_D_0;
      P[1]= PID_P_P_0;
      I[1]= PID_P_I_0;
      D[1]= PID_P_D_0;
      P[2]= PID_Y_P_0;
      I[2]= PID_Y_I_0;
      D[2]= PID_Y_D_0;
    }
  else
    if (RCTarget[AUX1_CH]<-100)
      {
        P[0]= PID_R_P_2;
        I[0]= PID_R_I_2;
        D[0]= PID_R_D_2;
        P[1]= PID_P_P_2;
        I[1]= PID_P_I_2;
        D[1]= PID_P_D_2;
        P[2]= PID_Y_P_2;
        I[2]= PID_Y_I_2;
        D[2]= PID_Y_D_2;
      }
    else
      {
        P[0]= PID_R_P_1;
        I[0]= PID_R_I_1;
        D[0]= PID_R_D_1;
        P[1]= PID_P_P_1;
        I[1]= PID_P_I_1;
        D[1]= PID_P_D_1;
        P[2]= PID_Y_P_1;
        I[2]= PID_Y_I_1;
        D[2]= PID_Y_D_1;
      }
}

#define GYRO_I_MAX   256

/* pid:
 * PID controller
 * Based in Alex Khoroshko's PID controller for MultiWii
 * PID controller '1' in CleanFlight
 */

static void pid(int16_t *RCCommand)

{

  int axis;
  int16_t RateError;
  int32_t Delta;
  int32_t PTerm, ITerm, DTerm;

  // ----------PID controller----------
  for (axis=0; axis<3; axis++)
    {
      RateError=RCCommand[axis]-GyroData[axis];

      // -----calculate P component
      PTerm=(RateError*P[axis])>>4;

      // -----calculate I component
      ErrorI[axis]=ErrorI[axis]+((RateError*MICROS_PER_LOOP)>>8)*I[axis];

      // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
      // I coefficient (I8) moved before integration to make limiting independent from PID settings
      ErrorI[axis]=Constrain(ErrorI[axis], (int32_t) -GYRO_I_MAX<<13, (int32_t) +GYRO_I_MAX<<13);
      ITerm=ErrorI[axis]>>14;

      //-----calculate D-term
      Delta=RateError-LastError[axis];
      LastError[axis]=RateError;
//      Delta=0-GyroData[axis]-LastError[axis];
//      LastError[axis]=0-GyroData[axis];
      Delta=(Delta*((uint16_t) 0xFFFF/(MICROS_PER_LOOP>>4)))>>4;

      DTerm=(Delta * D[axis])>>6;
      DTerm=Filter2ndOrder(DTerm,
                           &DTermFilterIns[2*axis],
                           &DTermFilterOuts[2*axis],
                           Butter2_03_ACoeffs,
                           Butter2_03_BCoeffs);
      // -----calculate total PID output
      axisPID[axis]=PTerm+ITerm+DTerm;
#if LOG
        LogEntry.AxisPID_P[axis] = (int16_t)((PTerm*10));
        LogEntry.AxisPID_I[axis] = (int16_t)((ITerm*100));
        LogEntry.AxisPID_D[axis] = (int16_t)((DTerm*100));
#endif

    }
}
 /* MixTable
  * Calculates individual motor values based on the ouptus of
  * the pid controller and the current throttle
  */

static void MixTable(int16_t *RCCommand)
{
  int16_t maxMotor;
  uint32_t i;
  for (i=0; i<4; i++)
    {
      motor[i]=RCCommand[THROTTLE_CH]*mixerQuadX[i].throttle
          +axisPID[PITCH]*mixerQuadX[i].pitch+axisPID[ROLL]*mixerQuadX[i].roll
          +axisPID[YAW]*mixerQuadX[i].yaw;
    }

  maxMotor=motor[0];
  for (i=1; i<4; i++)
    if (motor[i]>maxMotor)
      maxMotor=motor[i];

  if (maxMotor>THROTTLE_MAX)
    for (i=0; i<4; i++)
      motor[i]-=maxMotor-THROTTLE_MAX;

  for (i=0; i<4; i++)
    if (motor[i]<THROTTLE_MIN_PID)
      motor[i]= THROTTLE_MIN_PID;
}

/* MPUThread:
 * Main thread that runs the control loop
 * Waits for new MPU Values, gets the current RC Command
 * calls PID, Mixer an sets the values to the motors/ESCs
 *
 */

static THD_WORKING_AREA(MPUThread_wa, 1024);
static THD_FUNCTION(MPUThread, p)
{

  (void) p;
#if BOARD == SPARKY2
  register volatile uint32_t *Motors12Enable;
  register volatile uint32_t *Motors34Enable;
#endif
#if BOARD == MOTOF3
  register volatile uint32_t *MotorsEnable;
#endif
  register volatile uint32_t *Motor1Counter;
  register volatile uint32_t *Motor2Counter;
  register volatile uint32_t *Motor3Counter;
  register volatile uint32_t *Motor4Counter;
  int16_t LocalRCTarget[NUMBER_OF_CHANNELS+1];
  chRegSetThreadName("MPU Thread");
  chBSemObjectInit(&MPUDataReady, TRUE); /* Semaphore initialization*/
  StartCalibration(); /* Gyro calibration */
#if TIME_METER
  chTMObjectInit(&FilterTime);
  chTMObjectInit(&LoopTime);
  chTMObjectInit(&IntTime);
  chTMObjectInit(&SampleTime);
#endif

  if (MPUInit()!=0) /* Gyro initialization */
    {
      while (TRUE)
        {
          TOGGLE_LED();                  // Gyro Sensor not found
          chThdSleepMilliseconds(1000);  // Toggle LED forever
        }
    }
  extStart(&EXTD1, &ExternalInterruptConfig); /* External interrupt setup */
#if BOARD == SPARKY2
  // Motor 4 on timer 1 Slot 4
  Motors12Enable=&(PWMD3.tim->CR1);        // Pointer to motor 1&2 enable
  Motors34Enable=&(PWMD5.tim->CR1);        // Pointer to motor 3&4 enable
  // Motor counters
  Motor1Counter=&(PWMD3.tim->CCR[2]);    // Pointer to motor 1 counter
  Motor2Counter=&(PWMD3.tim->CCR[3]);    // Pointer to motor 2 counter
  Motor3Counter=&(PWMD5.tim->CCR[3]);    // Pointer to motor 3 counter
  Motor4Counter=&(PWMD5.tim->CCR[2]);    // Pointer to motor 4 counter
#endif
#if BOARD == MOTOF3
  // Motor 4 on timer 1 Slot 4
  MotorsEnable=&(PWMD3.tim->CR1);        // Pointer to motor 1&2 enable
  // Motor counters
  Motor1Counter=&(PWMD3.tim->CCR[1]);    // Pointer to motor 1 counter
  Motor2Counter=&(PWMD3.tim->CCR[0]);    // Pointer to motor 2 counter
  Motor3Counter=&(PWMD3.tim->CCR[2]);    // Pointer to motor 3 counter
  Motor4Counter=&(PWMD3.tim->CCR[3]);    // Pointer to motor 4 counter
#endif

  while (true)
    {
      do
        {
          chBSemWait(&MPUDataReady);    // Wait until new Gyro Data is available
        }
      while (GetMPUData()==FALSE);                     // Read the data
#if TIME_METER
      chTMStartMeasurementX(&LoopTime);
#endif
      chSysLock();               // Get the data from receiver under SysLock
                                 // to avoid getting values from different frames
                                 // Loop or memcpy would be nicer, single assigns are faster
                                 // using a mutex is also "nicer". SysLock is faster.
      LocalRCTarget[THROTTLE_CH]=RCTarget[THROTTLE_CH];
      LocalRCTarget[AILERON_CH]=RCTarget[AILERON_CH];
      LocalRCTarget[ELEVATOR_CH]=RCTarget[ELEVATOR_CH];
      LocalRCTarget[RUDDER_CH]=RCTarget[RUDDER_CH];
      LocalRCTarget[AUX1_CH]=RCTarget[AUX1_CH];
      LocalRCTarget[AUX2_CH]=RCTarget[AUX2_CH];
      LocalRCTarget[STATUS_CH]=RCTarget[STATUS_CH];
      chSysUnlock();                        // Unlock
      if (!Armed)
        {
#if CALIBRATE_ESC
            {
              extern int16_t CalibrationCycles;

              motor[0] = (CalibrationCycles) ? THROTTLE_MAX :THROTTLE_MIN;
              motor[1] = (CalibrationCycles) ? THROTTLE_MAX :THROTTLE_MIN;
              motor[2] = (CalibrationCycles) ? THROTTLE_MAX :THROTTLE_MIN;
              motor[3] = (CalibrationCycles) ? THROTTLE_MAX :THROTTLE_MIN;
            }
#else
          motor[0]= THROTTLE_MIN;        // not armed.
          motor[1]= THROTTLE_MIN;
          motor[2]= THROTTLE_MIN;
          motor[3]= THROTTLE_MIN;
#endif
        }
      else
        {
//        if ((LocalRCTarget[THROTTLE_CH]>THROTTLE_MIN)/*|| // Armed and throttle higher than minimun
//            (LocalRCTarget[AUX2_CH]>0)*/)
          if(1)
            {
#if LOG
              LogEntry.RCTarget[THROTTLE_CH]=LocalRCTarget[THROTTLE_CH];
              LogEntry.RCTarget[AILERON_CH]=LocalRCTarget[AILERON_CH];
              LogEntry.RCTarget[ELEVATOR_CH]=LocalRCTarget[ELEVATOR_CH];
              LogEntry.RCTarget[RUDDER_CH]=LocalRCTarget[RUDDER_CH];
              LogEntry.RCTarget[AUX1_CH]=LocalRCTarget[AUX1_CH];
              LogEntry.RCTarget[AUX2_CH]=LocalRCTarget[AUX2_CH];
              LogEntry.RCTarget[STATUS_CH]=LocalRCTarget[STATUS_CH];
              LogEntry.GyroRoll=(int16_t) GyroData[ROLL];
              LogEntry.GyroPitch=(int16_t) GyroData[PITCH];
              LogEntry.GyroYaw=(int16_t) GyroData[YAW];

#endif
              if (LocalRCTarget[THROTTLE_CH]==THROTTLE_MIN)
                LocalRCTarget[RUDDER_CH]=0;   // Disable YAW while arming / Disarming
              pid(&LocalRCTarget[ROLL_CH]);   // Calculate PIDs
              MixTable(LocalRCTarget);        // Calculate Motor outputs
#if LOG
              LogEntry.motor[0]=motor[0];
              LogEntry.motor[1]=motor[1];
              LogEntry.motor[2]=motor[2];
              LogEntry.motor[3]=motor[3];
              AddLogEntry();
#endif

            }
          else
            {
              ResetPID();                 // Armed but Throttle below min
              motor[0]= THROTTLE_MIN_PID; // REset PIDs and set motors to min
              motor[1]= THROTTLE_MIN_PID;
              motor[2]= THROTTLE_MIN_PID;
              motor[3]= THROTTLE_MIN_PID;
            }
        }
      SET_MOTOR(Motor1Counter, motor[0]);      // Set motor counters
      SET_MOTOR(Motor2Counter, motor[1]);
      SET_MOTOR(Motor3Counter, motor[2]);
      SET_MOTOR(Motor4Counter, motor[3]);
#if BOARD == SPARKY2
      START_MOTORS(Motors12Enable, Motors34Enable); // Enable timers -> Start motors
#endif
#if BOARD == MOTOF3
      START_MOTORS(MotorsEnable); // Enable timers -> Start motors
#endif
#if TIME_METER
      chTMStopMeasurementX(&LoopTime);
#endif

    }
}

/*
 * main thread
 * Initialize some stuff and starts the other threads
 */

int main(void)
{
  halInit();
  chSysInit();

  /*
   *  Timers
   */
  InitHardware();

#if LOG
  LogDataPtr=LogData;
  CurrentLogPage=0;
#endif

#if BUZZER
  // Start Log Thread: Lowest prio
  chThdCreateStatic(BuzzerThread_wa, sizeof(BuzzerThread_wa),
                    NORMALPRIO,BuzzerThread, NULL);
#endif
#if LOG
  // Start Log Thread: Lowest prio
  chThdCreateStatic(LogThread_wa, sizeof(LogThread_wa),
                    NORMALPRIO +1,LogThread, NULL);
#endif
  // Start MPU Thread: Middle prio
  chThdCreateStatic(MPUThread_wa, sizeof(MPUThread_wa),
                    NORMALPRIO +5,MPUThread, NULL);
  // Start receiver thread: Highest prio
  chThdCreateStatic(waReceiverThread, sizeof(waReceiverThread),
                    NORMALPRIO +10, ReceiverThread, NULL);
#if DEBUG_MODE
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3,
      NULL);
#endif

  /*
   * Normal main() thread activity, it does nothing.
   */
  while (TRUE)
    chThdSleep(TIME_INFINITE);
  return 0;
}
