/*
 * spektrum.c
 *
 *  Created on: Apr 25, 2015
 *       Author: Pablo
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

#include <drivers/spektrum.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "config.h"
#include "drivers/mpu9250.h"
#if LOG
#include "flash.h"
#endif
#include "lowlevel.h"

extern void SetPIDValues(void);

/***********************************/
/* Global variables in this module */
/***********************************/

uint8_t Frame[16];
uint16_t ReceiverData[14];
volatile int16_t RCTarget[NUMBER_OF_CHANNELS+1];
volatile bool Armed;
#if BUZZER
volatile bool FailSafeState;
#endif
uint32_t ReceiverState;
uint32_t OldReceiverState;
uint32_t StateCounter;

static const SerialConfig MyConfig =
{
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

#if BUZZER
    extern binary_semaphore_t BuzzerSemaphore; /* Semaphore for the Buzzer thread */
#endif

/***********************************/
/* Routines                        */
/***********************************/

/* Map:
 * Maps an input value 'In' with range InMin-InMax
 * to an output value (returned) between OutMin and OutMax
 */

static int16_t Map(int16_t In, int16_t InMin, int16_t InMax, int16_t OutMin,
                   int16_t OutMax)
  {
    if (In <= InMin)
      return OutMin;
    if (In >= InMax)
      return OutMax;
    return (In - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
  }

/* ReceiverFSM:
 * Software Finite state machine that defines the state
 * of the system according to stick commads
 * States areaArmed, disarmed, etc...
 */

static void ReceiverFSM(void)
{

   if (Armed && (ReceiverData[THROTTLE_CH] <= RC_MIN) && (ReceiverData[YAW_CH] <= RC_MIN) &&
       (ReceiverData[PITCH_CH] < RC_MAX))
     ReceiverState = DISARM;
   else if (!Armed && (ReceiverData[THROTTLE_CH] <= RC_MIN) && (ReceiverData[YAW_CH] >= RC_MAX) &&
       (ReceiverData[PITCH_CH] < RC_MAX))
     ReceiverState = ARM;
   else if (!Armed && (ReceiverData[THROTTLE_CH] <= RC_MIN) && (ReceiverData[YAW_CH] <= RC_MIN) &&
       (ReceiverData[PITCH_CH] >= RC_MAX))
     ReceiverState = CALIBRATE;
#if LOG
   else if (!Armed && (ReceiverData[THROTTLE_CH] >= RC_MAX) && (ReceiverData[YAW_CH] >= RC_MAX) &&
       (ReceiverData[PITCH_CH] < RC_MAX))
     ReceiverState = RECORDING;
   else if (!Armed && (ReceiverData[THROTTLE_CH] >= RC_MAX) && (ReceiverData[YAW_CH] <= RC_MIN) &&
       (ReceiverData[PITCH_CH] < RC_MAX))
     ReceiverState = READING;
#endif
   else
     {
       ReceiverState = UNKNOWN;
       StateCounter = 0;
     }
    if (ReceiverState == OldReceiverState)
      {
        StateCounter++;
        if (StateCounter >= ARM_FRAMES)
          switch (ReceiverState)
            {
            case ARM:
              //Arm();
              SetPIDValues();
              Armed = TRUE;
              TURN_LED_ON();
              ResetPID();
              StateCounter = 0;
              break;
            case DISARM:
              Armed = FALSE;
              TURN_LED_OFF();
              StateCounter = 0;
              //Disarm();
              break;
            case CALIBRATE:
              StartCalibration();
              StateCounter = 0;
              //Calibrate();
              break;
#if LOG
            case RECORDING:
              StartRecording();
              StateCounter = 0;
              //Calibrate();
              break;
            case READING:
              StartReading();
              StateCounter = 0;
              //Calibrate();
              break;
#endif
            }
      }
    else
      {
        OldReceiverState = ReceiverState;
        StateCounter = 0;
      }
}

/* DecodeFrame:
 * Decodes a receiver data frame received in 'Frame'
 * Outputs the new data in 'RCTarget'
 * returns TRUE if OK, FALSE if frame error
 */

static bool DecodeFrame(uint8_t *Frame)
  {
    uint32_t ByteInFrame, i;
    uint16_t Word;
    uint8_t ChannelNumber;

    ByteInFrame = 2;
    for (i = 0; i < DSM_CHANNELS_PER_FRAME; i++)
      {
        Word = ((uint16_t) Frame[ByteInFrame] << 8)
            | Frame[ByteInFrame + 1];
        ByteInFrame += 2;
        /* skip empty channel slot */
        if (Word == 0xffff)
          continue;

        /* minimal data validation */
        if ((i > 0) && (Word & DSM_2ND_FRAME_MASK))
          {
            /* invalid frame data, ignore rest of the frame */
            return FALSE;
          }
        /* extract and save the channel value */
        ChannelNumber = (Word >> 11) & 0x0f;
        ReceiverData[ChannelNumber] = (Word & DSM_MASK);
      }
    ReceiverFSM();
    chSysLock(); // SysLock to have mutual exclusion with MPU Thread
                 // that is using this data
    RCTarget[THROTTLE_CH] = Map(ReceiverData[THROTTLE_CH], RC_MIN, RC_MAX, THROTTLE_MIN,
                                THROTTLE_MAX);
    RCTarget[AILERON_CH] = Map(ReceiverData[AILERON_CH], RC_MIN, RC_MAX,
                               -PITCH_RATE, PITCH_RATE);
    RCTarget[ELEVATOR_CH] = Map(ReceiverData[ELEVATOR_CH], RC_MIN, RC_MAX,
                                -ROLL_RATE, ROLL_RATE);
    RCTarget[RUDDER_CH] = Map(ReceiverData[RUDDER_CH], RC_MIN, RC_MAX,
                              -YAW_RATE, YAW_RATE);
    RCTarget[AUX1_CH] = Map(ReceiverData[AUX1_CH], RC_MIN, RC_MAX, -AUX_RATE,
                            AUX_RATE);
    RCTarget[AUX2_CH] = Map(ReceiverData[AUX2_CH], RC_MIN, RC_MAX, -AUX_RATE,
                            AUX_RATE);
    RCTarget[STATUS_CH]++;// = Armed;
    chSysUnlock();
#if BUZZER
    if (RCTarget[AUX2_CH]<-100)
      chBSemSignal(&BuzzerSemaphore);
#endif
    return TRUE;
  }

/* FailSafeHandling:
 * Timer interrupt service routine for the
 * failsafe timer.
 * In case of Failsafe just disarm.
 */

void FailSafeHandling(void *arg)
  {
    (void) arg;
    RCTarget[THROTTLE_CH] = THROTTLE_MIN;
    Armed = FALSE;
    TURN_LED_OFF();
#if BUZZER
    FailSafeState = TRUE;
    chBSemSignal(&BuzzerSemaphore);
#endif
  }

/* ReceiverThread:
 *
 */

THD_WORKING_AREA(waReceiverThread, 1024);
THD_FUNCTION(ReceiverThread, arg)
  {
    (void) arg;
    chRegSetThreadName("SerialReceiver");
    unsigned int ByteInFrame;
    bool GotFrame;
    systime_t LastByteTime, CurrentTime;
    msg_t ByteRead;
    virtual_timer_t FailsafeTimer;

    chVTObjectInit(&FailsafeTimer);  // virtual timer used for failsafe handling
    memset(ReceiverData, 0xff, sizeof(ReceiverData));
    memset((void *)RCTarget, 0, sizeof(RCTarget));

    sdStart(&RECEIVER_UART, &MyConfig);  // Serial port to receiver
    Armed = FALSE;

    ReceiverState = UNKNOWN;
    OldReceiverState = UNKNOWN;
    StateCounter = 0;
    LastByteTime= 0;
#if BUZZER
    FailSafeState = FALSE;
#endif

    while (TRUE)
      {
        GotFrame = FALSE;
        ByteInFrame = 0;
        while (!GotFrame)
          {
            ByteRead = chnGetTimeout(&RECEIVER_UART, TIME_INFINITE); // Get next byte from serial port
            CurrentTime = chVTGetSystemTime();
            Frame[ByteInFrame] = (uint8_t) ByteRead;
            if (ByteInFrame == 0)
              {
                LastByteTime = CurrentTime;
                ByteInFrame = 1;
              }
            else
              if (ST2MS((systime_t)(CurrentTime-LastByteTime)) > 5)
                { // More than 5ms from last byte: We lost some bytes. Restart frame counter.
                  Frame[0] = Frame[ByteInFrame];
                  LastByteTime = CurrentTime;
                  ByteInFrame = 1;
                }
              else
                {
                  ByteInFrame++;
                  LastByteTime = CurrentTime;
                  if (ByteInFrame == DSM_FRAME_LENGTH)
                    GotFrame = TRUE;  // Whole frame received
                }
          }
        if (DecodeFrame(Frame))  // Decode frame
          {
            chVTSet(&FailsafeTimer, MS2ST(500), FailSafeHandling, 0); // re-start failsafe timer
#if BUZZER
            FailSafeState = FALSE;
#endif
          }

      }
  }

