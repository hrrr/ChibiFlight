/*
 * lowlevel.c
 *
 *  Created on: Jan 17, 2016
 *      Author: Pablo
 */
#include "ch.h"
#include "hal.h"
#include "config.h"
#include "usbcfg.h"
#include "lowlevel.h"

extern void MPUISR(EXTDriver *extp, expchannel_t channel);

/*
 * Maximum speed SPI configuration (18MHz, CPHA=0, CPOL=0, MSb first).
 */
const SPIConfig HSSpiConfig =
  {
  NULL,
  GPIOB,
  3,
  0
  };

// Timer configuration for the OneShot signals to the ESC/Motors
static PWMConfig pwmcfg= {
                          COUNTER_FREQ,
                          COUNTER_WRAP,
                          NULL, {
                                 {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                                 {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                                 {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                                 {PWM_OUTPUT_ACTIVE_HIGH, NULL}
                          },
                          0,
                          0,
#if STM32_PWM_USE_ADVANCED
                          0
#endif
                        };
// Enable Gyro Interrupt input as edge interrupt
const EXTConfig ExternalInterruptConfig= {{
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_RISING_EDGE|EXT_CH_MODE_AUTOSTART|EXT_MODE_GPIOC, MPUISR},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL}
                                }};

void InitHardware()
{
  // Pin configuration for motors/ESCs
  palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATE(2)); // motor 1 -> PB0 : TIM3 CH3
  palSetPadMode(GPIOB, 1, PAL_MODE_ALTERNATE(2)); // motor 2 -> PB1 : TIM3 CH4
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(2)); // motor 3 -> PA3 : TIM5 CH4
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(2)); // motor 4 -> PA2 : TIM5 CH3

  // Timer 5 used for all motors
  pwmStart(&PWMD5, &pwmcfg);
  PWMD5.tim->CR1=0;  // Disable counter
  PWMD5.tim->CNT=0;  // Reset timer counter

  pwmStart(&PWMD3, &pwmcfg);
  PWMD3.tim->CR1=0;  // Disable counter
  PWMD3.tim->CNT=0;  // Reset timer counter

  // Enable timer 1 Channel 0
  pwmEnableChannel(&PWMD3, 2, THROTTLE_MIN);
  pwmEnableChannel(&PWMD3, 3, THROTTLE_MIN);
  pwmEnableChannel(&PWMD5, 2, THROTTLE_MIN);
  pwmEnableChannel(&PWMD5, 3, THROTTLE_MIN);

  PWMD5.tim->CCMR2|=(7<<4)|(7<<12);  // PWM mode 2
  PWMD5.tim->CCMR2&=~((1<<3)|(1<<11)); // Clear OC1PE
  PWMD3.tim->CCMR2|=(7<<4)|(7<<12);  // PWM mode 2
  PWMD3.tim->CCMR2&=~((1<<3)|(1<<11)); // Clear OC1PE

  /*
   * SPI1 I/O pins setup.
   */

  palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5) |PAL_STM32_OSPEED_HIGHEST);       /* New SCK.     */
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5) |PAL_STM32_OSPEED_HIGHEST);       /* New MISO.    */
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);       /* New MOSI.    */
  palSetPadMode(GPIOC, 4, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);       /* New CS.      */
  palSetPad(GPIOC, 4);

#if LOG
  /*
   * SPI2 I/O pins setup.
   */
  palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(6) |PAL_STM32_OSPEED_HIGHEST);       /* New SCK.     */
  palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(6) |PAL_STM32_OSPEED_HIGHEST);       /* New MISO.    */
  palSetPadMode(GPIOC, 12, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);       /* New MOSI.    */
  palSetPadMode(GPIOB, 3, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);       /* New CS.      */
  palSetPad(GPIOB, 3);
#endif
  /*
   *  LEDs settings
   */

  palSetPadMode(GPIOB, 4, PAL_MODE_OUTPUT_PUSHPULL); /* SCK. */
  palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL); /* SCK. */
  palSetPadMode(GPIOB, 6, PAL_MODE_OUTPUT_PUSHPULL); /* SCK. */
  TURN_O_LED_OFF();
  TURN_LED_OFF();
  TURN_B2_LED_OFF();
#if BUZZER
  /*
   * UART Inverter settings
   */
  palSetPadMode(GPIOC, 9, PAL_MODE_OUTPUT_PUSHPULL); /* SCK. */
  palClearPad(GPIOC, 9);
#endif
  /*
   * UART Inverter settings
   */
  palSetPadMode(GPIOC, 6, PAL_MODE_OUTPUT_PUSHPULL); /* SCK. */
  palClearPad(GPIOC, 6);
  /*
   * UART Receiver pin
   */
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(8)); /* UART 6 RX. */

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
#if (DEBUG_MODE || LOG)
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);
#endif


}
