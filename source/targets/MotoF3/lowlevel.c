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
const SPIConfig HSSpiConfig = {
  NULL,
  GPIOB,
  12,
  0,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
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
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_DISABLED, NULL},
                                 {EXT_CH_MODE_RISING_EDGE|EXT_CH_MODE_AUTOSTART|EXT_MODE_GPIOA, MPUISR}
                                }};
/*0x00902025
 * 0x00E0257A
 * CR1 1
 * CR2 0x020100D0
 */
static const I2CConfig i2cconfig = {
  0x00E0257A,
  0,
  0
};

void InitHardware()
{
  // Pin configuration for motors/ESCs
  palSetPadMode(GPIOA, 4, PAL_MODE_ALTERNATE(2)); // motor 1 -> PA4 : TIM3 CH2
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(2)); // motor 2 -> PA6 : TIM3 CH1
  palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATE(2)); // motor 3 -> PB0 : TIM3 CH3
  palSetPadMode(GPIOB, 1, PAL_MODE_ALTERNATE(2)); // motor 4 -> PB1 : TIM3 CH4

  // Timer 3 used for all motors
  pwmStart(&PWMD3, &pwmcfg);
  PWMD3.tim->CR1=0;  // Disable counter
  PWMD3.tim->CNT=0;  // Reset timer counter

  // Enable timer 1 Channel 0
  pwmEnableChannel(&PWMD3, 1, THROTTLE_MIN);
  pwmEnableChannel(&PWMD3, 0, THROTTLE_MIN);
  pwmEnableChannel(&PWMD3, 2, THROTTLE_MIN);
  pwmEnableChannel(&PWMD3, 3, THROTTLE_MIN);

  PWMD3.tim->CCMR2|=(7<<4)|(7<<12);  // PWM mode 2
  PWMD3.tim->CCMR2&=~((1<<3)|(1<<11)); // Clear OC1PE
  PWMD3.tim->CCMR1|=(7<<4)|(7<<12);  // PWM mode 2
  PWMD3.tim->CCMR1&=~((1<<3)|(1<<11)); // Clear OC1PE

  /*
   * I2C I/O pins setup.
   */
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(4));
  i2cStart(&I2CD2, &i2cconfig);

#if LOG
  /*
   * SPI2 I/O pins setup.
   */
  palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(5) |PAL_STM32_OSPEED_HIGHEST);       /* New SCK.     */
  palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(5) |PAL_STM32_OSPEED_HIGHEST);       /* New MISO.    */
  palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);       /* New MOSI.    */
  palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);       /* New CS.      */
  palSetPad(GPIOB, 12);
#endif
  /*
   *  LEDs settings
   */

  palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL); /* SCK. */
  TURN_LED_OFF();
#if BUZZER
  /*
   * UART Inverter settings
   */
  palSetPadMode(GPIOA, 0, PAL_MODE_OUTPUT_PUSHPULL); /* SCK. */
  palClearPad(GPIOA, 0);
#endif
  /*
   * UART Receiver pin
   */
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(7)); /* UART 6 RX. */

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
