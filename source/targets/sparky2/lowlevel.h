/*
 * lowlevel.h
 *
 *  Created on: Jan 17, 2016
 *      Author: Pablo
 */

#ifndef SOURCE_TARGETS_SPARKY2_LOWLEVEL_H_
#define SOURCE_TARGETS_SPARKY2_LOWLEVEL_H_

void InitHardware(void);

extern const EXTConfig ExternalInterruptConfig;
extern const SPIConfig HSSpiConfig;

#define RECEIVER_UART       SD6
#define FLASH_SPI           SPID3

#define BEEP_ON()   palSetPad(GPIOC, 9)
#define BEEP_OFF()  palClearPad(GPIOC, 9)

/* LED */
// Blue LED
#define TURN_O_LED_ON()         palClearPad(GPIOB, 4)
#define TURN_O_LED_OFF()        palSetPad(GPIOB, 4)
#define TOGGLE_O_LED()          palTogglePad(GPIOB,4)

#define TURN_LED_ON()             palClearPad(GPIOB, 5)
#define TURN_LED_OFF()          palSetPad(GPIOB, 5)
#define TOGGLE_LED()            palTogglePad(GPIOB,5)

#define TURN_B2_LED_ON()         palClearPad(GPIOB, 6)
#define TURN_B2_LED_OFF()        palSetPad(GPIOB, 6)
#define TOGGLE_B2_LED()          palTogglePad(GPIOB,6)


#endif /* SOURCE_TARGETS_SPARKY2_LOWLEVEL_H_ */
