/*
 * lowlevel.h
 *
 *  Created on: Jan 17, 2016
 *      Author: Pablo
 */

#ifndef SOURCE_TARGETS_MOTOF3_LOWLEVEL_H_
#define SOURCE_TARGETS_MOTOF3_LOWLEVEL_H_

#define usb_lld_connect_bus(usbp)
#define usb_lld_disconnect_bus(usbp)


void InitHardware(void);

extern const EXTConfig ExternalInterruptConfig;
extern const SPIConfig HSSpiConfig;

#define RECEIVER_UART       SD2
#define FLASH_SPI           SPID2

#define BEEP_ON()   palSetPad(GPIOA, 0)
#define BEEP_OFF()  palClearPad(GPIOA, 0)

/* LED */
// Blue LED

#define TURN_LED_ON()           palClearPad(GPIOB, 5)
#define TURN_LED_OFF()          palSetPad(GPIOB, 5)
#define TOGGLE_LED()            palTogglePad(GPIOB,5)

#define TURN_O_LED_ON()         {}/*Only one led in MotoF3*/
#define TURN_O_LED_OFF()        {}/*Only one led in MotoF3*/
#define TOGGLE_O_LED()          {}/*Only one led in MotoF3*/


#define TURN_B2_LED_ON()        {}/*Only one led in MotoF3*/
#define TURN_B2_LED_OFF()       {}/*Only one led in MotoF3*/
#define TOGGLE_B2_LED()         {}/*Only one led in MotoF3*/
#endif /* SOURCE_TARGETS_MOTOF3_LOWLEVEL_H_ */
