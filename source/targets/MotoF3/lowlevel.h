/*
 * lowlevel.h
 *
 *  Created on: Jan 17, 2016
 *      Author: Pablo
 *
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
