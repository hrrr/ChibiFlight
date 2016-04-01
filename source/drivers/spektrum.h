/*
 * spektrum.h
 *
 *  Created on: Apr 25, 2015
 *      Author: Pablo
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

#ifndef SPEKTRUM_H_
#define SPEKTRUM_H_


#define DSM_MASK                (0x7FF)
#define DSM_CHANNELS_PER_FRAME  7
#define DSM_FRAME_LENGTH        (1+1+DSM_CHANNELS_PER_FRAME*2)
#define DSM_DSM2_RES_MASK       0x0010
#define DSM_2ND_FRAME_MASK      0x8000

#define NUMBER_OF_CHANNELS      6

//extern  uint8_t Frame[16];
//extern  uint32_t ReceiverData[14];

THD_FUNCTION(ReceiverThread, arg);
//extern THD_WORKING_AREA(waReceiverThread, 512);

#define THROTTLE_CH           0
#define AILERON_CH            1
#define ROLL_CH               1
#define ELEVATOR_CH           2
#define PITCH_CH              2
#define RUDDER_CH             3
#define YAW_CH                3
#define AUX1_CH               4
#define AUX2_CH               5
#define STATUS_CH             6

#define RC_MIN                0x100
#define RC_NEUTRAL            0x400
#define RC_MAX                0x700
#define YAW_DEADBAND          0x30


#define ARM_FRAMES            20

#define UNKNOWN                0xFFFFFFFF
#define ARM                    1
#define DISARM                 2
#define CALIBRATE              3
#define RECORDING              4
#define READING                5

#endif /* SPEKTRUM_H_ */
