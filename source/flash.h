/*
 * flash.h
 *
 *  Created on: May 25, 2015
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

#ifndef FLASH_H_
#define FLASH_H_

#define M25P16_RDID             0x9F
#define M25P16_READ_BYTES       0x03
#define M25P16_READ_BYTES_HS    0x0B
#define M25P16_READ_STATUS_REG  0x05
#define M25P16_WRITE_STATUS_REG 0x01
#define M25P16_WRITE_ENABLE     0x06
#define M25P16_WRITE_DISABLE    0x04
#define M25P16_PAGE_PROGRAM     0x02
#define M25P16_SECTOR_ERASE     0xD8
#define M25P16_BULK_ERASE       0xC7

#define NO_OPCODE              0x00
#define RECORDING_OPCODE       0x01
#define READING_OPCODE         0x02
#define WRITEPAGE_OPCODE       0x03

#define FLASH_IDLE                      0
#define FLASH_ERASING                   1
#define FLASH_WAIT_FOR_PAGE             2
#define FLASH_WRITING                   3
#define FLASH_READING                   4
#define FLASH_FINISHED                  5


THD_FUNCTION(LogThread, arg);
void StartRecording(void);
void StartReading(void);
void WritePage(uint8_t *Buffer);
#endif /* FLASH_H_ */
