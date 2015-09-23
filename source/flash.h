/*
 * flash.h
 *
 *  Created on: May 25, 2015
 *      Author: Pablo
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
