/*
 * flash.c
 *
 *  Created on: May 24, 2015
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

#include "config.h"
#include "ch.h"
#include "hal.h"

#include "flash.h"
#include "drivers/mpu9250.h"
#include "lowlevel.h"
#if LOG

/***********************************/
/* Global variables in this module */
/***********************************/

static binary_semaphore_t FlashSemaphore; /* Semaphore used in the log thread*/
uint32_t OpCode;      // Opcode: Codes the operation
uint8_t FlashSate;    // FlashState stores the current state of the SPI flash
uint8_t *DataBuffer;  // Pointer to the Read/Write buffer

/***********************************/
/* Extern declared variables       */
/***********************************/

extern SerialUSBDriver SDU1;
extern uint16_t LogData[];



/***********************************/
/* Routines                        */
/***********************************/

/* SPIExchangeData:
 * Exchanges 'Size' bytes from the SPI bus on 'SPIPtr'
 * 'RXBuf' is the buffer where the data is received
 * 'TxBuf' is the buffer with the data to be transmitted
 */

static int SPIExchangeData(SPIDriver *SPIPtr, uint8_t *TxBuf, uint8_t *RxBuf, size_t Size)
  {
    spiStart(SPIPtr, &HSSpiConfig); /* Setup transfer parameters.       */
    spiSelect(SPIPtr); /* Slave Select assertion.          */
    spiExchange(SPIPtr, Size, TxBuf, RxBuf); /* Atomic transfer operations.      */
    spiUnselect(SPIPtr); /* Slave Select de-assertion.       */
    return 0;
  }

/* SPISendData:
 * Sends 'Size' bytes on the SPI bus on 'SPIPtr'
 * 'TxBuf' is the buffer with the data to be transmitted
 */

static int SPISendData(SPIDriver *SPIPtr, uint8_t *TxBuf, size_t Size)
  {
    spiStart(SPIPtr, &HSSpiConfig); /* Setup transfer parameters.       */
    spiSelect(SPIPtr); /* Slave Select assertion.          */
    spiSend(SPIPtr, Size, TxBuf); /* Send command                     */
    spiUnselect(SPIPtr); /* Slave Select de-assertion.       */
    spiStop(SPIPtr);
    return 0;
  }

/* M25P16WriteRegister:
 * Sends one single byte to the M25P16 SPI flash memory
 * 'Value' is the value to be transmitted
 */

static void M25P16WriteRegister(uint8_t Value)
  {
    SPISendData(&FLASH_SPI, &Value, 1);
  }

/* M25P16ReadRegister:
 * Sends a register from  the M25P16 SPI flash memory
 * Returns teh register value
 * 'Address' is the addres of teh register
 */

static uint8_t M25P16ReadRegister(uint8_t Address)
  {
    uint8_t RxBuf[3];
    uint8_t TxBuf[3];

    TxBuf[0] = Address;
    TxBuf[1] = 0xFF;
    TxBuf[2] = 0xFF;

    SPIExchangeData(&FLASH_SPI, TxBuf, RxBuf, 3);

    return RxBuf[1];
  }


#define M25P16SetWriteEnable()        (M25P16WriteRegister(M25P16_WRITE_ENABLE))
#define M25P16SetWriteDisable()       (M25P16WriteRegister(M25P16_WRITE_DISABLE))
#define M25P16ReadStatus()            (M25P16ReadRegister(M25P16_READ_STATUS_REG))

/* M25P16ReadPage:
 * Read a whole page of data from the M25P16 SPI flash memory
 * 'Address' is the address in flash memory of the data to be read
 * The data is read into 'Buffer'
 */

static void M25P16ReadPage(uint32_t Address, uint8_t * Buffer)
  {
    uint8_t Command[] = { M25P16_READ_BYTES, (Address >> 16) & 0xFF, (Address >> 8) & 0xFF, Address & 0xFF};

    spiStart(&FLASH_SPI, &HSSpiConfig); /* Setup transfer parameters.       */
    spiSelect(&FLASH_SPI); /* Slave Select assertion.          */
    spiSend(&FLASH_SPI, 4, Command); /* Send command                     */
    spiReceive(&FLASH_SPI, PAGE_SIZE, Buffer);
    spiUnselect(&FLASH_SPI); /* Slave Select de-assertion.       */
    spiStop(&FLASH_SPI);
  }

/* M25P16WritePage:
 * Writes a whole page of data (256 Bytes) from 'Buffer' in the M25P16 SPI flash memory
 * 'Address' is the address in flash memory were the data is going to be stored
 */

static void M25P16WritePage(uint32_t Address, uint8_t * Buffer)
  {
    uint8_t Command[] = { M25P16_PAGE_PROGRAM, (Address >> 16) & 0xFF, (Address >> 8) & 0xFF, Address & 0xFF};

    M25P16SetWriteEnable();
    spiStart(&FLASH_SPI, &HSSpiConfig); /* Setup transfer parameters.       */
    spiSelect(&FLASH_SPI); /* Slave Select assertion.          */
    spiSend(&FLASH_SPI, 4, Command); /* Send command                     */
    spiSend(&FLASH_SPI, 256, Buffer);
    spiUnselect(&FLASH_SPI); /* Slave Select de-assertion.       */
    spiStop(&FLASH_SPI);
    while(M25P16ReadStatus() & 0x1)
      chThdSleepMicroseconds(100);
  }

/* M25P16BulkErase:
 * Erases the whole SPI flash memory
 */

static void M25P16BulkErase(void)
  {
    M25P16SetWriteEnable();
    M25P16WriteRegister(M25P16_BULK_ERASE);
    while(M25P16ReadStatus() & 0x1)
       chThdSleepMilliseconds(50);
  }

/* StartRecording:
 * Called from receiver thread when the user set the
 * control sticks to start recording.
 *
 * Sticks command: (accepted only if disarmed)
 *   Roll ad pitch: neutral
 *   Yaw: full to the right
 *   Throttle: Full throttle
 *
 * First step will be erasing the whole flash.
 * Watch the blue led. It will be on while erasing
 * After it goes off, it is safe to arm and fly
 */

void StartRecording(void)
  {
    if (FlashSate == FLASH_IDLE)
      {
        FlashSate = FLASH_ERASING;
        OpCode = RECORDING_OPCODE;
        chBSemSignal(&FlashSemaphore);
      }

  }

/* StartReading:
 * Called from receiver thread when the user set the
 * control sticks to start reading.
 *
 * Sticks command: (accepted only if disarmed)
 *   Roll ad pitch: neutral
 *   Yaw: full to the left
 *   Throttle: Full throttle
 *
 * It will read the whole flash.
 * and send the content on the USB serial port
 * Blue LED ill be on while not finished
 */

void StartReading(void)
  {
    if (FlashSate == FLASH_IDLE)
      {
        DisableMPUInt();
        FlashSate = FLASH_READING;
        OpCode = READING_OPCODE;
        chBSemSignal(&FlashSemaphore);
      }
  }

/* WritePage:
 * Called from the MPUThread when page of log data
 * has to be written to flash.
 */

void WritePage(uint8_t *Buffer)
  {
    if (FlashSate == FLASH_WAIT_FOR_PAGE)
      {
        FlashSate = FLASH_WRITING;
        OpCode = WRITEPAGE_OPCODE;
        DataBuffer = Buffer;
        chBSemSignal(&FlashSemaphore);
      }
    else if (FlashSate != FLASH_FINISHED)
    	TURN_O_LED_ON();
  }

/* LogThread:
 * Thread of the flash log.
 */

THD_WORKING_AREA(LogThread_wa, 512);
THD_FUNCTION(LogThread, arg)
  {
    (void) arg;
    uint32_t FlashAddress;
    chRegSetThreadName("Flash Logger");
    int16_t i;

    OpCode = NO_OPCODE;
    FlashSate = FLASH_IDLE;
    FlashAddress = 0;
    chBSemObjectInit(&FlashSemaphore, TRUE); /* Semaphore initialization before use */

    while (TRUE)
      {
        chBSemWait(&FlashSemaphore);
        switch (OpCode)
          {
          case NO_OPCODE:
            break;
          case RECORDING_OPCODE:
            TURN_LED_ON();
            M25P16SetWriteEnable();
            M25P16BulkErase();
            FlashSate = FLASH_WAIT_FOR_PAGE;
            FlashAddress = 0;
            TURN_LED_OFF();
            break;
          case WRITEPAGE_OPCODE:
            M25P16WritePage(FlashAddress, DataBuffer);
            FlashAddress += PAGE_SIZE;
            if (FlashAddress < LAST_FLASH_ADDRESS)
              FlashSate = FLASH_WAIT_FOR_PAGE;
            else
              {
                FlashSate = FLASH_FINISHED;
                TURN_LED_OFF();
                chThdSleepMilliseconds(1000);
                TURN_LED_ON();
                while (TRUE)
                  chThdSleep(TIME_INFINITE);
              }
            break;
          case READING_OPCODE:
            TURN_LED_ON();
            FlashAddress = 0;
            FlashSate = FLASH_FINISHED;
            DataBuffer = (uint8_t *) LogData;
            while (FlashAddress < LAST_FLASH_ADDRESS)
              {
                M25P16ReadPage(FlashAddress, DataBuffer);
                for (i=0;i<PAGE_SIZE/16;i++)
                  {
                    if (SDU1.config->usbp->state == USB_ACTIVE)
                      {
                        chnWrite(&SDU1, &(DataBuffer[16*i]),16);
                      }
                   // chThdSleepMicroseconds(50);

                  }
                FlashAddress += PAGE_SIZE;
              }
            TURN_LED_OFF();
            while (TRUE)
              chThdSleep(TIME_INFINITE);
            break;
          }

      }
  }

#endif
