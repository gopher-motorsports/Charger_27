#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

/* ==================================================================== */
/* ============================= INCLUDES ============================= */
/* ==================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

// The driver needs the HAL I2C module. It is off in the CubeMX configuration today (the bring-up
// build turns it on), so without it this file and eeprom.c compile to nothing.
#ifdef HAL_I2C_MODULE_ENABLED

/* ==================================================================== */
/* ============================= DEFINES ============================== */
/* ==================================================================== */

// Microchip 24LC02B (IC2), datasheet DS21709J: 2 Kbit organized as one block of 256 x 8 bits
#define EEPROM_SIZE_BYTES         (256U)

// Page write buffer. A page write that runs past the end of a page wraps to the start of the same
// page and overwrites it, so eepromWrite() splits writes at page boundaries (DS21709J section 6.2)
#define EEPROM_PAGE_SIZE_BYTES    (8U)

// Control code 1010. The three block select bits are "don't care" on this part (A0-A2 are not
// connected inside), so the device answers at every address from 0x50 to 0x57 (section 5.0)
#define EEPROM_I2C_ADDR_7BIT      (0x50U)

// Write cycle time T_WC for a byte or page write, 5 ms max (AC characteristics, param 15)
#define EEPROM_WRITE_CYCLE_MAX_MS (5U)

// Default timeout for one I2C transaction
#define EEPROM_I2C_TIMEOUT_MS     (20U)

/* ==================================================================== */
/* ============================== STRUCTS ============================= */
/* ==================================================================== */

typedef enum
{
    EEPROM_OK = 0,
    EEPROM_ERR_PARAM,  // NULL pointer, zero length, or range past the end of the array
    EEPROM_ERR_NACK,   // device did not acknowledge (absent, or still in a write cycle)
    EEPROM_ERR_BUS,    // bus busy, bus error, arbitration lost or transaction timeout
    EEPROM_ERR_BUSY,   // write cycle did not finish within twice T_WC
    EEPROM_ERR_VERIFY  // read-back differs from what was written (WP pin high, or a worn cell)
} eepromStatus_E;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t devAddr;   // HAL form: 7-bit address shifted left by one
    uint32_t timeoutMs; // per I2C transaction
} eeprom_S;

/* ==================================================================== */
/* =================== GLOBAL FUNCTION DECLARATIONS =================== */
/* ==================================================================== */

// None of these functions are thread-safe. If more than one task uses the bus, the caller must
// hold a lock around each call. eepromWrite() busy-waits up to ~10 ms per page.

/*!
  @brief    Set up the driver for the EEPROM on an initialized I2C bus and check that it answers
  @param    eeprom Driver state to fill in
  @param    hi2c   I2C handle, already initialized (100 or 400 kHz; the part supports both at 2.5-5.5 V)
  @return   EEPROM_OK if the device acknowledges its address
*/
eepromStatus_E eepromInit(eeprom_S *eeprom, I2C_HandleTypeDef *hi2c);

/*!
  @brief    Check once whether the device acknowledges. It does not acknowledge during a write cycle.
*/
eepromStatus_E eepromIsReady(eeprom_S *eeprom);

/*!
  @brief    ACK-poll until the device finishes its internal write cycle (DS21709J section 7.0)
  @param    timeoutMs Give up after this many milliseconds
  @return   EEPROM_OK when the device acknowledges, EEPROM_ERR_BUSY on timeout
*/
eepromStatus_E eepromWaitReady(eeprom_S *eeprom, uint32_t timeoutMs);

/*!
  @brief    Read bytes starting at a word address (random read followed by a sequential read)
  @param    addr Word address, 0x00-0xFF
  @param    buf  Destination
  @param    len  Number of bytes; addr + len must not pass the end of the array
*/
eepromStatus_E eepromRead(eeprom_S *eeprom, uint8_t addr, uint8_t *buf, uint16_t len);

/*!
  @brief    Write bytes starting at a word address. Splits the data into page writes and waits for
            each write cycle to finish, so the data is stored when this returns EEPROM_OK.
  @param    addr Word address, 0x00-0xFF
  @param    data Source
  @param    len  Number of bytes; addr + len must not pass the end of the array
  @note     A write with the WP pin high is acknowledged but not stored. Use eepromVerify() when
            that matters.
*/
eepromStatus_E eepromWrite(eeprom_S *eeprom, uint8_t addr, const uint8_t *data, uint16_t len);

/*!
  @brief    Read back a range and compare it with the expected data
  @return   EEPROM_OK if every byte matches, EEPROM_ERR_VERIFY on the first difference
*/
eepromStatus_E eepromVerify(eeprom_S *eeprom, uint8_t addr, const uint8_t *data, uint16_t len);

#endif /* HAL_I2C_MODULE_ENABLED */

#endif /* INC_EEPROM_H_ */
