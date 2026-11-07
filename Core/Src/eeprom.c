/* ==================================================================== */
/* ============================= INCLUDES ============================= */
/* ==================================================================== */
#include "eeprom.h"

#ifdef HAL_I2C_MODULE_ENABLED

#include <stddef.h>

/* ==================================================================== */
/* ============================= DEFINES ============================== */
/* ==================================================================== */

// ACK polling gives up after twice the datasheet maximum write cycle time
#define WRITE_CYCLE_TIMEOUT_MS  (2U * EEPROM_WRITE_CYCLE_MAX_MS)

// Bytes read per transaction by eepromVerify()
#define VERIFY_CHUNK_BYTES      (16U)

/* ==================================================================== */
/* ======================= PRIVATE FUNCTIONS ========================== */
/* ==================================================================== */

static bool rangeValid(uint8_t addr, uint16_t len)
{
    return len > 0U && (uint32_t)addr + len <= EEPROM_SIZE_BYTES;
}

// HAL_I2C_IsDeviceReady() returns HAL_ERROR with no error code on a NACK, and the memory
// transfer functions set HAL_I2C_ERROR_AF. Everything else is a bus problem.
static eepromStatus_E halToStatus(const eeprom_S *eeprom, HAL_StatusTypeDef hal)
{
    if (hal == HAL_OK)
    {
        return EEPROM_OK;
    }
    uint32_t err = eeprom->hi2c->ErrorCode;
    if (hal == HAL_ERROR && (err == HAL_I2C_ERROR_NONE || (err & HAL_I2C_ERROR_AF) != 0U))
    {
        return EEPROM_ERR_NACK;
    }
    return EEPROM_ERR_BUS;
}

/* ==================================================================== */
/* =================== GLOBAL FUNCTION DEFINITIONS ==================== */
/* ==================================================================== */

eepromStatus_E eepromInit(eeprom_S *eeprom, I2C_HandleTypeDef *hi2c)
{
    if (eeprom == NULL || hi2c == NULL)
    {
        return EEPROM_ERR_PARAM;
    }
    eeprom->hi2c = hi2c;
    eeprom->devAddr = (uint16_t)(EEPROM_I2C_ADDR_7BIT << 1);
    eeprom->timeoutMs = EEPROM_I2C_TIMEOUT_MS;

    // A write cycle may still be running if the MCU reset right after a write. If the device never
    // answers it is missing rather than busy.
    eepromStatus_E status = eepromWaitReady(eeprom, WRITE_CYCLE_TIMEOUT_MS);
    return (status == EEPROM_ERR_BUSY) ? EEPROM_ERR_NACK : status;
}

eepromStatus_E eepromIsReady(eeprom_S *eeprom)
{
    return halToStatus(eeprom, HAL_I2C_IsDeviceReady(eeprom->hi2c, eeprom->devAddr, 1U, eeprom->timeoutMs));
}

eepromStatus_E eepromWaitReady(eeprom_S *eeprom, uint32_t timeoutMs)
{
    uint32_t start = HAL_GetTick();

    for (;;)
    {
        eepromStatus_E status = eepromIsReady(eeprom);
        if (status != EEPROM_ERR_NACK)
        {
            return status;
        }
        if (HAL_GetTick() - start > timeoutMs)
        {
            return EEPROM_ERR_BUSY;
        }
    }
}

eepromStatus_E eepromRead(eeprom_S *eeprom, uint8_t addr, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || !rangeValid(addr, len))
    {
        return EEPROM_ERR_PARAM;
    }
    return halToStatus(eeprom, HAL_I2C_Mem_Read(eeprom->hi2c, eeprom->devAddr, addr, I2C_MEMADD_SIZE_8BIT,
                                                buf, len, eeprom->timeoutMs));
}

eepromStatus_E eepromWrite(eeprom_S *eeprom, uint8_t addr, const uint8_t *data, uint16_t len)
{
    if (data == NULL || !rangeValid(addr, len))
    {
        return EEPROM_ERR_PARAM;
    }

    uint16_t off = 0U;
    while (off < len)
    {
        uint16_t wordAddr = (uint16_t)(addr + off);
        uint16_t room = (uint16_t)(EEPROM_PAGE_SIZE_BYTES - (wordAddr % EEPROM_PAGE_SIZE_BYTES));
        uint16_t n = (uint16_t)((len - off < room) ? (len - off) : room);

        eepromStatus_E status = halToStatus(eeprom, HAL_I2C_Mem_Write(eeprom->hi2c, eeprom->devAddr, wordAddr,
                                                                      I2C_MEMADD_SIZE_8BIT, (uint8_t *)&data[off],
                                                                      n, eeprom->timeoutMs));
        if (status != EEPROM_OK)
        {
            return status;
        }
        // The page is only stored once the internal write cycle ends
        status = eepromWaitReady(eeprom, WRITE_CYCLE_TIMEOUT_MS);
        if (status != EEPROM_OK)
        {
            return status;
        }
        off += n;
    }
    return EEPROM_OK;
}

eepromStatus_E eepromVerify(eeprom_S *eeprom, uint8_t addr, const uint8_t *data, uint16_t len)
{
    uint8_t chunk[VERIFY_CHUNK_BYTES];

    if (data == NULL || !rangeValid(addr, len))
    {
        return EEPROM_ERR_PARAM;
    }

    for (uint16_t off = 0U; off < len; off += VERIFY_CHUNK_BYTES)
    {
        uint16_t remaining = (uint16_t)(len - off);
        uint16_t n = (remaining < VERIFY_CHUNK_BYTES) ? remaining : (uint16_t)VERIFY_CHUNK_BYTES;
        eepromStatus_E status = eepromRead(eeprom, (uint8_t)(addr + off), chunk, n);
        if (status != EEPROM_OK)
        {
            return status;
        }
        for (uint16_t i = 0U; i < n; i++)
        {
            if (chunk[i] != data[off + i])
            {
                return EEPROM_ERR_VERIFY;
            }
        }
    }
    return EEPROM_OK;
}

#endif /* HAL_I2C_MODULE_ENABLED */
