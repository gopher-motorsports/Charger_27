#include "bringup.h"
#include "eeprom.h"
#include <stdio.h>
#include <string.h>

/* IC2 24LC02B EEPROM on I2C3: SCL = PA8 (U1 pin 41), SDA = PC9 (U1 pin 40), 3.3 kOhm pull-ups to 5V.
 * The part ignores A0-A2, so it answers at every address from 0x50 to 0x57. */
#define EEPROM_ADDR         (0x50U << 1)
#define EXPECT_FIRST        0x50U
#define EXPECT_LAST         0x57U
#define SCAN_FIRST          0x08U
#define SCAN_LAST           0x77U

/* The 24LC02B has an 8-byte page buffer: a write that crosses a page boundary wraps back to the
 * start of the same page, so a longer write must be split into page-sized transactions. */
#define EEPROM_PAGE         8U
#define EEPROM_TWC_MAX_US   5000U
#define PATTERN_LEN         16U
#define I2C_TIMEOUT_MS      20U

static I2C_HandleTypeDef hi2c3;

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef g = {0};

    if (hi2c->Instance != I2C3) {
        return;
    }
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();

    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C3;
    g.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &g);
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C3) {
        return;
    }
    __HAL_RCC_I2C3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
}

/* A reset in the middle of a transfer can leave the EEPROM holding SDA low. Clock SCL until it
 * releases, then issue a STOP, so the first transaction after a reboot does not hang. */
static void bus_recover(void)
{
    GPIO_InitTypeDef g = {0};

    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    bp_delay_us(10);

    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        bp_delay_us(5);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        bp_delay_us(5);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    bp_delay_us(5);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    bp_delay_us(5);
}

static bool i2c_start(uint32_t hz)
{
    if (hi2c3.State != HAL_I2C_STATE_RESET) {
        HAL_I2C_DeInit(&hi2c3);
    }
    bus_recover();

    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = hz;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    return HAL_I2C_Init(&hi2c3) == HAL_OK;
}

static bool parse_speed(const char *s, uint32_t *hz)
{
    if (strcmp(s, "100") == 0) {
        *hz = 100000U;
        return true;
    }
    if (strcmp(s, "400") == 0) {
        *hz = 400000U;
        return true;
    }
    return false;
}

static bool eeprom_acks(void)
{
    return HAL_I2C_IsDeviceReady(&hi2c3, EEPROM_ADDR, 1, 3) == HAL_OK;
}

static bool eeprom_read(uint8_t word_addr, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c3, EEPROM_ADDR, word_addr, I2C_MEMADD_SIZE_8BIT, buf, len,
                            I2C_TIMEOUT_MS) == HAL_OK;
}

/* Writes in page-sized pieces and ACK-polls until each internal write cycle finishes. */
static bool eeprom_write(uint8_t word_addr, const uint8_t *data, uint32_t len, uint32_t *twc_max_us)
{
    for (uint32_t off = 0; off < len;) {
        uint32_t room = EEPROM_PAGE - ((word_addr + off) % EEPROM_PAGE);
        uint32_t n = (len - off < room) ? (len - off) : room;

        if (HAL_I2C_Mem_Write(&hi2c3, EEPROM_ADDR, (uint16_t)(word_addr + off), I2C_MEMADD_SIZE_8BIT,
                              (uint8_t *)&data[off], (uint16_t)n, I2C_TIMEOUT_MS) != HAL_OK) {
            return false;
        }
        uint32_t start = bp_cycles();
        while (HAL_I2C_IsDeviceReady(&hi2c3, EEPROM_ADDR, 1, 2) != HAL_OK) {
            if (bp_elapsed_us(start) > 2U * EEPROM_TWC_MAX_US) {
                return false;
            }
        }
        uint32_t twc = bp_elapsed_us(start);
        if (twc > *twc_max_us) {
            *twc_max_us = twc;
        }
        off += n;
    }
    return true;
}

static bool eeprom_verify(const char *label, const uint8_t *expect, uint32_t *bad_bytes)
{
    uint8_t got[PATTERN_LEN];

    if (!eeprom_read(0x00, got, sizeof got)) {
        bp_printf("  %-10s read FAILED (HAL error 0x%lX)\n", label, (unsigned long)hi2c3.ErrorCode);
        return false;
    }
    uint32_t bad = 0;
    for (uint32_t i = 0; i < sizeof got; i++) {
        if (got[i] != expect[i]) {
            bad++;
            bp_printf("  %-10s [0x%02lX] wrote 0x%02X read 0x%02X\n", label, (unsigned long)i, expect[i], got[i]);
        }
    }
    *bad_bytes += bad;
    bp_printf("  %-10s %s\n", label, bad == 0U ? "matches" : "MISMATCH");
    return bad == 0U;
}

static void eeprom_test(uint32_t hz)
{
    char id[24];
    uint8_t original[PATTERN_LEN];
    uint8_t pattern[PATTERN_LEN];
    uint8_t inverted[PATTERN_LEN];
    uint32_t bad = 0;
    uint32_t twc_max = 0;
    bool ok;

    snprintf(id, sizeof id, "i2c.eeprom.%luk", (unsigned long)(hz / 1000U));
    bp_printf("EEPROM write/read-back at %lu kHz\n", (unsigned long)(hz / 1000U));

    if (!i2c_start(hz) || !eeprom_acks() || !eeprom_read(0x00, original, sizeof original)) {
        bp_result(id, BP_FAIL, "no ACK from 0x%02X or initial read failed", EEPROM_ADDR >> 1);
        return;
    }
    for (uint32_t i = 0; i < PATTERN_LEN; i++) {
        pattern[i] = (uint8_t)i;
        inverted[i] = (uint8_t)~i;
    }

    ok = eeprom_write(0x00, pattern, sizeof pattern, &twc_max);
    ok = ok && eeprom_verify("pattern", pattern, &bad);
    ok = ok && eeprom_write(0x00, inverted, sizeof inverted, &twc_max);
    ok = ok && eeprom_verify("inverted", inverted, &bad);

    uint32_t restore_bad = 0;
    bool restored = eeprom_write(0x00, original, sizeof original, &twc_max)
                    && eeprom_verify("restore", original, &restore_bad);
    if (!restored) {
        bp_puts("  WARNING: original contents of 0x00-0x0F could not be restored\n");
    }

    bp_result(id, (ok && bad == 0U) ? BP_PASS : BP_FAIL, "mismatched_bytes=%lu write_cycle_max=%luus restored=%s",
              (unsigned long)bad, (unsigned long)twc_max, restored ? "yes" : "NO");
}

static const char *eeprom_status_name(eepromStatus_E s)
{
    switch (s) {
    case EEPROM_OK:         return "OK";
    case EEPROM_ERR_PARAM:  return "ERR_PARAM";
    case EEPROM_ERR_NACK:   return "ERR_NACK";
    case EEPROM_ERR_BUS:    return "ERR_BUS";
    case EEPROM_ERR_BUSY:   return "ERR_BUSY";
    case EEPROM_ERR_VERIFY: return "ERR_VERIFY";
    }
    return "?";
}

/* Exercises Core/Src/eeprom.c (the application driver) rather than the helpers above. The write
 * starts at 0x05 so it is split into 3 + 8 + 5 bytes across three pages. */
#define DRV_ADDR 0x05U

static void driver_test(uint32_t hz)
{
    eeprom_S ee;
    uint8_t original[PATTERN_LEN];
    uint8_t pattern[PATTERN_LEN];
    uint8_t probe[PATTERN_LEN];
    eepromStatus_E s;
    bool ok = true;

    bp_printf("eeprom.c driver test at %lu kHz, bytes 0x%02X-0x%02X\n", (unsigned long)(hz / 1000U), DRV_ADDR,
              DRV_ADDR + PATTERN_LEN - 1U);
    if (!i2c_start(hz)) {
        bp_result("i2c.driver", BP_FAIL, "I2C3 init failed");
        return;
    }
    s = eepromInit(&ee, &hi2c3);
    bp_printf("  eepromInit           %s\n", eeprom_status_name(s));
    if (s != EEPROM_OK) {
        bp_result("i2c.driver", BP_FAIL, "eepromInit returned %s", eeprom_status_name(s));
        return;
    }
    s = eepromRead(&ee, DRV_ADDR, original, sizeof original);
    bp_printf("  eepromRead original  %s\n", eeprom_status_name(s));
    if (s != EEPROM_OK) {
        bp_result("i2c.driver", BP_FAIL, "initial eepromRead returned %s", eeprom_status_name(s));
        return;
    }

    for (uint32_t i = 0; i < PATTERN_LEN; i++) {
        pattern[i] = (uint8_t)(0xA0U + i);
    }
    uint32_t t0 = bp_cycles();
    s = eepromWrite(&ee, DRV_ADDR, pattern, sizeof pattern);
    uint32_t write_us = bp_elapsed_us(t0);
    bp_printf("  eepromWrite 16 bytes %s (%lu us for 3 pages)\n", eeprom_status_name(s), (unsigned long)write_us);
    ok = ok && s == EEPROM_OK;

    s = eepromVerify(&ee, DRV_ADDR, pattern, sizeof pattern);
    bp_printf("  eepromVerify         %s\n", eeprom_status_name(s));
    ok = ok && s == EEPROM_OK;

    /* Out-of-range requests must be refused before any bus traffic. */
    eepromStatus_E past_end = eepromRead(&ee, 0xF8U, probe, sizeof probe);
    eepromStatus_E zero_len = eepromWrite(&ee, 0x00U, pattern, 0U);
    bp_printf("  past-end read        %s (expect ERR_PARAM)\n", eeprom_status_name(past_end));
    bp_printf("  zero-length write    %s (expect ERR_PARAM)\n", eeprom_status_name(zero_len));
    ok = ok && past_end == EEPROM_ERR_PARAM && zero_len == EEPROM_ERR_PARAM;

    bool restored = eepromWrite(&ee, DRV_ADDR, original, sizeof original) == EEPROM_OK
                    && eepromVerify(&ee, DRV_ADDR, original, sizeof original) == EEPROM_OK;
    if (!restored) {
        bp_printf("  WARNING: original contents of 0x%02X-0x%02X could not be restored\n", DRV_ADDR,
                  DRV_ADDR + PATTERN_LEN - 1U);
    }
    bp_result("i2c.driver", (ok && restored) ? BP_PASS : BP_FAIL, "write_3_pages=%luus restored=%s",
              (unsigned long)write_us, restored ? "yes" : "NO");
}

static void scan(uint32_t hz)
{
    uint32_t expected_hits = 0;
    uint32_t unexpected = 0;
    char id[] = "i2c.scan";

    if (!i2c_start(hz)) {
        bp_result(id, BP_FAIL, "I2C3 init failed");
        return;
    }
    bp_printf("scanning 0x%02X-0x%02X at %lu kHz\n", SCAN_FIRST, SCAN_LAST, (unsigned long)(hz / 1000U));
    for (uint32_t a = SCAN_FIRST; a <= SCAN_LAST; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c3, a << 1, 1, 3) == HAL_OK) {
            bp_printf("  ACK 0x%02lX\n", (unsigned long)a);
            if (a >= EXPECT_FIRST && a <= EXPECT_LAST) {
                expected_hits++;
            } else {
                unexpected++;
            }
        }
    }
    bool pass = expected_hits == (EXPECT_LAST - EXPECT_FIRST + 1U) && unexpected == 0U;
    bp_result(id, pass ? BP_PASS : BP_FAIL, "acks_in_0x50-0x57=%lu/8 unexpected=%lu", (unsigned long)expected_hits,
              (unsigned long)unexpected);
}

static void who_am_i(void)
{
    uint8_t first;

    if (!i2c_start(100000U) || !eeprom_acks()) {
        bp_result("i2c.id", BP_FAIL, "no ACK from EEPROM at 0x%02X", EEPROM_ADDR >> 1);
        return;
    }
    if (!eeprom_read(0x00, &first, 1)) {
        bp_result("i2c.id", BP_FAIL, "ACK ok but random read failed (HAL error 0x%lX)", (unsigned long)hi2c3.ErrorCode);
        return;
    }
    bp_result("i2c.id", BP_PASS, "24LC02B at 0x%02X ACKs, byte[0x00]=0x%02X (no ID register on this part)",
              EEPROM_ADDR >> 1, first);
}

static void read_loop(uint32_t hz)
{
    uint8_t buf[PATTERN_LEN];
    uint32_t reads = 0;
    uint32_t errors = 0;
    uint32_t last_print = HAL_GetTick();

    if (!i2c_start(hz)) {
        bp_result("i2c.loop", BP_FAIL, "I2C3 init failed");
        return;
    }
    bp_printf("reading EEPROM at %lu kHz, press any key to stop\n", (unsigned long)(hz / 1000U));
    while (!bp_key_pressed()) {
        reads++;
        if (!eeprom_read(0x00, buf, sizeof buf)) {
            errors++;
            i2c_start(hz);
        }
        if (HAL_GetTick() - last_print >= 1000U) {
            last_print = HAL_GetTick();
            bp_printf("reads=%lu errors=%lu\n", (unsigned long)reads, (unsigned long)errors);
        }
    }
    bp_result("i2c.loop", errors == 0U ? BP_PASS : BP_FAIL, "reads=%lu errors=%lu speed=%lukHz", (unsigned long)reads,
              (unsigned long)errors, (unsigned long)(hz / 1000U));
}

static void usage(void)
{
    bp_puts("Plan row            Command\n"
            "Eye diagram         i2c blast [100|400]   continuous EEPROM reads until a key (alias: eye, loop), default 400\n"
            "Who-am-I            i2c whoami            EEPROM ACK + random read (alias: id; the 24LC02B has no ID register)\n"
            "Peripheral ping     i2c ping [100|400]    ACK every address 0x08-0x77, expect exactly 0x50-0x57 (alias: scan)\n"
            "Data transfer       i2c eeprom [100|400]  write 0x00-0x0F, then the inverse, read back, restore (default both)\n"
            "Driver check        i2c driver [100|400]  run Core/Src/eeprom.c: 3-page write at 0x05, verify, range checks, restore\n");
}

int bp_cmd_i2c(int argc, char **argv)
{
    uint32_t hz = 0;

    if (argc < 2) {
        usage();
        return 0;
    }
    if (argc > 3 || (argc == 3 && !parse_speed(argv[2], &hz))) {
        usage();
        return -1;
    }

    if (bp_streq(argv[1], "scan") || bp_streq(argv[1], "ping")) {
        scan(argc == 3 ? hz : 100000U);
    } else if (bp_streq(argv[1], "id") || bp_streq(argv[1], "whoami")) {
        who_am_i();
    } else if (bp_streq(argv[1], "eeprom")) {
        if (argc == 3) {
            eeprom_test(hz);
        } else {
            eeprom_test(100000U);
            eeprom_test(400000U);
        }
    } else if (bp_streq(argv[1], "driver")) {
        driver_test(argc == 3 ? hz : 400000U);
    } else if (bp_streq(argv[1], "loop") || bp_streq(argv[1], "eye") || bp_streq(argv[1], "blast")) {
        read_loop(argc == 3 ? hz : 400000U);
    } else {
        usage();
        return bp_streq(argv[1], "help") ? 0 : -1;
    }
    return 0;
}
