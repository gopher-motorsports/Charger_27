#ifndef BRINGUP_H_
#define BRINGUP_H_

/*
 * Bring-up test firmware for the Charger_27 board.
 *
 * Built only with -DBRINGUP_TEST=ON (see CMakeLists.txt). bringup_run() is entered from main()
 * after the normal peripheral init and never returns; the RTOS scheduler is not started, so the
 * production tasks do not run and nothing here competes with them.
 *
 * Console: USART1, 1 Mbaud 8N1 (SWD1 pin 3 TX / pin 5 RX, or the ST-Link bridge UART).
 * Every test finishes with a machine-readable line:  RESULT <id> <PASS|FAIL|INFO> <detail>
 * Pin names and numbers follow the schematic (Charger_27_Board_Interfaces), not main.h.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim4;

typedef enum {
    BP_PASS,
    BP_FAIL,
    BP_INFO
} bp_verdict_t;

typedef struct {
    uint32_t fe;
    uint32_t ne;
    uint32_t ore;
} bp_rx_errors_t;

/* Receive errors counted by bp_getc_nowait() since the last clear. */
extern bp_rx_errors_t bp_rx_errors;

/* ---- console (bringup_io.c) ---- */
void bp_putc(char c);
void bp_flush(void);
void bp_puts(const char *s); /* '\n' is sent as "\r\n" */
void bp_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int bp_getc_nowait(void);    /* next received byte, or -1 */
bool bp_key_pressed(void);   /* consumes one received byte */
bool bp_poll_line(char *buf, size_t size);
void bp_result(const char *id, bp_verdict_t verdict, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/* ---- helpers (bringup.c) ---- */
void bp_irq_unmask(void); /* call after any FreeRTOS-using library call; see bringup.c */
bool bp_streq(const char *a, const char *b); /* case-insensitive */
bool bp_parse_u32(const char *s, uint32_t *out); /* decimal or 0x hex */
uint32_t bp_cycles(void);
void bp_delay_us(uint32_t us);
uint32_t bp_elapsed_us(uint32_t start_cycles);

/* ---- commands: each prints its usage when called without arguments ---- */
int bp_cmd_info(int argc, char **argv);
int bp_cmd_gpio(int argc, char **argv);
int bp_cmd_sdc(int argc, char **argv);
int bp_cmd_i2c(int argc, char **argv);
int bp_cmd_uart(int argc, char **argv);
int bp_cmd_can(int argc, char **argv);
int bp_cmd_cp(int argc, char **argv);
int bp_cmd_pp(int argc, char **argv);

/* Shared with the SDC command: report level changes on the named pins until a key is pressed. */
void bp_watch_pins(const char *const *names, int count);

/* The prompt loop blinks LD1 (PC0). Cleared while 'gpio out HEARTBEAT_LED' holds the pin so a meter
 * can read it; set again by 'gpio walk' or 'gpio heartbeat on'. */
extern bool bp_heartbeat_enabled;

/* Production-style CAN sending, usable from any test loop (bringup_can.c). 'pattern' sends fixed test
 * values on the real IDs (bench only); otherwise limit_w and the live SDC levels are sent. */
typedef struct {
    bool pattern;
    float limit_w;
    uint32_t next_power_ms;
    uint32_t next_sdc_ms;
    uint32_t sent_power;
    uint32_t sent_sdc;
    uint32_t failures;
} bp_can_tx_t;

void bp_can_tx_start(bp_can_tx_t *t, bool pattern, float limit_w);
void bp_can_tx_tick(bp_can_tx_t *t);   /* call every loop: sends due groups, services TX and RX */
void bp_can_tx_report(const bp_can_tx_t *t, const char *id);

void bringup_run(void);

#endif /* BRINGUP_H_ */
