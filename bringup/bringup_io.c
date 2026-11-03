#include "bringup.h"
#include <stdarg.h>
#include <stdio.h>

bp_rx_errors_t bp_rx_errors;

/* Second input path for when the UART RX wire does not work: a debugger writes bytes into buf over
 * SWD, then advances head; the console consumes them exactly like received UART bytes.
 * tools/bringup_host.py finds this by symbol name and checks the magic. */
#define SWD_INPUT_MAGIC 0x49445753U /* "SWDI" */
#define SWD_INPUT_SIZE  256U

typedef struct {
    uint32_t magic;
    volatile uint32_t head; /* written by the host only */
    volatile uint32_t tail; /* written by the firmware only */
    volatile uint8_t buf[SWD_INPUT_SIZE];
} bp_swd_input_t;

__attribute__((used)) volatile bp_swd_input_t bp_swd_input = {.magic = SWD_INPUT_MAGIC};

static int swd_getc(void)
{
    uint32_t tail = bp_swd_input.tail;

    if (tail == bp_swd_input.head) {
        return -1;
    }
    uint8_t c = bp_swd_input.buf[tail % SWD_INPUT_SIZE];
    bp_swd_input.tail = tail + 1U;
    return c;
}

/* Register-level console so a continuous stream (uart u) has no inter-frame gaps and the receive
 * path can be polled fast enough for 1 Mbaud echo without interrupts. */

void bp_putc(char c)
{
    while (!(USART1->SR & USART_SR_TXE)) {
    }
    USART1->DR = (uint8_t)c;
}

void bp_flush(void)
{
    while (!(USART1->SR & USART_SR_TC)) {
    }
}

void bp_puts(const char *s)
{
    for (; *s; s++) {
        if (*s == '\n') {
            bp_putc('\r');
        }
        bp_putc(*s);
    }
}

void bp_printf(const char *fmt, ...)
{
    char buf[200];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    bp_puts(buf);
}

int bp_getc_nowait(void)
{
    uint32_t sr = USART1->SR;

    if (!(sr & USART_SR_RXNE)) {
        if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
            /* Reading SR then DR clears the error flags. */
            (void)USART1->DR;
            bp_rx_errors.ore += (sr & USART_SR_ORE) ? 1U : 0U;
            bp_rx_errors.fe += (sr & USART_SR_FE) ? 1U : 0U;
            bp_rx_errors.ne += (sr & USART_SR_NE) ? 1U : 0U;
        }
        return swd_getc();
    }

    uint8_t c = (uint8_t)USART1->DR;
    bp_rx_errors.ore += (sr & USART_SR_ORE) ? 1U : 0U;
    bp_rx_errors.fe += (sr & USART_SR_FE) ? 1U : 0U;
    bp_rx_errors.ne += (sr & USART_SR_NE) ? 1U : 0U;
    return c;
}

bool bp_key_pressed(void)
{
    return bp_getc_nowait() >= 0;
}

bool bp_poll_line(char *buf, size_t size)
{
    static size_t len;
    static bool after_cr;

    int c = bp_getc_nowait();
    if (c < 0) {
        return false;
    }

    bool skip = after_cr && c == '\n';
    after_cr = false;
    if (skip) {
        return false;
    }

    if (c == '\r' || c == '\n') {
        after_cr = (c == '\r');
        bp_puts("\n");
        buf[len] = '\0';
        len = 0;
        return true;
    }
    if (c == 0x08 || c == 0x7F) {
        if (len > 0) {
            len--;
            bp_puts("\b \b");
        }
        return false;
    }
    if (c >= 0x20 && c < 0x7F && len < size - 1) {
        buf[len++] = (char)c;
        bp_putc((char)c);
    }
    return false;
}

void bp_result(const char *id, bp_verdict_t verdict, const char *fmt, ...)
{
    static const char *const names[] = {"PASS", "FAIL", "INFO"};
    char detail[140];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(detail, sizeof detail, fmt, ap);
    va_end(ap);
    bp_printf("RESULT %s %s %s\n", id, names[verdict], detail);
}
