#include "bringup.h"

#define U_CHAR              0x55U
#define ECHO_IDLE_MS        1000U
#define ECHO_DEFAULT_S      30U
#define ECHO_FIFO_SIZE      256U

static uint32_t baud_rate(void)
{
    /* 16x oversampling: BRR holds USARTDIV * 16, so baud = PCLK2 / BRR. */
    return HAL_RCC_GetPCLK2Freq() / USART1->BRR;
}

/* 0x55 in 8N1 is start 0, then 1,0,1,0,1,0,1,0, then stop 1: with no gap between frames the line
 * alternates at exactly the baud rate. Keeping TXE topped up in a tight loop is what removes the gap. */
static void stream_u(uint32_t seconds)
{
    uint32_t start = HAL_GetTick();
    uint32_t sent = 0;

    bp_printf("streaming 0x55 at %lu baud, press any key to stop\n", (unsigned long)baud_rate());
    bp_flush();

    for (;;) {
        uint32_t sr = USART1->SR;
        if (sr & USART_SR_RXNE) {
            (void)USART1->DR;
            break;
        }
        if (sr & USART_SR_TXE) {
            USART1->DR = U_CHAR;
            sent++;
        }
        if (seconds != 0U && HAL_GetTick() - start >= seconds * 1000U) {
            break;
        }
    }
    bp_flush();
    bp_printf("\nstreamed %lu x 0x55\n", (unsigned long)sent);
    bp_result("uart.u", BP_INFO, "sent=%lu baud=%lu", (unsigned long)sent, (unsigned long)baud_rate());
}

/* Echoes every received byte. The FIFO decouples receive from transmit so a burst from the host
 * never waits on the transmitter, which would risk an overrun at 1 Mbaud. */
static void echo(uint32_t max_seconds)
{
    static uint8_t fifo[ECHO_FIFO_SIZE];
    uint32_t head = 0;
    uint32_t tail = 0;
    uint32_t received = 0;
    uint32_t dropped = 0;
    uint32_t start = HAL_GetTick();
    uint32_t last_rx = start;

    bp_rx_errors = (bp_rx_errors_t){0};
    bp_printf("ECHO READY %lu baud 8N1\n", (unsigned long)baud_rate());
    bp_flush();

    for (;;) {
        int c = bp_getc_nowait();
        if (c >= 0) {
            received++;
            last_rx = HAL_GetTick();
            if (head - tail < ECHO_FIFO_SIZE) {
                fifo[head++ % ECHO_FIFO_SIZE] = (uint8_t)c;
            } else {
                dropped++;
            }
        }
        if (tail != head && (USART1->SR & USART_SR_TXE)) {
            USART1->DR = fifo[tail++ % ECHO_FIFO_SIZE];
        }

        uint32_t now = HAL_GetTick();
        bool drained = tail == head;
        if (drained && ((received > 0U && now - last_rx >= ECHO_IDLE_MS) || now - start >= max_seconds * 1000U)) {
            break;
        }
    }
    bp_flush();

    bool pass = received > 0U && dropped == 0U && bp_rx_errors.fe == 0U && bp_rx_errors.ne == 0U
                && bp_rx_errors.ore == 0U;
    bp_printf("\nrx=%lu framing=%lu noise=%lu overrun=%lu dropped=%lu\n", (unsigned long)received,
              (unsigned long)bp_rx_errors.fe, (unsigned long)bp_rx_errors.ne, (unsigned long)bp_rx_errors.ore,
              (unsigned long)dropped);
    bp_result("uart.echo", pass ? BP_PASS : BP_FAIL, "rx=%lu fe=%lu ne=%lu ore=%lu dropped=%lu",
              (unsigned long)received, (unsigned long)bp_rx_errors.fe, (unsigned long)bp_rx_errors.ne,
              (unsigned long)bp_rx_errors.ore, (unsigned long)dropped);
}

static void usage(void)
{
    bp_puts("Plan row            Command\n"
            "Eye diagram         uart blast [seconds]  gap-free 0x55 stream until a key (alias: eye, u)\n"
            "Data transmission   uart blast            same stream; watch the terminal for unbroken U's\n"
            "Data reception      uart echo [seconds]   echo received bytes and report framing/noise/overrun errors;\n"
            "                                          ends after 1 s idle or the limit (default 30 s).\n"
            "                                          PC side: tools/bringup_host.py echo\n");
}

int bp_cmd_uart(int argc, char **argv)
{
    uint32_t seconds = 0;

    if (argc < 2) {
        usage();
        return 0;
    }
    if (argc == 3 && (!bp_parse_u32(argv[2], &seconds) || seconds == 0U)) {
        usage();
        return -1;
    }

    if ((bp_streq(argv[1], "u") || bp_streq(argv[1], "blast") || bp_streq(argv[1], "eye")) && argc <= 3) {
        stream_u(argc == 3 ? seconds : 0U);
    } else if (bp_streq(argv[1], "echo") && argc <= 3) {
        echo(argc == 3 ? seconds : ECHO_DEFAULT_S);
    } else {
        usage();
        return bp_streq(argv[1], "help") ? 0 : -1;
    }
    return 0;
}
