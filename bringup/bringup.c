#include "bringup.h"
#include <stdlib.h>
#include <string.h>

#define MAX_ARGS            8
#define LINE_SIZE           80
#define PROMPT              "bringup> "
#define STM32F446_DEV_ID    0x421U

/* Same cadence as the production heartbeat: 300 ms on, 2 s period. */
#define HEARTBEAT_ON_MS     300U
#define HEARTBEAT_OFF_MS    1700U

typedef struct {
    const char *name;
    int (*fn)(int argc, char **argv);
    const char *summary;
} bp_cmd_t;

static const bp_cmd_t commands[] = {
    {"info", bp_cmd_info, "MCU ID, clocks, reset cause (SWD test)"},
    {"gpio", bp_cmd_gpio, "drive and read GPIO / LEDs by schematic name"},
    {"sdc",  bp_cmd_sdc,  "shutdown-circuit sense inputs"},
    {"i2c",  bp_cmd_i2c,  "I2C3: scan, ID read, EEPROM read/write, read loop"},
    {"uart", bp_cmd_uart, "UART: continuous 'U' stream and echo"},
    {"can",  bp_cmd_can,  "CAN2: parameter frames and bus status"},
    {"cp",   bp_cmd_cp,   "J1772 control pilot (CP)"},
    {"pp",   bp_cmd_pp,   "J1772 proximity pilot (PP)"},
};

static uint32_t reset_flags;
bool bp_heartbeat_enabled = true;

/* gsense_init() creates a FreeRTOS task and GopherCAN takes a FreeRTOS mutex on every send. Before
 * the scheduler starts, FreeRTOS leaves BASEPRI at configMAX_SYSCALL_INTERRUPT_PRIORITY after each of
 * those calls, which blocks the HAL tick (TIM6), CAN and TIM4 interrupts. The scheduler never starts
 * in this build, so the mask has to be dropped again after any such call. */
void bp_irq_unmask(void)
{
    __set_BASEPRI(0);
}

bool bp_streq(const char *a, const char *b)
{
    for (; *a && *b; a++, b++) {
        char ca = (*a >= 'A' && *a <= 'Z') ? (char)(*a + 32) : *a;
        char cb = (*b >= 'A' && *b <= 'Z') ? (char)(*b + 32) : *b;
        if (ca != cb) {
            return false;
        }
    }
    return *a == *b;
}

bool bp_parse_u32(const char *s, uint32_t *out)
{
    char *end;
    unsigned long v = strtoul(s, &end, 0);

    if (end == s || *end != '\0') {
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

uint32_t bp_cycles(void)
{
    return DWT->CYCCNT;
}

uint32_t bp_elapsed_us(uint32_t start_cycles)
{
    return (DWT->CYCCNT - start_cycles) / (SystemCoreClock / 1000000U);
}

void bp_delay_us(uint32_t us)
{
    uint32_t start = bp_cycles();

    while (bp_elapsed_us(start) < us) {
    }
}

static void hw_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    reset_flags = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
}

/* LD1 (PC0) is the board heartbeat LED. Re-asserted after every command because the gpio
 * command can leave it configured as an input. */
static void heartbeat_pin_init(void)
{
    GPIO_InitTypeDef g = {0};

    g.Pin = GPIO_PIN_0;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
}

static void heartbeat(void)
{
    static uint32_t last;
    static bool on;
    uint32_t now = HAL_GetTick();

    if (!bp_heartbeat_enabled) {
        return;
    }
    if (now - last >= (on ? HEARTBEAT_ON_MS : HEARTBEAT_OFF_MS)) {
        last = now;
        on = !on;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

int bp_cmd_info(int argc, char **argv)
{
    if (argc > 1 && bp_streq(argv[1], "help")) {
        bp_puts("info    print MCU ID, flash size, UID, clocks, HSE mode and reset cause\n");
        return 0;
    }

    uint32_t idcode = DBGMCU->IDCODE;
    uint32_t dev_id = idcode & 0xFFFU;
    uint32_t rev_id = idcode >> 16;
    const volatile uint32_t *uid = (const volatile uint32_t *)UID_BASE;

    bp_printf("build        %s %s\n", __DATE__, __TIME__);
    bp_printf("MCU          DEV_ID 0x%03lX REV_ID 0x%04lX (STM32F446 is 0x%03X)\n",
              (unsigned long)dev_id, (unsigned long)rev_id, STM32F446_DEV_ID);
    bp_printf("flash        %u KB\n", *(const volatile uint16_t *)FLASHSIZE_BASE);
    bp_printf("UID          %08lX-%08lX-%08lX\n", (unsigned long)uid[2], (unsigned long)uid[1],
              (unsigned long)uid[0]);
    bp_printf("clocks       SYSCLK %lu  HCLK %lu  PCLK1 %lu  PCLK2 %lu Hz\n",
              (unsigned long)HAL_RCC_GetSysClockFreq(), (unsigned long)HAL_RCC_GetHCLKFreq(),
              (unsigned long)HAL_RCC_GetPCLK1Freq(), (unsigned long)HAL_RCC_GetPCLK2Freq());
    bp_printf("HSE          %s, %s\n", (RCC->CR & RCC_CR_HSEBYP) ? "bypass" : "crystal mode",
              (RCC->CR & RCC_CR_HSERDY) ? "ready" : "NOT ready");
    bp_printf("reset cause %s%s%s%s%s%s%s\n",
              (reset_flags & RCC_CSR_PORRSTF) ? " power-on" : "",
              (reset_flags & RCC_CSR_PINRSTF) ? " pin" : "",
              (reset_flags & RCC_CSR_BORRSTF) ? " brown-out" : "",
              (reset_flags & RCC_CSR_SFTRSTF) ? " software" : "",
              (reset_flags & RCC_CSR_IWDGRSTF) ? " iwdg" : "",
              (reset_flags & RCC_CSR_WWDGRSTF) ? " wwdg" : "",
              (reset_flags & RCC_CSR_LPWRRSTF) ? " low-power" : "");

    bp_result("swd.mcu_id", dev_id == STM32F446_DEV_ID ? BP_PASS : BP_FAIL, "dev_id=0x%03lX rev=0x%04lX",
              (unsigned long)dev_id, (unsigned long)rev_id);
    return 0;
}

static void print_help(void)
{
    bp_puts("Commands (run one without arguments for its usage):\n");
    for (size_t i = 0; i < sizeof commands / sizeof commands[0]; i++) {
        bp_printf("  %-6s %s\n", commands[i].name, commands[i].summary);
    }
    bp_puts("  help   this list; 'help <cmd>' shows a command's usage\n");
    bp_puts("Press any key to stop a running test.\n");
}

static const bp_cmd_t *find_command(const char *name)
{
    for (size_t i = 0; i < sizeof commands / sizeof commands[0]; i++) {
        if (bp_streq(name, commands[i].name)) {
            return &commands[i];
        }
    }
    return NULL;
}

static void dispatch(char *line)
{
    char *argv[MAX_ARGS];
    int argc = 0;

    for (char *tok = strtok(line, " \t"); tok && argc < MAX_ARGS; tok = strtok(NULL, " \t")) {
        argv[argc++] = tok;
    }
    if (argc == 0) {
        return;
    }

    if (bp_streq(argv[0], "help")) {
        const bp_cmd_t *cmd = argc > 1 ? find_command(argv[1]) : NULL;
        if (cmd) {
            char *help_argv[2] = {argv[1], "help"};
            cmd->fn(2, help_argv);
        } else {
            print_help();
        }
        return;
    }

    const bp_cmd_t *cmd = find_command(argv[0]);
    if (cmd) {
        cmd->fn(argc, argv);
    } else {
        bp_printf("unknown command '%s' (try 'help')\n", argv[0]);
    }
}

void bringup_run(void)
{
    char line[LINE_SIZE];

    bp_irq_unmask();
    hw_init();
    heartbeat_pin_init();

    bp_puts("\n\n== Charger_27 bring-up test firmware ==\n");
    bp_printf("build %s %s - type 'help'\n", __DATE__, __TIME__);
    bp_puts(PROMPT);

    for (;;) {
        bp_irq_unmask();
        heartbeat();
        if (bp_poll_line(line, sizeof line)) {
            dispatch(line);
            if (bp_heartbeat_enabled) {
                heartbeat_pin_init();
            }
            bp_puts(PROMPT);
        }
    }
}
