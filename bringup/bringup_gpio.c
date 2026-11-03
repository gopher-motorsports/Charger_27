#include "bringup.h"
#include <string.h>

#define F_OUT       0x01U /* firmware may drive it */
#define F_IN        0x02U /* firmware may read it as an input */
#define F_HARNESS   0x04U /* wired to a connector: never touched by 'walk' */
#define F_SENSITIVE 0x08U /* changes external hardware: never touched by 'walk' */

typedef struct {
    const char *name;
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t flags;
    const char *where;
} pin_t;

/* From the schematic netlist (Charger_27_Board_Interfaces, sections 4, 5, 7, 8). */
static const pin_t pins[] = {
    {"HEARTBEAT_LED", GPIOC, GPIO_PIN_0,  F_OUT, "LD1 green"},
    {"GSENSE_LED",    GPIOC, GPIO_PIN_1,  F_OUT, "LD2 blue"},
    {"FAULT_LED",     GPIOC, GPIO_PIN_2,  F_OUT, "LD3 red"},
    {"D3_FULL",       GPIOA, GPIO_PIN_0,  F_OUT, "D3 segment 1"},
    {"D3_HALF",       GPIOA, GPIO_PIN_3,  F_OUT, "D3 segment 2"},
    {"D3_EMPTY",      GPIOA, GPIO_PIN_4,  F_OUT, "D3 segment 3"},
    {"CP_EN",         GPIOC, GPIO_PIN_3,  F_OUT | F_SENSITIVE, "LD6 + Q2 gate (state-C load)"},
    {"GEN_MCU1",      GPIOB, GPIO_PIN_8,  F_OUT | F_IN | F_HARNESS, "J2-19"},
    {"GEN_MCU2",      GPIOB, GPIO_PIN_9,  F_OUT | F_IN | F_HARNESS, "J2-17"},
    {"GEN_MCU3",      GPIOB, GPIO_PIN_10, F_OUT | F_IN | F_HARNESS, "J2-6"},
    {"GEN_MCU4",      GPIOB, GPIO_PIN_12, F_OUT | F_IN | F_HARNESS, "J2-8"},
    {"SDC1",          GPIOC, GPIO_PIN_6,  F_IN, "J2-12"},
    {"SDC2",          GPIOA, GPIO_PIN_12, F_IN, "J2-14"},
    {"SDC3",          GPIOC, GPIO_PIN_8,  F_IN, "J2-10"},
    {"SDC4",          GPIOA, GPIO_PIN_11, F_IN, "J2-16"},
};

#define NUM_PINS (sizeof pins / sizeof pins[0])
#define MAX_WATCH 8

static const pin_t *find_pin(const char *name)
{
    for (size_t i = 0; i < NUM_PINS; i++) {
        if (bp_streq(name, pins[i].name)) {
            return &pins[i];
        }
    }
    return NULL;
}

static int pin_index(const pin_t *p)
{
    return __builtin_ctz(p->pin);
}

static char port_letter(const pin_t *p)
{
    return p->port == GPIOA ? 'A' : (p->port == GPIOB ? 'B' : 'C');
}

static void set_mode(const pin_t *p, uint32_t mode)
{
    GPIO_InitTypeDef g = {0};

    g.Pin = p->pin;
    g.Mode = mode;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(p->port, &g);
}

static void drive(const pin_t *p, bool high)
{
    HAL_GPIO_WritePin(p->port, p->pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    set_mode(p, GPIO_MODE_OUTPUT_PP);
}

static int level(const pin_t *p)
{
    return HAL_GPIO_ReadPin(p->port, p->pin) == GPIO_PIN_SET ? 1 : 0;
}

static const char *mode_name(const pin_t *p)
{
    static const char *const names[] = {"in", "out", "af", "analog"};

    return names[(p->port->MODER >> (2U * (uint32_t)pin_index(p))) & 3U];
}

static void usage(void)
{
    bp_puts("gpio list                     pins, schematic location, mode, level\n"
            "gpio out <NAME> <0|1>         drive a pin push-pull and hold it\n"
            "gpio in <NAME>                make a pin a floating input and print the level\n"
            "gpio read [NAME...]           print levels without changing any pin mode\n"
            "gpio expect <NAME> <0|1>      PASS if the pin reads that level\n"
            "gpio watch [NAME...]          report changes (default GEN_MCU1-4 and SDC1-4)\n"
            "gpio sq <NAME> [hz]           square wave for the scope (default 1000 Hz, max 200000)\n"
            "gpio walk [ms]                light each LED in turn (default 700 ms)\n"
            "gpio heartbeat <on|off>       resume or pause the LD1 heartbeat blink\n"
            "Walk skips connector pins and CP_EN; drive those by name. 'gpio out HEARTBEAT_LED' pauses\n"
            "the heartbeat so the pin holds its level for a meter.\n");
}

static void list_pins(void)
{
    bp_puts("NAME           PIN   WHERE                          MODE    LEVEL\n");
    for (size_t i = 0; i < NUM_PINS; i++) {
        bp_printf("%-14s P%c%-3d %-30s %-7s %d\n", pins[i].name, port_letter(&pins[i]),
                  pin_index(&pins[i]), pins[i].where, mode_name(&pins[i]), level(&pins[i]));
    }
}

void bp_watch_pins(const char *const *names, int count)
{
    const pin_t *sel[MAX_WATCH];
    int last[MAX_WATCH];
    int n = 0;

    for (int i = 0; i < count && n < MAX_WATCH; i++) {
        const pin_t *p = find_pin(names[i]);
        if (!p || !(p->flags & F_IN)) {
            bp_printf("skipping '%s': not a readable input\n", names[i]);
            continue;
        }
        set_mode(p, GPIO_MODE_INPUT);
        sel[n] = p;
        last[n] = level(p);
        bp_printf("%-10s = %d\n", p->name, last[n]);
        n++;
    }
    if (n == 0) {
        return;
    }

    bp_puts("watching, press any key to stop\n");
    while (!bp_key_pressed()) {
        for (int i = 0; i < n; i++) {
            int now = level(sel[i]);
            if (now != last[i]) {
                last[i] = now;
                bp_printf("[%8lu ms] %-10s -> %d\n", (unsigned long)HAL_GetTick(), sel[i]->name, now);
            }
        }
    }
}

static void square_wave(const pin_t *p, uint32_t hz)
{
    uint32_t half = SystemCoreClock / (2U * hz);
    uint32_t next;
    bool high = false;

    drive(p, false);
    bp_printf("%s toggling at %lu Hz, press any key to stop\n", p->name, (unsigned long)hz);
    next = bp_cycles() + half;
    while (!bp_key_pressed()) {
        if ((int32_t)(bp_cycles() - next) >= 0) {
            next += half;
            high = !high;
            HAL_GPIO_WritePin(p->port, p->pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }
    drive(p, false);
}

static void walk(uint32_t ms)
{
    bp_puts("walking LEDs, press any key to stop\n");
    for (size_t i = 0; i < NUM_PINS; i++) {
        if (!(pins[i].flags & F_OUT) || (pins[i].flags & (F_HARNESS | F_SENSITIVE))) {
            continue;
        }
        drive(&pins[i], true);
        bp_printf("%-14s ON  (%s)\n", pins[i].name, pins[i].where);
        uint32_t start = HAL_GetTick();
        while (HAL_GetTick() - start < ms) {
            if (bp_key_pressed()) {
                drive(&pins[i], false);
                return;
            }
        }
        drive(&pins[i], false);
    }
    bp_puts("walk done\n");
    bp_heartbeat_enabled = true;
}

int bp_cmd_gpio(int argc, char **argv)
{
    const pin_t *p;
    uint32_t v;

    if (argc < 2) {
        usage();
        return 0;
    }

    if (bp_streq(argv[1], "list")) {
        list_pins();
    } else if (bp_streq(argv[1], "out") && argc == 4 && (p = find_pin(argv[2])) != NULL
               && (p->flags & F_OUT) && bp_parse_u32(argv[3], &v) && v <= 1U) {
        if (p->port == GPIOC && p->pin == GPIO_PIN_0) {
            bp_heartbeat_enabled = false;
        }
        drive(p, v == 1U);
        bp_printf("%s (P%c%d, %s) driven %lu%s\n", p->name, port_letter(p), pin_index(p), p->where,
                  (unsigned long)v, bp_heartbeat_enabled ? "" : " (heartbeat paused)");
    } else if (bp_streq(argv[1], "heartbeat") && argc == 3
               && (bp_streq(argv[2], "on") || bp_streq(argv[2], "off"))) {
        bp_heartbeat_enabled = bp_streq(argv[2], "on");
        bp_printf("heartbeat %s\n", bp_heartbeat_enabled ? "on" : "off");
    } else if (bp_streq(argv[1], "in") && argc == 3 && (p = find_pin(argv[2])) != NULL
               && (p->flags & F_IN)) {
        set_mode(p, GPIO_MODE_INPUT);
        bp_printf("%s (P%c%d, %s) = %d\n", p->name, port_letter(p), pin_index(p), p->where, level(p));
    } else if (bp_streq(argv[1], "read")) {
        if (argc == 2) {
            list_pins();
        }
        for (int i = 2; i < argc; i++) {
            p = find_pin(argv[i]);
            if (p) {
                bp_printf("%-14s = %d\n", p->name, level(p));
            } else {
                bp_printf("unknown pin '%s'\n", argv[i]);
            }
        }
    } else if (bp_streq(argv[1], "expect") && argc == 4 && (p = find_pin(argv[2])) != NULL
               && bp_parse_u32(argv[3], &v) && v <= 1U) {
        int now = level(p);
        bp_result("gpio.expect", now == (int)v ? BP_PASS : BP_FAIL, "%s read=%d expected=%lu", p->name, now,
                  (unsigned long)v);
    } else if (bp_streq(argv[1], "watch")) {
        static const char *const defaults[] = {"GEN_MCU1", "GEN_MCU2", "GEN_MCU3", "GEN_MCU4",
                                               "SDC1", "SDC2", "SDC3", "SDC4"};
        if (argc > 2) {
            bp_watch_pins((const char *const *)&argv[2], argc - 2);
        } else {
            bp_watch_pins(defaults, (int)(sizeof defaults / sizeof defaults[0]));
        }
    } else if (bp_streq(argv[1], "sq") && argc >= 3 && (p = find_pin(argv[2])) != NULL
               && (p->flags & F_OUT)) {
        uint32_t hz = 1000U;
        if (argc > 3 && (!bp_parse_u32(argv[3], &hz) || hz == 0U || hz > 200000U)) {
            usage();
            return -1;
        }
        square_wave(p, hz);
    } else if (bp_streq(argv[1], "walk")) {
        uint32_t ms = 700U;
        if (argc > 2 && !bp_parse_u32(argv[2], &ms)) {
            usage();
            return -1;
        }
        walk(ms);
    } else {
        usage();
        return bp_streq(argv[1], "help") ? 0 : -1;
    }
    return 0;
}

/* ---- shutdown-circuit sense: SDC1..SDC4 read HIGH when the SDC line is energized ---- */

static void sdc_usage(void)
{
    bp_puts("sdc                  read SDC1-4 (1 = energized/good, 0 = open/fault)\n"
            "sdc expect <bits>    PASS if levels match, SDC1 first, e.g. 'sdc expect 1111'\n"
            "sdc watch            report changes with a millisecond timestamp\n");
}

int bp_cmd_sdc(int argc, char **argv)
{
    static const char *const names[] = {"SDC1", "SDC2", "SDC3", "SDC4"};
    char bits[5];

    if (argc == 2 && bp_streq(argv[1], "help")) {
        sdc_usage();
        return 0;
    }
    if (argc == 2 && bp_streq(argv[1], "watch")) {
        bp_watch_pins(names, 4);
        return 0;
    }

    for (int i = 0; i < 4; i++) {
        const pin_t *p = find_pin(names[i]);
        set_mode(p, GPIO_MODE_INPUT);
        bits[i] = level(p) ? '1' : '0';
    }
    bits[4] = '\0';

    if (argc == 1 || (argc == 2 && bp_streq(argv[1], "read"))) {
        for (int i = 0; i < 4; i++) {
            const pin_t *p = find_pin(names[i]);
            bp_printf("%s (%s, P%c%d) = %c\n", names[i], p->where, port_letter(p), pin_index(p), bits[i]);
        }
        bp_result("sdc.read", BP_INFO, "%s", bits);
        return 0;
    }
    if (argc == 3 && bp_streq(argv[1], "expect") && strlen(argv[2]) == 4U) {
        bp_result("sdc.expect", strcmp(argv[2], bits) == 0 ? BP_PASS : BP_FAIL, "read=%s expected=%s", bits,
                  argv[2]);
        return 0;
    }
    sdc_usage();
    return -1;
}
