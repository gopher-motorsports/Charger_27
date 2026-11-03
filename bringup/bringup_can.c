#include "bringup.h"
#include "GopherCAN.h"
#include <stdio.h>
#include <string.h>

/* The charger's two outgoing parameter groups (standard 11-bit IDs, from go4-25.yaml):
 *   chargingPowerLimit group, ID 1877 (0x755), every 100 ms:
 *       bytes 0-1 chargingPowerLimit (float, scale 1, 16-bit big-endian), byte 2 forceEnableBalancing_state
 *   sdcStatus group, ID 1876 (0x754), about 4 Hz:
 *       bytes 0, 1, 2 = sdcStatus17, sdcStatus18, sdcStatus19
 * GopherCAN trims trailing zero bytes from the DLC. The expected frames below are built here, not by
 * the library, so the analyzer comparison also checks the library's encoding. */
#define PERIOD_POWER_LIMIT_MS   100U
#define PERIOD_SDC_MS           250U
#define PRODUCTION_LIMIT_W      6000.0f /* what j1772.c currently hard-codes */

#define LEC_NONE_CLEARED        7U

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[3];
} frame_t;

typedef struct {
    float limit_w;
    uint8_t balancing;
    uint8_t sdc17;
    uint8_t sdc18;
    uint8_t sdc19;
} params_t;

static volatile uint32_t rx_frames;

/* Overrides the library's weak hook; runs in the CAN receive interrupt. */
void GCAN_onRX(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    rx_frames++;
}

static void trim(frame_t *f, uint8_t full_len)
{
    f->dlc = full_len;
    while (f->dlc > 0U && f->data[f->dlc - 1U] == 0U) {
        f->dlc--;
    }
}

static void build_frames(const params_t *p, frame_t *power, frame_t *sdc)
{
    uint32_t raw = (uint32_t)(p->limit_w + 0.5f);

    power->id = 1877U;
    power->data[0] = (uint8_t)(raw >> 8);
    power->data[1] = (uint8_t)raw;
    power->data[2] = p->balancing;
    trim(power, 3U);

    sdc->id = 1876U;
    sdc->data[0] = p->sdc17;
    sdc->data[1] = p->sdc18;
    sdc->data[2] = p->sdc19;
    trim(sdc, 3U);
}

static void format_frame(const frame_t *f, char *out, size_t size)
{
    size_t n = (size_t)snprintf(out, size, "0x%03X [", f->id);

    for (uint8_t i = 0; i < f->dlc && n < size; i++) {
        n += (size_t)snprintf(out + n, size - n, i ? " %02X" : "%02X", f->data[i]);
    }
    if (n < size) {
        snprintf(out + n, size - n, "] dlc=%u", f->dlc);
    }
}

static void sdc_inputs_init(void)
{
    GPIO_InitTypeDef g = {0};

    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    g.Pin = GPIO_PIN_6 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOA, &g);
}

/* Production-like: the real limit and live SDC levels. Pattern: values chosen so every data bit is
 * exercised in both polarities and every byte is non-zero (so no trailing byte is trimmed). */
static params_t current_params(bool pattern)
{
    if (pattern) {
        return (params_t){.limit_w = 4660.0f, .balancing = 0x56U, .sdc17 = 0xA5U, .sdc18 = 0x5AU, .sdc19 = 0x3CU};
    }
    return (params_t){.limit_w = PRODUCTION_LIMIT_W,
                      .balancing = 0U,
                      .sdc17 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET ? 1U : 0U,
                      .sdc18 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET ? 1U : 0U,
                      .sdc19 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET ? 1U : 0U};
}

static void apply_params(const params_t *p)
{
    chargingPowerLimit.data = p->limit_w;
    forceEnableBalancing_state.data = p->balancing;
    sdcStatus17.data = p->sdc17;
    sdcStatus18.data = p->sdc18;
    sdcStatus19.data = p->sdc19;
}

static uint32_t bitrate(void)
{
    uint32_t btr = hcan2.Instance->BTR;
    uint32_t brp = (btr & 0x3FFU) + 1U;
    uint32_t ts1 = ((btr >> 16) & 0xFU) + 1U;
    uint32_t ts2 = ((btr >> 20) & 0x7U) + 1U;

    return HAL_RCC_GetPCLK1Freq() / (brp * (1U + ts1 + ts2));
}

typedef struct {
    uint32_t tec;
    uint32_t rec;
    uint32_t lec;
    bool warning;
    bool passive;
    bool bus_off;
} bus_status_t;

static bus_status_t read_status(void)
{
    uint32_t esr = hcan2.Instance->ESR;

    return (bus_status_t){.tec = (esr >> 16) & 0xFFU,
                          .rec = (esr >> 24) & 0xFFU,
                          .lec = (esr >> 4) & 7U,
                          .warning = (esr & CAN_ESR_EWGF) != 0U,
                          .passive = (esr & CAN_ESR_EPVF) != 0U,
                          .bus_off = (esr & CAN_ESR_BOFF) != 0U};
}

static const char *lec_name(uint32_t lec)
{
    static const char *const names[] = {"none", "stuff", "form", "ack", "bit-recessive", "bit-dominant", "crc",
                                        "none(cleared)"};
    return names[lec & 7U];
}

static bool bus_clean(const bus_status_t *s)
{
    return s->tec == 0U && s->rec == 0U && !s->bus_off && (s->lec == 0U || s->lec == LEC_NONE_CLEARED);
}

static void print_status(void)
{
    bus_status_t s = read_status();

    bp_printf("bus: %lu bit/s  TEC=%lu REC=%lu LEC=%s%s%s%s  rx_frames=%lu\n", (unsigned long)bitrate(),
              (unsigned long)s.tec, (unsigned long)s.rec, lec_name(s.lec), s.warning ? " warning" : "",
              s.passive ? " error-passive" : "", s.bus_off ? " BUS-OFF" : "", (unsigned long)rx_frames);
}

static params_t params_for(const bp_can_tx_t *t)
{
    params_t p = current_params(t->pattern);

    if (!t->pattern) {
        p.limit_w = t->limit_w;
    }
    return p;
}

void bp_can_tx_start(bp_can_tx_t *t, bool pattern, float limit_w)
{
    uint32_t now = HAL_GetTick();

    *t = (bp_can_tx_t){.pattern = pattern, .limit_w = limit_w, .next_power_ms = now, .next_sdc_ms = now};
    sdc_inputs_init();
    hcan2.Instance->ESR = CAN_ESR_LEC;
    rx_frames = 0;
}

void bp_can_tx_tick(bp_can_tx_t *t)
{
    uint32_t now = HAL_GetTick();
    params_t p = params_for(t);

    apply_params(&p);
    if ((int32_t)(now - t->next_power_ms) >= 0) {
        t->next_power_ms += PERIOD_POWER_LIMIT_MS;
        if (send_group(chargingPowerLimit.info.GROUP_ID) == CAN_SUCCESS) {
            t->sent_power++;
        } else {
            t->failures++;
        }
    }
    if ((int32_t)(now - t->next_sdc_ms) >= 0) {
        t->next_sdc_ms += PERIOD_SDC_MS;
        if (send_group(sdcStatus17.info.GROUP_ID) == CAN_SUCCESS) {
            t->sent_sdc++;
        } else {
            t->failures++;
        }
    }
    service_can_tx(&hcan2);
    service_can_rx_buffer();
    bp_irq_unmask();
}

void bp_can_tx_report(const bp_can_tx_t *t, const char *id)
{
    bus_status_t s = read_status();
    bool pass = t->failures == 0U && bus_clean(&s);

    bp_result(id, pass ? BP_PASS : BP_FAIL,
              "%s power=%lu sdc=%lu queue_failures=%lu TEC=%lu REC=%lu LEC=%s bitrate=%lu",
              t->pattern ? "pattern" : "live", (unsigned long)t->sent_power, (unsigned long)t->sent_sdc,
              (unsigned long)t->failures, (unsigned long)s.tec, (unsigned long)s.rec, lec_name(s.lec),
              (unsigned long)bitrate());
}

static void print_expected(const bp_can_tx_t *t)
{
    params_t p = params_for(t);
    frame_t power;
    frame_t sdc;
    char power_text[40];
    char sdc_text[40];

    build_frames(&p, &power, &sdc);
    format_frame(&power, power_text, sizeof power_text);
    format_frame(&sdc, sdc_text, sizeof sdc_text);
    bp_printf("  %s every %lu ms | %s every %lu ms\n", power_text, (unsigned long)PERIOD_POWER_LIMIT_MS, sdc_text,
              (unsigned long)PERIOD_SDC_MS);
}

static void tx_loop(bool pattern, uint32_t seconds)
{
    bp_can_tx_t t;
    uint32_t start = HAL_GetTick();
    uint32_t next_print = start + 1000U;

    bp_can_tx_start(&t, pattern, PRODUCTION_LIMIT_W);

    if (pattern) {
        bp_puts("BENCH ONLY: these are the real charger IDs carrying test values (power limit 4660 W,\n"
                "forceEnableBalancing_state 0x56). Do not run this with a BMS or the car on the bus.\n");
    }
    bp_printf("sending %s values, press any key to stop. Expect on the analyzer:\n",
              pattern ? "fixed-pattern" : "production-like");
    bp_printf("  %lu bit/s, standard 11-bit IDs, a node must ACK or TEC will climb\n", (unsigned long)bitrate());

    while (!bp_key_pressed() && (seconds == 0U || HAL_GetTick() - start < seconds * 1000U)) {
        bp_can_tx_tick(&t);
        if ((int32_t)(HAL_GetTick() - next_print) >= 0) {
            next_print += 1000U;
            print_expected(&t);
            print_status();
        }
    }
    bp_can_tx_report(&t, "can.tx");
}

/* Group 0x300 (768) carries bmsBatteryCurrent_A (bytes 0-1, scale 0.01, offset -300) and
 * bmsBatteryVoltage_V (bytes 2-3, scale 0.1). Printing the decoded values proves the frame was not
 * just accepted by the filter but parsed by GopherCAN. */
static void rx_loop(uint32_t seconds)
{
    uint32_t start = HAL_GetTick();
    uint32_t next_print = start + 1000U;

    rx_frames = 0;
    bmsBatteryCurrent_A.info.last_rx = 0;
    bmsBatteryVoltage_V.info.last_rx = 0;
    bp_puts("listening, press any key to stop. Send 11-bit frames with a GopherCAN group ID, e.g.\n"
            "0x300 [7A 02 02 90] -> bmsBatteryCurrent_A 12.34 A, bmsBatteryVoltage_V 65.60 V\n");
    while (!bp_key_pressed() && (seconds == 0U || HAL_GetTick() - start < seconds * 1000U)) {
        service_can_rx_buffer();
        bp_irq_unmask();
        if ((int32_t)(HAL_GetTick() - next_print) >= 0) {
            next_print += 1000U;
            print_status();
        }
    }
    service_can_rx_buffer();
    bp_irq_unmask();
    bp_printf("decoded: bmsBatteryCurrent_A=%.2f (rx at %lu ms) bmsBatteryVoltage_V=%.2f (rx at %lu ms)\n",
              (double)bmsBatteryCurrent_A.data, (unsigned long)bmsBatteryCurrent_A.info.last_rx,
              (double)bmsBatteryVoltage_V.data, (unsigned long)bmsBatteryVoltage_V.info.last_rx);
    bp_result("can.rx", BP_INFO, "rx_frames=%lu current=%.2f voltage=%.2f", (unsigned long)rx_frames,
              (double)bmsBatteryCurrent_A.data, (double)bmsBatteryVoltage_V.data);
}

static void usage(void)
{
    bp_puts("Plan row            Command\n"
            "Eye diagram         can blast [seconds]   continuous charger frames until a key (alias: eye, tx)\n"
            "Parameter frames    can tx [seconds]      both charger groups with production-like values (limit 6000 W,\n"
            "                                          live SDC levels): 0x755 every 100 ms, 0x754 every 250 ms\n"
            "can pattern [seconds]  same IDs and rates with fixed values: 0x755 = 12 34 56, 0x754 = A5 5A 3C\n"
            "                       BENCH ONLY - never with a BMS or the car on the bus\n"
            "can rx [seconds]       count received frames and print the decoded 0x300 BMS values\n"
            "can stat               bit rate, error counters, last error code\n"
            "Needs another node on the bus to ACK frames (Kvaser / analyzer, bus terminated to ~60 ohm).\n"
            "PC side: tools/bringup_host.py run --groups can (checks IDs, payloads, periods, both directions)\n");
}

int bp_cmd_can(int argc, char **argv)
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

    if ((bp_streq(argv[1], "tx") || bp_streq(argv[1], "blast") || bp_streq(argv[1], "eye")
         || bp_streq(argv[1], "pattern")) && argc <= 3) {
        tx_loop(bp_streq(argv[1], "pattern"), seconds);
    } else if (bp_streq(argv[1], "rx") && argc <= 3) {
        rx_loop(seconds);
    } else if (bp_streq(argv[1], "stat") && argc == 2) {
        print_status();
        bus_status_t s = read_status();
        bp_result("can.stat", bus_clean(&s) ? BP_PASS : BP_INFO, "TEC=%lu REC=%lu LEC=%s bitrate=%lu",
                  (unsigned long)s.tec, (unsigned long)s.rec, lec_name(s.lec), (unsigned long)bitrate());
    } else {
        usage();
        return bp_streq(argv[1], "help") ? 0 : -1;
    }
    return 0;
}
