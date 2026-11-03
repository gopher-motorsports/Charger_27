#include "bringup.h"
#include "j1772.h"

/* Variables the production capture interrupt in main.c maintains. */
extern volatile uint32_t cpFrequency;
extern volatile uint32_t cpHighTime;
extern volatile uint32_t cpDutyCycle;
extern volatile uint32_t cpLastUpdate;

/* CP_PWM = PB7 (TIM4_CH2), CP_EN = PC3, PP_ADC = PA2 (ADC1_IN2). */
#define CP_EN_PORT          GPIOC
#define CP_EN_PIN           GPIO_PIN_3
#define CP_PWM_PORT         GPIOB
#define CP_PWM_PIN          GPIO_PIN_7
#define PP_PIN              GPIO_PIN_2

#define CP_PRESENT_MS       50U
#define VREFINT_CAL_ADDRESS 0x1FFF7A2AU /* VREFINT reading taken at VDDA = 3.3 V */
#define ADC_FULL_SCALE      4095U
#define PP_PULLUP_OHMS      1000U       /* R19 */
#define ADC_AVERAGES        32U

typedef struct {
    bool present;
    uint32_t age_ms;
    uint32_t period_us;
    uint32_t high_us;
    float hz;
    float duty;
} cp_t;

static bool amps_from_duty(float duty, float *amps)
{
    if (duty >= 10.0f && duty <= 85.0f) {
        *amps = duty * 0.6f;
        return true;
    }
    if (duty > 85.0f && duty <= 96.0f) {
        *amps = (duty - 64.0f) * 2.5f;
        return true;
    }
    return false;
}

/* Decoded straight from the TIM4 capture registers. CH2 holds the period (counter reset on each
 * rising edge) and CH1 the high time, in timer ticks. */
static cp_t read_cp(void)
{
    cp_t c = {0};
    uint32_t last = cpLastUpdate;
    uint32_t tick_hz = HAL_RCC_GetPCLK1Freq() * 2U / (htim4.Instance->PSC + 1U);
    uint32_t period = htim4.Instance->CCR2;
    uint32_t high = htim4.Instance->CCR1;

    c.age_ms = HAL_GetTick() - last;
    c.present = last != 0U && c.age_ms < CP_PRESENT_MS && period != 0U;
    if (c.present) {
        c.period_us = (uint32_t)(((uint64_t)period * 1000000U) / tick_hz);
        c.high_us = (uint32_t)(((uint64_t)high * 1000000U) / tick_hz);
        c.hz = (float)tick_hz / (float)period;
        c.duty = 100.0f * (float)high / (float)period;
    }
    return c;
}

static void print_cp(const cp_t *c)
{
    int pb7 = HAL_GPIO_ReadPin(CP_PWM_PORT, CP_PWM_PIN) == GPIO_PIN_SET ? 1 : 0;
    int en = HAL_GPIO_ReadPin(CP_EN_PORT, CP_EN_PIN) == GPIO_PIN_SET ? 1 : 0;
    float amps;

    if (!c->present) {
        if (cpLastUpdate == 0U) {
            bp_printf("no pilot PWM (no edge seen since boot)  PB7=%d  CP_EN=%d\n", pb7, en);
        } else {
            bp_printf("no pilot PWM (last edge %lu ms ago)  PB7=%d  CP_EN=%d\n", (unsigned long)c->age_ms, pb7, en);
        }
        return;
    }
    bp_printf("%.1f Hz  period %lu us  high %lu us  duty %.1f %%", (double)c->hz, (unsigned long)c->period_us,
              (unsigned long)c->high_us, (double)c->duty);
    if (amps_from_duty(c->duty, &amps)) {
        bp_printf("  -> %.1f A", (double)amps);
    } else {
        bp_puts("  -> no current limit encoded at this duty");
    }
    bp_printf("  PB7=%d CP_EN=%d | production vars: freq=%lu high=%lu duty=%lu\n", pb7, en,
              (unsigned long)cpFrequency, (unsigned long)cpHighTime, (unsigned long)cpDutyCycle);
}

static void set_cp_en(bool on)
{
    HAL_GPIO_WritePin(CP_EN_PORT, CP_EN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Runs the production decision (getJ1772Status) with production CAN sending, so the real timeout,
 * CP_EN behavior and the power-limit frames can all be observed together (the Lucid charger test). */
static void run_auto(void)
{
    chargingData_S data = {0};
    bp_can_tx_t tx;
    bool prev_en = HAL_GPIO_ReadPin(CP_EN_PORT, CP_EN_PIN) == GPIO_PIN_SET;
    uint32_t next_print = HAL_GetTick();

    bp_can_tx_start(&tx, false, 0.0f);
    bp_puts("running the production CP logic and sending the charger CAN frames, press any key to stop\n");
    while (!bp_key_pressed()) {
        getJ1772Status(&data);
        tx.limit_w = data.powerLimit;
        bp_can_tx_tick(&tx);
        bool en = HAL_GPIO_ReadPin(CP_EN_PORT, CP_EN_PIN) == GPIO_PIN_SET;

        if (!prev_en && en) {
            bp_puts("pilot detected: CP_EN raised\n");
        } else if (prev_en && !en) {
            uint32_t ms = HAL_GetTick() - cpLastUpdate;
            bp_printf("pilot lost: CP_EN dropped %lu ms after the last pilot edge\n", (unsigned long)ms);
            bp_result("cp.loss", BP_INFO, "latency_ms=%lu (production timeout 10 ms, +/-1 ms tick)", (unsigned long)ms);
        }
        prev_en = en;

        if ((int32_t)(HAL_GetTick() - next_print) >= 0) {
            next_print += 500U;
            cp_t c = read_cp();
            print_cp(&c);
        }
        HAL_Delay(1);
    }
    set_cp_en(false);
    bp_can_tx_report(&tx, "cp.auto.can");
}

static void cp_usage(void)
{
    bp_puts("cp                 decode the pilot: frequency, high time, duty, amps, PB7/CP_EN levels\n"
            "cp watch           the same every 250 ms until a key\n"
            "cp en <0|1>        force CP_EN (PC3): LD6 and the state-C load R14 on the CP line\n"
            "cp auto            production CP logic + charger CAN frames (0x755/0x754); reports CP_EN\n"
            "                   changes and loss-of-pilot latency. Use for the Lucid charger session test\n");
}

int bp_cmd_cp(int argc, char **argv)
{
    uint32_t v;

    if (argc < 2 || bp_streq(argv[1], "read")) {
        cp_t c = read_cp();
        print_cp(&c);
        bp_result("cp.read", BP_INFO, "present=%d hz=%.1f high_us=%lu duty=%.1f", c.present ? 1 : 0, (double)c.hz,
                  (unsigned long)c.high_us, (double)c.duty);
    } else if (bp_streq(argv[1], "watch")) {
        bp_puts("press any key to stop\n");
        while (!bp_key_pressed()) {
            cp_t c = read_cp();
            print_cp(&c);
            HAL_Delay(250);
        }
    } else if (bp_streq(argv[1], "en") && argc == 3 && bp_parse_u32(argv[2], &v) && v <= 1U) {
        set_cp_en(v == 1U);
        bp_printf("CP_EN %s\n", v ? "high (state-C load connected)" : "low");
    } else if (bp_streq(argv[1], "auto")) {
        run_auto();
    } else {
        cp_usage();
        return bp_streq(argv[1], "help") ? 0 : -1;
    }
    return 0;
}

/* ---- proximity pilot ---- */

typedef enum {
    PP_OPEN,
    PP_150,
    PP_480,
    PP_UNKNOWN
} pp_state_t;

static const char *const pp_names[] = {"open (unplugged)", "150 ohm (plugged, latch released)",
                                       "480 ohm (plugged, latch pressed)", "out of range"};

/* Mean of n conversions with a long sample time: the source is R19 || the plug through R20 + C15. */
static int32_t adc_mean(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};
    uint32_t sum = 0;

    cfg.Channel = channel;
    cfg.Rank = 1;
    cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) {
        return -1;
    }
    for (uint32_t i = 0; i < ADC_AVERAGES; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) {
            HAL_ADC_Stop(&hadc1);
            return -1;
        }
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    return (int32_t)(sum / ADC_AVERAGES);
}

static pp_state_t classify(float ratio)
{
    /* Expected ratio is R / (R + 1 kOhm), accepted within +/-10 percent. */
    if (ratio >= 0.90f) {
        return PP_OPEN;
    }
    if (ratio >= 0.1304f * 0.9f && ratio <= 0.1304f * 1.1f) {
        return PP_150;
    }
    if (ratio >= 0.3243f * 0.9f && ratio <= 0.3243f * 1.1f) {
        return PP_480;
    }
    return PP_UNKNOWN;
}

static pp_state_t read_pp(bool announce)
{
    GPIO_InitTypeDef g = {0};
    int32_t raw;
    int32_t vref;
    float vdda_mv = 3300.0f;
    uint16_t cal = *(const volatile uint16_t *)VREFINT_CAL_ADDRESS;

    g.Pin = PP_PIN;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);

    raw = adc_mean(ADC_CHANNEL_2);
    vref = adc_mean(ADC_CHANNEL_VREFINT);
    if (raw < 0) {
        bp_result("pp.read", BP_FAIL, "ADC conversion failed");
        return PP_UNKNOWN;
    }
    if (vref > 0 && cal != 0U && cal != 0xFFFFU) {
        vdda_mv = 3300.0f * (float)cal / (float)vref;
    }

    float ratio = (float)raw / (float)ADC_FULL_SCALE;
    float mv = ratio * vdda_mv;
    pp_state_t state = classify(ratio);

    if (announce) {
        if (raw >= (int32_t)ADC_FULL_SCALE - 4) {
            bp_printf("PP_ADC raw %ld = %.0f mV (VDDA %.0f mV)  -> %s\n", (long)raw, (double)mv, (double)vdda_mv,
                      pp_names[state]);
        } else {
            bp_printf("PP_ADC raw %ld = %.0f mV (VDDA %.0f mV)  PP resistor about %.0f ohm  -> %s\n", (long)raw,
                      (double)mv, (double)vdda_mv,
                      (double)((float)PP_PULLUP_OHMS * (float)raw / (float)(ADC_FULL_SCALE - (uint32_t)raw)),
                      pp_names[state]);
        }
    }
    return state;
}

static void pp_usage(void)
{
    bp_puts("pp                      read PP_ADC (PA2): millivolts, plug resistor, plug state\n"
            "pp watch                the same every 250 ms until a key (step the plug latch)\n"
            "pp expect <open|150|480> PASS if the state matches\n");
}

int bp_cmd_pp(int argc, char **argv)
{
    if (argc == 1) {
        pp_state_t s = read_pp(true);
        bp_result("pp.read", BP_INFO, "%s", pp_names[s]);
    } else if (argc == 2 && bp_streq(argv[1], "watch")) {
        bp_puts("press any key to stop\n");
        while (!bp_key_pressed()) {
            read_pp(true);
            HAL_Delay(250);
        }
    } else if (argc == 3 && bp_streq(argv[1], "expect")) {
        pp_state_t want = bp_streq(argv[2], "open") ? PP_OPEN
                          : bp_streq(argv[2], "150") ? PP_150
                          : bp_streq(argv[2], "480") ? PP_480
                                                     : PP_UNKNOWN;
        if (want == PP_UNKNOWN) {
            pp_usage();
            return -1;
        }
        pp_state_t got = read_pp(true);
        bp_result("pp.expect", got == want ? BP_PASS : BP_FAIL, "read=%s expected=%s", pp_names[got], pp_names[want]);
    } else {
        pp_usage();
        return (argc == 2 && bp_streq(argv[1], "help")) ? 0 : -1;
    }
    return 0;
}
