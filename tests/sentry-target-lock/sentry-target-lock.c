/* sentry-target-lock.c
 * MSP432 sentry mode test: continuous pan sweep, ToF distance polling,
 * target detection via rolling baseline, and gradient-descent tracking.
 *
 * Target detection:
 *   Maintains a rolling baseline of recent distances. If the newest reading
 *   drops significantly below the baseline, a target is declared.
 *
 * Tracking (Gradient Descent):
 *   Each cycle: measure at current pan, step TRACK_STEP in one direction
 *   (alternating left/right), measure again. The gradient (distance change
 *   per degree) drives the P-controller. No return-to-center — the servo
 *   moves from the dither position toward the next target position.
 *
 *   gain = -(d_step - d_cur) / (TRACK_STEP * GRADIENT_SCALE)
 *
 *   With GRADIENT_SCALE = 8 (mm/deg normalization), a 50mm drop over 3°
 *   gives ~2°/cycle towards the target. The clamp at ±MAX_MOVE prevents
 *   jerky motion.
 *
 * SIMULATED mode uses a quadratic distance bowl so gradient = 0 at center
 * and grows linearly with offset — a realistic proxy for a real object.
 *
 * Pins:
 *   P1.0  - LED (slow blink = sweeping, fast blink = tracking)
 *   P1.3  - USB UART TX
 *   P2.4  - Left motor PWM (TA0.1)
 *   P2.5  - Right motor PWM (TA0.2)
 *   P2.6  - Pan servo (TA0.3)
 *   P2.7  - Tilt servo (TA0.4)
 *   P3.0  - Right motor SLP
 *   P3.2  - ESP32 UART RX (EUSCI_A2 RX)
 *   P3.3  - ESP32 UART TX (EUSCI_A2 TX)
 *   P3.5  - Right motor DIR
 *   P3.6  - Left motor SLP
 *   P3.7  - Left motor DIR
 */

#define SIMULATED 1

#include "driverlib.h"
#include <stdint.h>

#define PAN_MIN             0
#define PAN_MAX             110
#define PAN_CENTER          55
#define TILT_LOCK           90

#define SERVO_PERIOD        30000
#define PULSE_MIN           1500
#define PULSE_MAX           3000

#define SWEEP_STEP          2
#define SWEEP_DELAY_MS      30

#define TRACK_STEP          3
#define TRACK_MAX_MOVE      4
#define GRADIENT_SCALE      8

#define BASELINE_WINDOW     8
#define DETECT_FACTOR       60
#define BASELINE_DIST_MM    1000
#define TARGET_LOST_MAX     8

#define SWEEP_LOST_MAX      5

/* Motor follower */
#define MOTOR_PERIOD        30000
#define FOLLOW_PWM          4000
#define RIGHT_RATIO         96
#define STOP_DIST_MM        200
#define FOLLOW_HYSTERESIS   15

static uint16_t cur_pan = 90;
static uint16_t cur_tilt = 90;

/* Rolling baseline */
static uint16_t base_buf[BASELINE_WINDOW];
static int base_idx = 0;
static int base_filled = 0;
static uint16_t base_median = BASELINE_DIST_MM;

/*===========================================================================*/
/* DELAY                                                                     */
/*===========================================================================*/

void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 3000; j++)
            __no_operation();
}

/*===========================================================================*/
/* UART - USB (EUSCI_A0, P1.2/P1.3) 115200                                  */
/*===========================================================================*/

const eUSCI_UART_Config uartCfg = {
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,
    6, 8, 32,
    EUSCI_A_UART_NO_PARITY,
    EUSCI_A_UART_LSB_FIRST,
    EUSCI_A_UART_ONE_STOP_BIT,
    EUSCI_A_UART_MODE,
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

void uart_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A0_BASE, &uartCfg);
    UART_enableModule(EUSCI_A0_BASE);
}

void uart_putc(char c) {
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = c;
}

void uart_print(const char *s) {
    while (*s) uart_putc(*s++);
}

void uart_int(int32_t val) {
    char buf[12];
    int i = 0;
    if (val < 0) { uart_putc('-'); val = -val; }
    if (val == 0) { uart_putc('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}

/*===========================================================================*/
/* ESP32 UART (EUSCI_A2, P3.2/P3.3) - bidirectional                         */
/*===========================================================================*/

void esp_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A2_BASE, &uartCfg);
    UART_enableModule(EUSCI_A2_BASE);
}

void esp_putc(char c) {
    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A2->TXBUF = c;
}

void esp_print(const char *s) {
    while (*s) esp_putc(*s++);
}

#if !SIMULATED
void esp_int(int32_t val) {
    char buf[12];
    int i = 0;
    if (val < 0) { esp_putc('-'); val = -val; }
    if (val == 0) { esp_putc('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) esp_putc(buf[--i]);
}
#endif

int esp_rx_available(void) {
    return (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG) != 0;
}

char esp_getc(void) {
    while (!esp_rx_available());
    return EUSCI_A2->RXBUF;
}

/*===========================================================================*/
/* SERVO PWM                                                                 */
/*===========================================================================*/

uint16_t deg_to_pulse(uint16_t deg) {
    if (deg > 180) deg = 180;
    return PULSE_MIN + ((uint32_t)deg * (PULSE_MAX - PULSE_MIN)) / 180;
}

void servo_set_pan(uint16_t deg) {
    cur_pan = deg;
    TIMER_A0->CCR[3] = deg_to_pulse(deg);
}

void servo_set_tilt(uint16_t deg) {
    cur_tilt = deg;
    TIMER_A0->CCR[4] = deg_to_pulse(deg);
}

void smooth_pan_to(uint16_t target) {
    if (target == cur_pan) return;
    int16_t step = (target > cur_pan) ? 1 : -1;
    while (cur_pan != target) {
        servo_set_pan(cur_pan + step);
        delay_ms(SWEEP_DELAY_MS);
    }
}

void smooth_tilt_to(uint16_t target) {
    if (target == cur_tilt) return;
    int16_t step = (target > cur_tilt) ? 1 : -1;
    while (cur_tilt != target) {
        servo_set_tilt(cur_tilt + step);
        delay_ms(SWEEP_DELAY_MS);
    }
}

/*===========================================================================*/
/* MOTOR CONTROL                                                             */
/*===========================================================================*/

void motor_pins_init(void) {
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);
}

void motor_set_pwm(uint16_t left, uint16_t right) {
    right = (right * RIGHT_RATIO) / 100;
    if (left > MOTOR_PERIOD) left = MOTOR_PERIOD;
    if (right > MOTOR_PERIOD) right = MOTOR_PERIOD;
    TIMER_A0->CCR[1] = left;
    TIMER_A0->CCR[2] = right;
}

void motor_stop(void) {
    motor_set_pwm(0, 0);
}

void motor_forward(void) {
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
    motor_set_pwm(FOLLOW_PWM, FOLLOW_PWM);
}

void motor_turn_left(void) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
    motor_set_pwm(FOLLOW_PWM, FOLLOW_PWM);
}

void motor_turn_right(void) {
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
    motor_set_pwm(FOLLOW_PWM, FOLLOW_PWM);
}

/*===========================================================================*/
/* TIMER INIT (Motors CCR1/2 + Servos CCR3/4 on TA0)                        */
/*===========================================================================*/

void timer_init(void) {
    TIMER_A0->CTL = 0;
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
        GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7,
        GPIO_PRIMARY_MODULE_FUNCTION);
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |
                    TIMER_A_CTL_ID__8 |
                    TIMER_A_CTL_CLR;
    TIMER_A0->CCR[0] = SERVO_PERIOD;
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[1] = 0;
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[2] = 0;
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[3] = deg_to_pulse(90);
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[4] = deg_to_pulse(90);
    TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;
}

/*===========================================================================*/
/* UART OUTPUT                                                               */
/*===========================================================================*/

void send_msg(const char *msg) {
    uart_print(msg); uart_print("\r\n");
    esp_print(msg); esp_print("\r\n");
}

#if !SIMULATED
void send_pos(uint16_t pan, uint16_t tilt) {
    esp_print("P,"); esp_int(pan);
    esp_putc(','); esp_int(tilt);
    esp_print("\r\n");
}
#endif

/*===========================================================================*/
/* ROLLING BASELINE                                                          */
/*===========================================================================*/

void baseline_add(uint16_t dist) {
    base_buf[base_idx] = dist;
    base_idx = (base_idx + 1) % BASELINE_WINDOW;
    if (!base_filled && base_idx == 0) base_filled = 1;

    if (base_filled) {
        uint16_t sorted[BASELINE_WINDOW];
        int i, j;
        uint16_t tmp;
        for (i = 0; i < BASELINE_WINDOW; i++) sorted[i] = base_buf[i];

        for (i = 0; i < BASELINE_WINDOW - 1; i++) {
            for (j = i + 1; j < BASELINE_WINDOW; j++) {
                if (sorted[j] < sorted[i]) {
                    tmp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = tmp;
                }
            }
        }
        base_median = sorted[BASELINE_WINDOW / 2];
    }
}

void baseline_reset(void) {
    base_filled = 0;
    base_idx = 0;
    base_median = BASELINE_DIST_MM;
}

int detect_target(uint16_t dist) {
    if (!base_filled) return 0;
    return (dist < base_median - DETECT_FACTOR);
}

/*===========================================================================*/
/* DISTANCE ACQUISITION                                                      */
/*===========================================================================*/

#if SIMULATED

/* Quadratic bowl: dist = BASELINE - STRENGTH * max(0, 1 - offset^2 / WIDTH^2)
 * Gradient is 0 at center, grows linearly with offset — realistic. */

#define BOWL_WIDTH_DEG      35
#define BOWL_WIDTH_SQ       ((int32_t)BOWL_WIDTH_DEG * BOWL_WIDTH_DEG)
#define BOWL_STRENGTH       850

static uint16_t sim_target_angle = 70;
static int sim_frame = 0;

static void sim_set_target(uint16_t angle) {
    sim_target_angle = angle;
}

uint16_t read_distance(uint16_t pan_deg) {
    int32_t offset_sq;
    uint32_t dist;

    sim_frame++;

    if (sim_frame % 300 == 0) {
        if (sim_target_angle >= PAN_MAX - 8)
            sim_target_angle = PAN_MIN + 8;
        else
            sim_target_angle += 4;
    }

    offset_sq = (int32_t)pan_deg - (int32_t)sim_target_angle;
    offset_sq = (offset_sq < 0) ? -offset_sq : offset_sq;
    offset_sq = offset_sq * offset_sq;

    if (offset_sq < BOWL_WIDTH_SQ) {
        dist = BASELINE_DIST_MM - BOWL_STRENGTH +
               (BOWL_STRENGTH * offset_sq) / BOWL_WIDTH_SQ;
    } else {
        dist = BASELINE_DIST_MM;
    }

    return (uint16_t)dist;
}

#else /* SIMULATED */

int read_esp_distance(uint16_t *dist_out) {
    char c;
    int val = 0;
    int state = 0;

    while (1) {
        if (!esp_rx_available()) return 0;
        c = esp_getc();

        if (state == 0) { if (c == 'D') state = 1; }
        else if (state == 1) { if (c == ',') { state = 2; val = 0; } else state = 0; }
        else if (state == 2) {
            if (c >= '0' && c <= '9') val = val * 10 + (c - '0');
            else if (c == '\n') { *dist_out = (uint16_t)val; return 1; }
            else if (c != '\r') state = 0;
        }
    }
}

uint16_t read_distance(uint16_t pan_deg) {
    uint16_t dist = 0;
    uint32_t timeout = 0;

    send_pos(pan_deg, TILT_LOCK);

    while (!read_esp_distance(&dist) && timeout < 100) {
        timeout++;
        delay_ms(1);
    }

    return (dist > 0) ? dist : 2000;
}

#endif /* SIMULATED */

/*===========================================================================*/
/* TRACKING — GRADIENT DESCENT                                              */
/*===========================================================================*/

void track_loop(uint16_t lock_angle) {
    uint16_t d_cur;
    uint16_t d_step;
    int16_t delta;
    int adjust;
    int dither_pos;
    int dither_dir;
    int lost_count;
    uint16_t min_dist_ever;
    uint16_t min_angle_ever;

    lost_count = 0;
    dither_dir = 1;
    min_dist_ever = 0xFFFF;
    min_angle_ever = lock_angle;

    smooth_pan_to(lock_angle);
    delay_ms(200);

    uart_print("TRACK_START\r\n");

    while (1) {
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

        /* ---- Step 1: read at current position ---- */
        d_cur = read_distance(cur_pan);

        if (d_cur < min_dist_ever) {
            min_dist_ever = d_cur;
            min_angle_ever = cur_pan;
        }

        /* ---- Step 2: dither in current direction, read ---- */
        dither_pos = (int)cur_pan + dither_dir * TRACK_STEP;
        if (dither_pos < PAN_MIN) { dither_pos = PAN_MIN; dither_dir = 1; }
        if (dither_pos > PAN_MAX) { dither_pos = PAN_MAX; dither_dir = -1; }
        smooth_pan_to((uint16_t)dither_pos);
        d_step = read_distance((uint16_t)dither_pos);

        if (d_step < min_dist_ever) {
            min_dist_ever = d_step;
            min_angle_ever = dither_pos;
        }

        /* ---- Step 3: gradient and P-control ---- */
        /* delta = d_step - d_cur.
         * delta > 0: distance increased in dither_dir → target is opposite
         * delta < 0: distance decreased in dither_dir → target is in dither_dir
         * adjust = -(delta) / (TRACK_STEP * GRADIENT_SCALE)
         *   = -(mm change) / (degrees * mm/deg)
         *   = degrees of adjustment
         * Negative sign: move toward decreasing distance */
        delta = (int16_t)d_step - (int16_t)d_cur;
        adjust = -(delta * 1) / (TRACK_STEP * GRADIENT_SCALE);

        if (adjust < -TRACK_MAX_MOVE) adjust = -TRACK_MAX_MOVE;
        if (adjust > TRACK_MAX_MOVE) adjust = TRACK_MAX_MOVE;

        /* ---- Step 4: move to next position (from dither_pos) ---- */
        {
            int next_pan = dither_pos + adjust;
            if (next_pan < PAN_MIN) next_pan = PAN_MIN;
            if (next_pan > PAN_MAX) next_pan = PAN_MAX;
            smooth_pan_to((uint16_t)next_pan);
        }

        /* ---- Debug ---- */
        uart_print("T ");
        uart_int(dither_dir > 0 ? dither_pos - TRACK_STEP : dither_pos + TRACK_STEP);
        uart_print(" "); uart_int(d_cur);
        uart_print(" "); uart_int(d_step);
        uart_print(" D "); uart_int(delta);
        uart_print(" A "); uart_int(adjust);
        uart_print("\r\n");

        /* ---- Adaptive target lost check ---- */
        {
            uint16_t lost_thresh = base_filled ? (base_median - 20) : BASELINE_DIST_MM - 20;
            if (d_cur > lost_thresh && d_step > lost_thresh) {
                lost_count++;
                if (lost_count >= TARGET_LOST_MAX) {
                    uart_print("TARGET_LOST\r\n");
                    motor_stop();
                    delay_ms(300);
                    return;
                }
            } else {
                lost_count = 0;
            }
        }

        /* ---- Follower: drive toward target ---- */
        if (cur_pan < PAN_CENTER - FOLLOW_HYSTERESIS) {
            motor_turn_left();
        } else if (cur_pan > PAN_CENTER + FOLLOW_HYSTERESIS) {
            motor_turn_right();
        } else if (d_cur < STOP_DIST_MM && d_step < STOP_DIST_MM) {
            motor_stop();
        } else {
            motor_forward();
        }

        /* Alternate dither direction */
        dither_dir = -dither_dir;

        delay_ms(20);
    }
}

/*===========================================================================*/
/* SWEEP LOOP                                                               */
/*===========================================================================*/

void sentry_loop(void) {
    int pan;
    uint16_t dist;
    int lost_in_sweep;

    baseline_reset();
    lost_in_sweep = 0;

    while (1) {
        send_msg("SWEEP");

        /* Forward sweep: PAN_MIN → PAN_MAX */
        smooth_pan_to(PAN_MIN);
        for (pan = PAN_MIN; pan <= PAN_MAX; pan += SWEEP_STEP) {
            servo_set_pan((uint16_t)pan);
            delay_ms(SWEEP_DELAY_MS);

            dist = read_distance((uint16_t)pan);
            baseline_add(dist);

            uart_print("S "); uart_int(pan);
            uart_print(" "); uart_int(dist);
            uart_print(" B "); uart_int(base_median);
            uart_print("\r\n");

            if (base_filled && detect_target(dist)) {
                uart_print(">> LOCK "); uart_int(pan);
                uart_print(" "); uart_int(dist);
                uart_print("\r\n");
                track_loop((uint16_t)pan);
                send_msg("RESUME");
                baseline_reset();
                lost_in_sweep = 0;
                break;
            }
        }

        /* Reverse sweep: PAN_MAX → PAN_MIN */
        smooth_pan_to(PAN_MAX);
        for (pan = PAN_MAX; pan >= PAN_MIN; pan -= SWEEP_STEP) {
            servo_set_pan((uint16_t)pan);
            delay_ms(SWEEP_DELAY_MS);

            dist = read_distance((uint16_t)pan);
            baseline_add(dist);

            uart_print("S "); uart_int(pan);
            uart_print(" "); uart_int(dist);
            uart_print(" B "); uart_int(base_median);
            uart_print("\r\n");

            if (base_filled && detect_target(dist)) {
                uart_print(">> LOCK "); uart_int(pan);
                uart_print(" "); uart_int(dist);
                uart_print("\r\n");
                track_loop((uint16_t)pan);
                send_msg("RESUME");
                baseline_reset();
                lost_in_sweep = 0;
                break;
            }
        }
    }
}

/*===========================================================================*/
/* MAIN                                                                      */
/*===========================================================================*/

int main(void) {
    WDT_A_holdTimer();
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    uart_init();
    esp_init();
    delay_ms(100);

    send_msg("Sentry Target Lock v2");

#if SIMULATED
    uart_print("SIMULATED\r\n");
#else
    uart_print("REAL MODE\r\n");
#endif

    motor_pins_init();
    timer_init();
    motor_stop();
    send_msg("MOTORS_OK SERVOS_OK");

    smooth_pan_to(PAN_CENTER);
    smooth_tilt_to(TILT_LOCK);
    delay_ms(500);

    send_msg("BEGIN");
    sentry_loop();

    return 0;
}
