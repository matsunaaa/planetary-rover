/* sentry-target-lock.c
 * MSP432 sentry mode: sweep pan, detect target when dist < 300mm,
 * lock pan angle, drive forward slowly, stop & re-sweep when lost.
 *
 * Pins:
 *   P1.0  - LED
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

#include "driverlib.h"
#include <stdint.h>

#define PAN_MIN             0
#define PAN_MAX             110
#define PAN_CENTER          55
#define TILT_LOCK           90

#define SWEEP_STEP          3
#define SWEEP_DELAY_MS      30

#define SERVO_PERIOD        30000
#define PULSE_MIN           1500
#define PULSE_MAX           3000

#define MOTOR_PERIOD        30000
#define RIGHT_RATIO         96
#define FOLLOW_PWM          3000
#define STOP_DIST_MM        150
#define TARGET_THRESHOLD_MM 300

static uint16_t cur_pan = 90;
static uint16_t cur_tilt = 90;

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
/* UART - USB (EUSCI_A0, P1.2/P1.3)                                         */
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
/* ESP32 UART (EUSCI_A2, P3.2/P3.3)                                         */
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

int esp_rx_available(void) {
    return (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG) != 0;
}

char esp_getc(void) {
    while (!esp_rx_available());
    return EUSCI_A2->RXBUF;
}

/*===========================================================================*/
/* SERVO                                                                     */
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
/* MOTOR                                                                     */
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

/*===========================================================================*/
/* TIMER INIT                                                                */
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

/*===========================================================================*/
/* ESP32 PROTOCOL: ESP32 streams D,dist continuously; read latest line       */
/*===========================================================================*/

static char line_buf[32];
static int line_pos = 0;

int read_esp_line(void) {
    while (esp_rx_available() && line_pos < 31) {
        char c = esp_getc();
        if (c == '\n') {
            line_buf[line_pos] = '\0';
            line_pos = 0;
            return 1;
        }
        if (c != '\r') {
            line_buf[line_pos++] = c;
        }
    }
    return 0;
}

uint16_t read_latest_distance(void) {
    uint16_t d = 0;
    int i, val;

    while (read_esp_line()) {
        if (line_buf[0] == 'D' && line_buf[1] == ',') {
            val = 0;
            for (i = 2; line_buf[i] >= '0' && line_buf[i] <= '9'; i++) {
                val = val * 10 + (line_buf[i] - '0');
            }
            if (val > 0) d = (uint16_t)val;
        }
    }

    return d;
}

uint16_t query_distance(uint16_t pan_deg) {
    uint16_t dist;
    uint32_t timeout = 0;

    (void)pan_deg;

    while (timeout < 200) {
        dist = read_latest_distance();
        if (dist > 0) return dist;
        timeout++;
        delay_ms(1);
    }

    return 4000;
}

/*===========================================================================*/
/* FOLLOW TARGET                                                             */
/*===========================================================================*/

void follow_target(uint16_t lock_angle) {
    uint16_t dist;

    smooth_pan_to(lock_angle);
    smooth_tilt_to(TILT_LOCK);
    delay_ms(200);

    send_msg("FOLLOW");
    uart_print("LOCK "); uart_int(lock_angle); uart_print("\r\n");

    while (1) {
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

        dist = query_distance(cur_pan);

        uart_print("F "); uart_int(cur_pan); uart_print(" ");
        uart_int(dist); uart_print("\r\n");

        /* Target lost — dist back up above threshold */
        if (dist >= TARGET_THRESHOLD_MM) {
            uart_print("LOST "); uart_int(dist); uart_print("\r\n");
            motor_stop();
            return;
        }

        /* Too close — stop but keep checking */
        if (dist < STOP_DIST_MM) {
            motor_stop();
        } else {
            motor_forward();
        }

        delay_ms(50);
    }
}

/*===========================================================================*/
/* SCAN LOOP                                                                 */
/*===========================================================================*/

void sentry_loop(void) {
    int pan;
    uint16_t dist;

    while (1) {
        send_msg("SCAN");
        smooth_pan_to(PAN_MIN);

        /* Forward sweep */
        for (pan = PAN_MIN; pan <= PAN_MAX; pan += SWEEP_STEP) {
            servo_set_pan((uint16_t)pan);
            delay_ms(SWEEP_DELAY_MS);

            dist = query_distance((uint16_t)pan);

            uart_print("S "); uart_int(pan); uart_print(" ");
            uart_int(dist); uart_print("\r\n");

            if (dist < TARGET_THRESHOLD_MM) {
                follow_target((uint16_t)pan);
                send_msg("RESCAN");
                break;
            }
        }

        /* Reverse sweep fallback */
        smooth_pan_to(PAN_MAX);
        for (pan = PAN_MAX; pan >= PAN_MIN; pan -= SWEEP_STEP) {
            servo_set_pan((uint16_t)pan);
            delay_ms(SWEEP_DELAY_MS);

            dist = query_distance((uint16_t)pan);

            uart_print("S "); uart_int(pan); uart_print(" ");
            uart_int(dist); uart_print("\r\n");

            if (dist < TARGET_THRESHOLD_MM) {
                follow_target((uint16_t)pan);
                send_msg("RESCAN");
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
    motor_pins_init();
    timer_init();
    motor_stop();
    delay_ms(100);

    send_msg("Sentry Target Lock v4 (simple threshold)");

    smooth_pan_to(PAN_CENTER);
    smooth_tilt_to(TILT_LOCK);
    delay_ms(500);

    send_msg("BEGIN");
    sentry_loop();

    return 0;
}
