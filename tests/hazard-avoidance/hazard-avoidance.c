/* hazard-avoidance.c
 * MSP432 autonomous hazard avoidance test: drive forward while sweeping pan,
 * poll ToF distance, halt motors immediately when obstacle detected
 * (distance < CRITICAL_THRESHOLD_MM), then back up and turn.
 *
 * Architecture:
 *   In SIMULATED mode (SIMULATED=1): synthetic distance that decreases
 *     with "drive time" until threshold crossing.
 *   In real mode (SIMULATED=0): communicates with ESP32 companion sketch.
 *     MSP432 sends P,pan,tilt, receives D,dist back.
 *
 * Pins:
 *   P1.0  - LED (blink rate indicates state)
 *   P1.3  - USB UART TX (debug console)
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
 *
 * TimerA0: SMCLK/8=1.5MHz, UP mode, period=30000
 *   CCR0 = period, CCR1/2 = motor PWM, CCR3/4 = servo PWM
 * UART: 115200 baud
 */

#define SIMULATED 1

#include "driverlib.h"
#include <stdint.h>

/*===========================================================================*/
/* CONFIGURATION                                                             */
/*===========================================================================*/

#define PAN_MIN             0
#define PAN_MAX             110
#define PAN_CENTER          55
#define TILT_HORIZONTAL     90

#define CRITICAL_THRESHOLD_MM  150
#define SWEEP_STEP          2
#define SWEEP_DELAY_MS      30
#define POLL_INTERVAL_MS    20

#define SERVO_PERIOD        30000
#define MOTOR_PERIOD        30000
#define SERVO_MIN           1500
#define SERVO_MAX           3000
#define BASE_PWM            9000
#define RIGHT_RATIO         96
#define BACKUP_DIST_CM      10
#define TURN_DEG            45
#define SPEED_CM_PER_MS     0.038f

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
/* UART - USB Debug (EUSCI_A0, P1.2/P1.3)                                   */
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
/* MOTOR CONTROL (P3.x)                                                      */
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

void motor_wake(void) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
}

void motor_sleep(void) {
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
    motor_set_pwm(0, 0);
}

void motor_stop(void) {
    motor_set_pwm(0, 0);
}

void motor_forward(void) {
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);
    motor_wake();
    motor_set_pwm(BASE_PWM, BASE_PWM);
}

void motor_backward(void) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);
    motor_wake();
    motor_set_pwm(BASE_PWM, BASE_PWM);
}

void motor_turn_left(void) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    motor_wake();
    motor_set_pwm(BASE_PWM, BASE_PWM);
}

void motor_turn_right(void) {
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
    motor_wake();
    motor_set_pwm(BASE_PWM, BASE_PWM);
}

void motor_drive_cm(float cm, int dir_forward) {
    uint32_t time_ms;
    if (cm <= 0) return;
    time_ms = (uint32_t)(cm / SPEED_CM_PER_MS);
    if (dir_forward)
        motor_forward();
    else
        motor_backward();
    delay_ms(time_ms);
    motor_stop();
}

/*===========================================================================*/
/* SERVO PWM                                                                 */
/*===========================================================================*/

uint16_t deg_to_pulse(uint16_t deg) {
    if (deg > 180) deg = 180;
    return SERVO_MIN + ((uint32_t)deg * (SERVO_MAX - SERVO_MIN)) / 180;
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

void servo_center(void) {
    smooth_pan_to(PAN_CENTER);
    servo_set_tilt(TILT_HORIZONTAL);
}

/*===========================================================================*/
/* COMBINED TIMER INIT (Motors + Servos on TA0)                              */
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
/* DUAL OUTPUT                                                               */
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

void send_mode(const char *mode) {
    esp_print("M,"); esp_print(mode); esp_print("\r\n");
}

/*===========================================================================*/
/* DISTANCE ACQUISITION                                                      */
/*===========================================================================*/

#if SIMULATED

static uint32_t sim_drive_ticks = 0;

uint16_t read_distance(void) {
    uint32_t dist;

    sim_drive_ticks++;

    /* Distance starts at 1000mm and decreases linearly toward 0 */
    dist = 1000;
    if (sim_drive_ticks < 200) {
        dist = 1000 - (sim_drive_ticks * 5);
    }

    if (dist > 1000) dist = 1000;
    return (uint16_t)dist;
}

void reset_sim_distance(void) {
    sim_drive_ticks = 0;
}

#else /* SIMULATED */

int read_esp_distance(uint16_t *dist_out) {
    char c;
    int val = 0;
    int state = 0;

    while (1) {
        if (!esp_rx_available()) return 0;
        c = esp_getc();

        if (state == 0) {
            if (c == 'D') state = 1;
        } else if (state == 1) {
            if (c == ',') { state = 2; val = 0; }
            else state = 0;
        } else if (state == 2) {
            if (c >= '0' && c <= '9') {
                val = val * 10 + (c - '0');
            } else if (c == '\n') {
                *dist_out = (uint16_t)val;
                return 1;
            } else if (c != '\r') {
                state = 0;
            }
        }
    }
}

uint16_t poll_distance(void) {
    uint16_t dist = 0;
    uint32_t timeout = 0;

    esp_print("Q\r\n");

    while (!read_esp_distance(&dist) && timeout < 3000) {
        timeout++;
        delay_ms(1);
    }

    return (dist > 0) ? dist : 2000;
}

#endif /* SIMULATED */

/*===========================================================================*/
/* HAZARD AVOIDANCE LOOP                                                     */
/*===========================================================================*/

void hazard_loop(void) {
    int pan;
    int sweep_dir;
    uint16_t dist;
    int obstacle_hit;

    sweep_dir = 1;
    obstacle_hit = 0;

    send_msg(">> Hazard test start");
    send_mode("DRIVE");

    servo_center();
    delay_ms(500);

    /* Start driving forward */
    motor_forward();
    uart_print("Drive forward\r\n");

    smooth_pan_to(PAN_MIN);

    while (1) {
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

        /* Sweep pan in current direction */
        if (sweep_dir == 1) {
            for (pan = PAN_MIN; pan <= PAN_MAX; pan += SWEEP_STEP) {
                servo_set_pan((uint16_t)pan);
                delay_ms(SWEEP_DELAY_MS);

#if SIMULATED
                dist = read_distance();
#else
                send_pos((uint16_t)pan, TILT_HORIZONTAL);
                dist = poll_distance();
#endif

                uart_print("D ");
                uart_int(pan);
                uart_print(" ");
                uart_int(dist);
                uart_print("\r\n");

                if (dist > 0 && dist < CRITICAL_THRESHOLD_MM) {
                    obstacle_hit = 1;
                    break;
                }
            }
        } else {
            for (pan = PAN_MAX; pan >= PAN_MIN; pan -= SWEEP_STEP) {
                servo_set_pan((uint16_t)pan);
                delay_ms(SWEEP_DELAY_MS);

#if SIMULATED
                dist = read_distance();
#else
                send_pos((uint16_t)pan, TILT_HORIZONTAL);
                dist = poll_distance();
#endif

                uart_print("D ");
                uart_int(pan);
                uart_print(" ");
                uart_int(dist);
                uart_print("\r\n");

                if (dist > 0 && dist < CRITICAL_THRESHOLD_MM) {
                    obstacle_hit = 1;
                    break;
                }
            }
        }

        if (obstacle_hit) {
            uart_print(">> OBSTACLE! Distance ");
            uart_int(dist);
            uart_print(" mm at pan ");
            uart_int(pan);
            uart_print("\r\n");

            send_msg(">> OBSTACLE DETECTED");
            send_mode("HALT");

            /* Halt motors immediately */
            motor_stop();
            uart_print("Motors stopped\r\n");

            /* LED fast blink to indicate obstacle */
            {
                int i;
                for (i = 0; i < 10; i++) {
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                    delay_ms(50);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
                    delay_ms(50);
                }
            }

            send_mode("BACKUP");

            /* Back up */
            uart_print("Back up\r\n");
            motor_drive_cm(BACKUP_DIST_CM, 0);
            delay_ms(200);

            send_mode("TURN");

            /* Turn */
            uart_print("Turn\r\n");
            motor_turn_right();
            {
                uint32_t turn_ms = (uint32_t)(TURN_DEG * 5.5f);
                delay_ms(turn_ms);
            }
            motor_stop();
            delay_ms(300);

            send_mode("DRIVE");

            /* Reset and continue */
#if SIMULATED
            reset_sim_distance();
#endif
            obstacle_hit = 0;

            send_msg(">> Resume forward");
            motor_forward();

            sweep_dir = -sweep_dir;
        } else {
            /* No obstacle on full sweep, reverse direction */
            sweep_dir = -sweep_dir;
        }
    }
}

/*===========================================================================*/
/* MAIN                                                                      */
/*===========================================================================*/

int main(void) {
    WDT_A_holdTimer();
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    uart_init();
    esp_init();
    delay_ms(100);

    send_msg("Hazard Avoidance Test");

#if SIMULATED
    uart_print("Mode: SIMULATED (no ESP32)\r\n");
    uart_print("Distance decreases with drive time\r\n");
#else
    uart_print("Mode: REAL (ESP32 ToF required)\r\n");
#endif

    motor_pins_init();
    timer_init();
    delay_ms(500);

    send_msg("Motors + servos ready");

    servo_center();
    delay_ms(1000);

    send_msg("Starting hazard loop");
    hazard_loop();

    return 0;
}
