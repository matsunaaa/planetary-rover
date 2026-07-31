/* servo-sweep.c
 * MSP432 servo sweep: moves pan/tilt servos, sends position via UART.
 * Pan: 0-110 deg, Tilt: 37-180 deg. All movements are gradual — no snaps.
 *
 * Pins:
 *   P1.0  - LED
 *   P1.3  - USB UART TX (debug console)
 *   P2.6  - Pan servo (TA0.3)
 *   P2.7  - Tilt servo (TA0.4)
 *   P3.3  - ESP32 UART TX (EUSCI_A2)
 *
 * TimerA0: SMCLK/8=1.5MHz, period=30000 -> 50Hz servo PWM
 *
 * Dual UART output: P,pan,tilt\r\n  (USB + ESP32)
 */

#include "driverlib.h"
#include <stdint.h>

#define PAN_MIN         0
#define PAN_MAX         110
#define TILT_MIN        37
#define TILT_MAX        180
#define SMOOTH_DELAY_MS 30
#define SETTLE_MS       50
#define SERVO_PERIOD    30000
#define PULSE_MIN       1500
#define PULSE_MAX       3000

static uint16_t cur_pan = 90;
static uint16_t cur_tilt = 90;

/*===========================================================================*/
/* DELAY                                                                      */
/*===========================================================================*/
void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 3000; j++)
            __no_operation();
}

/*===========================================================================*/
/* UART (EUSCI_A0, P1.2/P1.3) - 115200 baud                                */
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

void uart_print(const char *str) {
    while (*str) uart_putc(*str++);
}

void uart_print_int(uint16_t val) {
    char buf[6];
    int i = 0;
    if (val == 0) { uart_putc('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}

/*===========================================================================*/
/* ESP32 UART (EUSCI_A2, P3.2/P3.3) - to ESP32 GPIO16                      */
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

void esp_print(const char *str) {
    while (*str) esp_putc(*str++);
}

void esp_print_int(uint16_t val) {
    char buf[6];
    int i = 0;
    if (val == 0) { esp_putc('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) esp_putc(buf[--i]);
}

/*===========================================================================*/
/* Dual output helper                                                        */
/*===========================================================================*/
void send_pos(uint16_t pan, uint16_t tilt) {
    uart_print("P,"); uart_print_int(pan);
    uart_putc(','); uart_print_int(tilt);
    uart_print("\r\n");

    esp_print("P,"); esp_print_int(pan);
    esp_putc(','); esp_print_int(tilt);
    esp_print("\r\n");
}

void send_msg(const char *msg) {
    uart_print(msg); uart_print("\r\n");
    esp_print(msg); esp_print("\r\n");
}

/*===========================================================================*/
/* SERVO PWM - TimerA0, UP mode, 50Hz                                      */
/*===========================================================================*/
uint16_t deg_to_pulse(uint16_t deg) {
    if (deg > 180) deg = 180;
    return PULSE_MIN + ((uint32_t)deg * (PULSE_MAX - PULSE_MIN)) / 180;
}

void set_pwm(uint8_t reg, uint16_t pulse) {
    if (pulse < PULSE_MIN) pulse = PULSE_MIN;
    if (pulse > PULSE_MAX) pulse = PULSE_MAX;
    Timer_A_CompareModeConfig cmp = {
        reg,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_RESET_SET,
        pulse
    };
    Timer_A_initCompare(TIMER_A0_BASE, &cmp);
}

void set_pan(uint16_t deg) {
    cur_pan = deg;
    set_pwm(TIMER_A_CAPTURECOMPARE_REGISTER_3, deg_to_pulse(deg));
}

void set_tilt(uint16_t deg) {
    cur_tilt = deg;
    set_pwm(TIMER_A_CAPTURECOMPARE_REGISTER_4, deg_to_pulse(deg));
}

void smooth_pan_to(uint16_t target) {
    if (target == cur_pan) return;
    int16_t step = (target > cur_pan) ? 1 : -1;
    while (cur_pan != target) {
        set_pan(cur_pan + step);
        delay_ms(SMOOTH_DELAY_MS);
    }
}

void smooth_tilt_to(uint16_t target) {
    if (target == cur_tilt) return;
    int16_t step = (target > cur_tilt) ? 1 : -1;
    while (cur_tilt != target) {
        set_tilt(cur_tilt + step);
        delay_ms(SMOOTH_DELAY_MS);
    }
}

void servo_init(void) {
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
        GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    Timer_A_UpModeConfig upCfg = {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_8,
        SERVO_PERIOD,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
        TIMER_A_DO_CLEAR
    };
    Timer_A_configureUpMode(TIMER_A0_BASE, &upCfg);

    set_pan(90);
    set_tilt(90);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

/*===========================================================================*/
/* SCAN SWEEPS                                                              */
/*===========================================================================*/

void test_pan_sweep(void) {
    send_msg("Pan sweep");
    smooth_pan_to(PAN_MAX);
    smooth_pan_to(PAN_MIN);
}

void test_tilt_sweep(void) {
    send_msg("Tilt sweep");
    smooth_tilt_to(TILT_MAX);
    smooth_tilt_to(TILT_MIN);
}

void test_pan_tilt_pattern(void) {
    int tilt, pan;
    send_msg("Raster scan");

    smooth_tilt_to(TILT_MIN);

    for (tilt = TILT_MIN; tilt <= TILT_MAX; tilt += 2) {
        smooth_pan_to((tilt == TILT_MIN || ((tilt - TILT_MIN) / 2) % 2 == 0)
                      ? PAN_MIN : PAN_MAX);

        if (((tilt - TILT_MIN) / 2) % 2 == 0) {
            for (pan = PAN_MAX; pan >= PAN_MIN; pan -= 2) {
                set_pan((uint16_t)pan);
                delay_ms(SETTLE_MS);
                send_pos((uint16_t)pan, (uint16_t)tilt);
            }
        } else {
            for (pan = PAN_MIN; pan <= PAN_MAX; pan += 2) {
                set_pan((uint16_t)pan);
                delay_ms(SETTLE_MS);
                send_pos((uint16_t)pan, (uint16_t)tilt);
            }
        }
        if (tilt + 2 <= TILT_MAX)
            smooth_tilt_to((uint16_t)(tilt + 2));
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
    send_msg("Servo Sweep Test");
    send_msg("Pan: 0-110, Tilt: 37-180");

    servo_init();
    send_msg("Servos ready");

    smooth_pan_to(55);
    smooth_tilt_to(108);
    delay_ms(1000);

    while (1) {
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

        test_pan_sweep();
        delay_ms(1000);

        test_tilt_sweep();
        delay_ms(1000);

        send_msg("Raster scan start");
        test_pan_tilt_pattern();
        send_msg("Raster scan done");
        delay_ms(2000);

        send_msg("Repeat");
    }
}
