/* servo-tof-dashboard.c
 * 3D scanning test: SG90 pan/tilt servos + VL53L1X ToF sensor
 * Raster scan: tilt 37->180, pan 84->0, ToF reading at each position
 * Dual UART output: USB debug (EUSCI_A0) + ESP32 bridge (EUSCI_A2)
 *
 * Pins:
 *   P1.0  - LED debug
 *   P1.6  - I2C SDA (ToF)
 *   P1.7  - I2C SCL (ToF)
 *   P2.6  - TA0.3 PWM - Pan servo
 *   P2.7  - TA0.4 PWM - Tilt servo
 *   P3.2  - EUSCI_A2 RX (ESP32 TX, unused)
 *   P3.3  - EUSCI_A2 TX (ESP32 RX)
 *   P4.0  - XSHUT (ToF reset)
 *
 * TimerA0: SMCLK/16 = 750kHz, period 15000 -> 50Hz servo PWM
 * CCR3 = P2.6 pan, CCR4 = P2.7 tilt
 *
 * Output: D,dist,status\r\n  (real-time distance)
 *         S,panAngle,tiltAngle,dist\r\n  (3D scan point)
 */

#include "driverlib.h"
#include <stdint.h>
#include <stdbool.h>

#define XSHUT_PORT  GPIO_PORT_P4
#define XSHUT_PIN   GPIO_PIN0
#define VL53L1X_ADDR 0x29

/* Sweep parameters */
#define TILT_MIN    37
#define TILT_MAX    180
#define PAN_MIN     0
#define PAN_MAX     180
#define SCAN_STEP   2
#define SERVO_SLEEP_MS  15
#define TOF_SETTLE_MS   30

/* Servo pulse: period=15000 ticks at 750kHz (20ms)
 * 0deg = 1ms = 750 ticks, 180deg = 2ms = 1500 ticks */
#define PULSE_0DEG     750
#define PULSE_180DEG   1500
#define PULSE_RANGE    (PULSE_180DEG - PULSE_0DEG)

static const uint8_t VL53L1X_DEFAULT_CONFIG[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
    0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00
};

/*===========================================================================*/
/* UART config - working 115200 @ 12MHz SMCLK                                */
/*===========================================================================*/
const eUSCI_UART_Config uartConfig = {
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,
    6, 8, 32,
    EUSCI_A_UART_NO_PARITY,
    EUSCI_A_UART_LSB_FIRST,
    EUSCI_A_UART_ONE_STOP_BIT,
    EUSCI_A_UART_MODE,
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

/*===========================================================================*/
/* I2C config - 100kHz                                                       */
/*===========================================================================*/
const eUSCI_I2C_MasterConfig i2cConfig = {
    EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
    12000000,
    EUSCI_B_I2C_SET_DATA_RATE_100KBPS,
    0,
    EUSCI_B_I2C_NO_AUTO_STOP
};

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
/* USB UART (EUSCI_A0, P1.2/P1.3)                                           */
/*===========================================================================*/
void usb_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    UART_enableModule(EUSCI_A0_BASE);
}

void usb_putc(char c) {
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = c;
}

void usb_puts(const char *str) {
    while (*str) usb_putc(*str++);
}

void usb_print_int(uint16_t val) {
    char buf[6];
    int i = 0;
    if (val == 0) { usb_putc('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) usb_putc(buf[--i]);
}

/*===========================================================================*/
/* BT UART (EUSCI_A2, P3.2/P3.3) - to ESP32                                */
/*===========================================================================*/
void bt_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A2_BASE, &uartConfig);
    UART_enableModule(EUSCI_A2_BASE);
}

void bt_putc(char c) {
    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A2->TXBUF = c;
}

void bt_puts(const char *str) {
    while (*str) bt_putc(*str++);
}

void bt_print_int(uint16_t val) {
    char buf[6];
    int i = 0;
    if (val == 0) { bt_putc('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) bt_putc(buf[--i]);
}

/*===========================================================================*/
/* Dual output helper                                                        */
/*===========================================================================*/
void send_line(const char *prefix, uint16_t a, uint16_t b, uint16_t c) {
    usb_puts(prefix);
    usb_print_int(a);
    usb_putc(',');
    usb_print_int(b);
    usb_putc(',');
    usb_print_int(c);
    usb_puts("\r\n");

    bt_puts(prefix);
    bt_print_int(a);
    bt_putc(',');
    bt_print_int(b);
    bt_putc(',');
    bt_print_int(c);
    bt_puts("\r\n");
}

void send_raw(const char *str) {
    usb_puts(str);
    bt_puts(str);
}

/*===========================================================================*/
/* LED                                                                       */
/*===========================================================================*/
void led_init(void) {
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void led_on(void)   { GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); }
void led_off(void)  { GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); }

void led_blink(int times) {
    int i;
    for (i = 0; i < times; i++) {
        led_on(); delay_ms(100);
        led_off(); delay_ms(100);
    }
}

/*===========================================================================*/
/* I2C - direct register access                                              */
/*===========================================================================*/
void i2c_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    I2C_enableModule(EUSCI_B0_BASE);
}

bool i2c_write_reg16(uint16_t reg, uint8_t data) {
    uint32_t timeout;
    EUSCI_B0->I2CSA = VL53L1X_ADDR;
    timeout = 10000;
    while ((EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY) && --timeout);
    if (timeout == 0) return false;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);
    if (timeout == 0) return false;
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG) {
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        return false;
    }
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = reg & 0xFF;
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = data;
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);
    return true;
}

uint8_t i2c_read_reg8(uint16_t reg) {
    uint32_t timeout;
    uint8_t data = 0;
    EUSCI_B0->I2CSA = VL53L1X_ADDR;
    timeout = 10000;
    while ((EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY) && --timeout);
    if (timeout == 0) return 0;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);
    if (timeout == 0) return 0;
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = reg & 0xFF;
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);
    data = EUSCI_B0->RXBUF;
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);
    return data;
}

uint16_t i2c_read_reg16(uint16_t reg) {
    uint32_t timeout;
    uint8_t hi = 0, lo = 0;
    EUSCI_B0->I2CSA = VL53L1X_ADDR;
    timeout = 10000;
    while ((EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY) && --timeout);
    if (timeout == 0) return 0;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = reg & 0xFF;
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);
    hi = EUSCI_B0->RXBUF;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);
    lo = EUSCI_B0->RXBUF;
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);
    return ((uint16_t)hi << 8) | lo;
}

/*===========================================================================*/
/* VL53L1X ToF Sensor                                                        */
/*===========================================================================*/
void vl53l1x_reset(void) {
    GPIO_setAsOutputPin(XSHUT_PORT, XSHUT_PIN);
    GPIO_setOutputLowOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(50);
    GPIO_setOutputHighOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(50);
}

bool vl53l1x_init(void) {
    uint8_t modelId, bootState;
    int i;

    modelId = i2c_read_reg8(0x010F);
    if (modelId != 0xEA) return false;

    for (i = 0; i < 100; i++) {
        bootState = i2c_read_reg8(0x00E5);
        if (bootState == 0x03) break;
        delay_ms(10);
    }
    if (bootState != 0x03) return false;

    for (i = 0; i < sizeof(VL53L1X_DEFAULT_CONFIG); i++) {
        if (!i2c_write_reg16(0x002D + i, VL53L1X_DEFAULT_CONFIG[i]))
            return false;
    }

    i2c_write_reg16(0x0086, 0x01);
    i2c_write_reg16(0x0087, 0x40);
    delay_ms(50);
    return true;
}

uint16_t vl53l1x_get_distance(void) {
    uint8_t dataReady;
    dataReady = i2c_read_reg8(0x0031);
    if (!(dataReady & 0x01)) return 0;
    uint16_t distance = i2c_read_reg16(0x0096);
    i2c_write_reg16(0x0086, 0x01);
    return distance;
}

uint16_t vl53l1x_read(void) {
    uint16_t distance;
    int retry;
    for (retry = 0; retry < 5; retry++) {
        distance = vl53l1x_get_distance();
        if (distance > 0) return distance;
        delay_ms(10);
    }
    return 0;
}

/*===========================================================================*/
/* SERVO PWM - TimerA0, UP mode, 50Hz                                      */
/*===========================================================================*/

/* SMCLK=12MHz, /16 -> 750kHz timer clock
 * Period 15000 -> 50Hz (20ms) */
#define SERVO_TIMER_PERIOD  15000

/* Pulse width in ticks: 0deg=750 (1ms), 180deg=1500 (2ms) */
#define SERVO_PULSE_MIN     750
#define SERVO_PULSE_MAX     1500

void servo_init(void) {
    /* P2.6 = TA0.3 (pan), P2.7 = TA0.4 (tilt) */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
        GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    /* TimerA0: UP mode, SMCLK/16, period 15000 */
    Timer_A_UpModeConfig upConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_16,
        SERVO_TIMER_PERIOD,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
        TIMER_A_DO_CLEAR
    };

    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);

    /* CCR3 = pan, CCR4 = tilt */
    Timer_A_CompareModeConfig cmpPan = {
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_RESET_SET,
        SERVO_PULSE_MIN
    };

    Timer_A_CompareModeConfig cmpTilt = {
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_RESET_SET,
        SERVO_PULSE_MIN
    };

    Timer_A_initCompare(TIMER_A0_BASE, &cmpPan);
    Timer_A_initCompare(TIMER_A0_BASE, &cmpTilt);

    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

void servo_set_pulse(uint8_t reg, uint16_t pulse) {
    if (pulse < SERVO_PULSE_MIN) pulse = SERVO_PULSE_MIN;
    if (pulse > SERVO_PULSE_MAX) pulse = SERVO_PULSE_MAX;

    Timer_A_CompareModeConfig cmp = {
        reg,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_RESET_SET,
        pulse
    };
    Timer_A_initCompare(TIMER_A0_BASE, &cmp);
}

/* Convert degrees (0-180) to pulse width ticks */
uint16_t deg_to_pulse(uint8_t deg) {
    if (deg > 180) deg = 180;
    return SERVO_PULSE_MIN + ((uint32_t)deg * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / 180;
}

void servo_set_pan(uint8_t deg) {
    servo_set_pulse(TIMER_A_CAPTURECOMPARE_REGISTER_3, deg_to_pulse(deg));
}

void servo_set_tilt(uint8_t deg) {
    servo_set_pulse(TIMER_A_CAPTURECOMPARE_REGISTER_4, deg_to_pulse(deg));
}

/* Smooth move: step 1 degree at a time with delay */
void servo_smooth_pan(uint8_t from, uint8_t to) {
    int8_t step = (to > from) ? 1 : -1;
    uint8_t pos;
    for (pos = from; pos != to; pos += step) {
        servo_set_pan(pos);
        delay_ms(SERVO_SLEEP_MS);
    }
    servo_set_pan(to);
}

void servo_smooth_tilt(uint8_t from, uint8_t to) {
    int8_t step = (to > from) ? 1 : -1;
    uint8_t pos;
    for (pos = from; pos != to; pos += step) {
        servo_set_tilt(pos);
        delay_ms(SERVO_SLEEP_MS);
    }
    servo_set_tilt(to);
}

/*===========================================================================*/
/* SCAN                                                                      */
/*===========================================================================*/

void announce(const char *msg) {
    send_raw(msg);
    send_raw("\r\n");
}

void do_scan(void) {
    uint16_t dist;
    int tilt, pan;
    int pointCount = 0;

    announce("Rover3D scan start");

    /* Serpentine raster scan: for each tilt step, sweep pan */
    for (tilt = TILT_MIN; tilt <= TILT_MAX; tilt += SCAN_STEP) {
        /* Alternate pan direction each row (serpentine) */
        if (((tilt - TILT_MIN) / SCAN_STEP) % 2 == 0) {
            /* Even rows: pan MAX -> MIN */
            for (pan = PAN_MAX; pan >= PAN_MIN; pan -= SCAN_STEP) {
                servo_set_pan((uint8_t)pan);
                delay_ms(SERVO_SLEEP_MS);
                delay_ms(TOF_SETTLE_MS);

                dist = vl53l1x_read();

                send_line("D,", dist, (dist > 0) ? 1 : 0, 0);
                send_line("S,", (uint16_t)pan, (uint16_t)tilt, dist);
                pointCount++;
            }
        } else {
            /* Odd rows: pan MIN -> MAX */
            for (pan = PAN_MIN; pan <= PAN_MAX; pan += SCAN_STEP) {
                servo_set_pan((uint8_t)pan);
                delay_ms(SERVO_SLEEP_MS);
                delay_ms(TOF_SETTLE_MS);

                dist = vl53l1x_read();

                send_line("D,", dist, (dist > 0) ? 1 : 0, 0);
                send_line("S,", (uint16_t)pan, (uint16_t)tilt, dist);
                pointCount++;
            }
        }

        /* Move to next tilt position */
        if (tilt + SCAN_STEP <= TILT_MAX) {
            servo_smooth_tilt((uint8_t)tilt, (uint8_t)(tilt + SCAN_STEP));
        }
    }

    announce("Rover3D scan done");
}

/*===========================================================================*/
/* MAIN                                                                      */
/*===========================================================================*/
int main(void) {
    WDT_A_holdTimer();
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    led_init();
    led_blink(3);

    usb_init();
    bt_init();
    delay_ms(100);
    i2c_init();
    delay_ms(100);

    /* Init servos (moves to 0deg position on init, then home to tilt min) */
    servo_init();

    /* Home servos to start position */
    servo_smooth_pan(PAN_MAX, PAN_MAX);
    servo_smooth_tilt(TILT_MIN, TILT_MIN);

    /* Init ToF */
    vl53l1x_reset();
    delay_ms(100);

    if (vl53l1x_init()) {
        led_on();
        announce("ToF OK, servos ready");
    } else {
        led_blink(10);
        announce("ToF init FAIL");
        while (1);
    }

    delay_ms(1000);

    while (1) {
        do_scan();

        /* Return to home position */
        servo_smooth_pan(0, PAN_MAX);
        servo_smooth_tilt(TILT_MAX, TILT_MIN);

        announce("Rover3D scan loop restart");
        delay_ms(2000);
    }
}
