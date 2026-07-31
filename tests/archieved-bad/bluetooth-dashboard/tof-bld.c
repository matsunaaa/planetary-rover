/* =============================================================================
 * ToF Distance Test - Single File (USB + Bluetooth/ESP32 output)
 * Reads VL53L1X and outputs via USB UART (P1.2/P1.3) and BT UART (P3.2/P3.3)
 *
 * Connections:
 *   VL53L1X:   SDA=P1.6, SCL=P1.7, XSHUT=P4.0
 *   USB UART:  P1.2 (RX), P1.3 (TX) - XDS110 debug serial (COM6)
 *   BT UART:   P3.2 (RX), P3.3 (TX) - to ESP32 GPIO16
 *   LED:       P1.0 (debug)
 *
 * Output format: D,dist,status\r\n  (same as working ToF-test.c)
 *                P,idx,dist,0\r\n    (point cloud for dashboard)
 * ============================================================================= */

#include "driverlib.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
 * Pin Definitions
 * --------------------------------------------------------------------------- */
#define LED_PORT        GPIO_PORT_P1
#define LED_PIN         GPIO_PIN0

#define XSHUT_PORT      GPIO_PORT_P4
#define XSHUT_PIN       GPIO_PIN0

#define I2C_PORT        GPIO_PORT_P1
#define I2C_SDA_PIN     GPIO_PIN6
#define I2C_SCL_PIN     GPIO_PIN7

#define BT_PORT         GPIO_PORT_P3
#define BT_TX_PIN       GPIO_PIN3
#define BT_RX_PIN       GPIO_PIN2

/* ---------------------------------------------------------------------------
 * VL53L1X Address & Registers
 * --------------------------------------------------------------------------- */
#define VL53L1X_ADDR                    0x29
#define VL53L1X_MODEL_ID                0x010F
#define VL53L1X_SYSTEM_STATUS           0x00E5
#define VL53L1X_GPIO_TIO_HV_STATUS      0x0031
#define VL53L1X_SYSTEM_INTERRUPT_CLEAR  0x0086
#define VL53L1X_SYSTEM_START            0x0087
#define VL53L1X_RESULT_DISTANCE         0x0096
#define VL53L1X_CONFIG_START            0x002D

/* Adafruit VL53L1X init sequence (written starting at 0x002D) */
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

/* ---------------------------------------------------------------------------
 * Delay (no SysTick)
 * --------------------------------------------------------------------------- */
void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 3000; j++)
            __no_operation();
}

/* ---------------------------------------------------------------------------
 * USB UART (EUSCI_A0, P1.2/P1.3) - 115200 baud
 * --------------------------------------------------------------------------- */
void usb_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    eUSCI_UART_Config cfg = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        6, 8, 32,
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };
    UART_initModule(EUSCI_A0_BASE, &cfg);
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

/* ---------------------------------------------------------------------------
 * BT UART (EUSCI_A2, P3.2/P3.3) - 115200 baud to ESP32
 * --------------------------------------------------------------------------- */
void bt_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(BT_PORT,
        BT_RX_PIN | BT_TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    eUSCI_UART_Config cfg = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        6, 8, 32,
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };
    UART_initModule(EUSCI_A2_BASE, &cfg);
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

/* ---------------------------------------------------------------------------
 * I2C (EUSCI_B0, P1.6/P1.7) - direct register access
 * --------------------------------------------------------------------------- */
void i2c_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    eUSCI_I2C_MasterConfig i2cConfig = {
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
        12000000,
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,
        0,
        EUSCI_B_I2C_NO_AUTO_STOP
    };
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

/* ---------------------------------------------------------------------------
 * VL53L1X
 * --------------------------------------------------------------------------- */
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
    uint16_t distance;

    dataReady = i2c_read_reg8(0x0031);
    if (!(dataReady & 0x01)) return 0;

    distance = i2c_read_reg16(0x0096);
    i2c_write_reg16(0x0086, 0x01);
    return distance;
}

/* ---------------------------------------------------------------------------
 * LED
 * --------------------------------------------------------------------------- */
void led_init(void) {
    GPIO_setAsOutputPin(LED_PORT, LED_PIN);
    GPIO_setOutputLowOnPin(LED_PORT, LED_PIN);
}

void led_on(void) {
    GPIO_setOutputHighOnPin(LED_PORT, LED_PIN);
}

void led_off(void) {
    GPIO_setOutputLowOnPin(LED_PORT, LED_PIN);
}

void led_blink(int times) {
    int i;
    for (i = 0; i < times; i++) {
        led_on(); delay_ms(100);
        led_off(); delay_ms(100);
    }
}

/* ---------------------------------------------------------------------------
 * Main
 * --------------------------------------------------------------------------- */
int main(void) {
    uint16_t distance;
    uint16_t status;
    int retry;
    int point_idx = 0;

    WDT_A_holdTimer();
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    led_init();
    led_blink(3);

    usb_init();
    bt_init();
    delay_ms(100);
    i2c_init();
    delay_ms(100);

    vl53l1x_reset();
    delay_ms(100);

    if (vl53l1x_init()) {
        led_on();
    } else {
        led_blink(10);
        while (1);
    }

    delay_ms(500);

    while (1) {
        distance = 0;
        for (retry = 0; retry < 5; retry++) {
            distance = vl53l1x_get_distance();
            if (distance > 0) break;
            delay_ms(10);
        }

        status = (distance > 0) ? 1 : 0;

        usb_puts("D,");
        usb_print_int(distance);
        usb_puts(",");
        usb_print_int(status);
        usb_puts("\r\n");

        bt_puts("D,");
        bt_print_int(distance);
        bt_puts(",");
        bt_print_int(status);
        bt_puts("\r\n");

        usb_puts("P,");
        usb_print_int(point_idx);
        usb_puts(",");
        usb_print_int(distance);
        usb_puts(",0\r\n");

        bt_puts("P,");
        bt_print_int(point_idx);
        bt_puts(",");
        bt_print_int(distance);
        bt_puts(",0\r\n");
        point_idx++;

        delay_ms(50);
    }
}