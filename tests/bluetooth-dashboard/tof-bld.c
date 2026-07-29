/* =============================================================================
 * ToF Distance Test - Single File
 * Reads VL53L1X and outputs via Bluetooth (UART on P3.2/P3.3)
 *
 * Connections:
 *   VL53L1X: SDA=P1.6, SCL=P1.7, XSHUT=P4.0
 *   HC-05:   RX=P3.3 (MSP TX), TX=P3.2 (MSP RX)
 *   LED:     P1.0 (debug)
 * ============================================================================= */

#include "driverlib.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* -----------------------------------------------------------------------------
 * Pin Definitions
 * ----------------------------------------------------------------------------- */
#define LED_PORT        GPIO_PORT_P1
#define LED_PIN         GPIO_PIN0

#define XSHUT_PORT      GPIO_PORT_P4
#define XSHUT_PIN       GPIO_PIN0

#define I2C_PORT        GPIO_PORT_P1
#define I2C_SDA_PIN     GPIO_PIN6
#define I2C_SCL_PIN     GPIO_PIN7

#define BT_UART_PORT    GPIO_PORT_P3
#define BT_TX_PIN       GPIO_PIN3
#define BT_RX_PIN       GPIO_PIN2

/* -----------------------------------------------------------------------------
 * VL53L1X Definitions
 * ----------------------------------------------------------------------------- */
#define VL53L1X_ADDR                0x29

#define VL53L1X_MODEL_ID            0x010F
#define VL53L1X_MODEL_ID_VALUE      0xEA
#define VL53L1X_FIRMWARE_SYSTEM_STATUS 0x00E5
#define VL53L1X_GPIO_HV_MUX_CTRL    0x0030
#define VL53L1X_GPIO_TIO_HV_STATUS  0x0031
#define VL53L1X_SYSTEM_INTERRUPT_CLEAR 0x0086
#define VL53L1X_SYSTEM_START        0x0087
#define VL53L1X_RESULT_RANGE_STATUS 0x0089
#define VL53L1X_RESULT_DISTANCE     0x0096
#define VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND 0x0008
#define VL53L1X_MYSTERY_REG         0x002D

/* -----------------------------------------------------------------------------
 * Global Variables
 * ----------------------------------------------------------------------------- */
static volatile uint32_t ms_ticks = 0;

/* VL53L1X default configuration (from ST/Adafruit) */
static const uint8_t VL53L1X_DEFAULT_CONFIG[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00
};

/* -----------------------------------------------------------------------------
 * Delay Functions
 * ----------------------------------------------------------------------------- */
void delay_ms(uint32_t ms)
{
    uint32_t i;
    uint32_t j;
    for (i = 0; i < ms; i++)
    {
        for (j = 0; j < 3000; j++)
        {
            __no_operation();
        }
    }
}

void delay_us(uint32_t us)
{
    uint32_t i;
    for (i = 0; i < us * 3; i++)
    {
        __no_operation();
    }
}

/* -----------------------------------------------------------------------------
 * UART Functions (Bluetooth on EUSCI_A2, P3.2/P3.3)
 * ----------------------------------------------------------------------------- */
void uart_init(void)
{
    /* Configure pins P3.2 (RX) and P3.3 (TX) for UART */
    GPIO_setAsPeripheralModuleFunctionInputPin(BT_UART_PORT,
        BT_RX_PIN | BT_TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    /* 115200 baud at 12MHz SMCLK */
    eUSCI_UART_Config uartConfig;
    uartConfig.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    uartConfig.clockPrescalar = 6;
    uartConfig.firstModReg = 8;
    uartConfig.secondModReg = 32;
    uartConfig.parity = EUSCI_A_UART_NO_PARITY;
    uartConfig.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    uartConfig.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    uartConfig.uartMode = EUSCI_A_UART_MODE;
    uartConfig.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    UART_initModule(EUSCI_A2_BASE, &uartConfig);
    UART_enableModule(EUSCI_A2_BASE);
}

void uart_putc(char c)
{
    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG))
    {
        /* Wait for TX buffer empty */
    }
    EUSCI_A2->TXBUF = c;
}

void uart_puts(const char *str)
{
    while (*str)
    {
        uart_putc(*str++);
    }
}

void uart_print_int(int32_t val)
{
    char buf[16];
    int i = 0;
    int neg = 0;
    uint32_t uval;

    if (val < 0)
    {
        neg = 1;
        uval = (uint32_t)(-val);
    }
    else
    {
        uval = (uint32_t)val;
    }

    if (uval == 0)
    {
        uart_putc('0');
        return;
    }

    while (uval > 0)
    {
        buf[i++] = '0' + (uval % 10);
        uval /= 10;
    }

    if (neg)
    {
        uart_putc('-');
    }

    while (i > 0)
    {
        uart_putc(buf[--i]);
    }
}

/* -----------------------------------------------------------------------------
 * I2C Functions (EUSCI_B0, P1.6/P1.7)
 * ----------------------------------------------------------------------------- */
void i2c_init(void)
{
    /* Configure pins P1.6 (SDA) and P1.7 (SCL) */
    GPIO_setAsPeripheralModuleFunctionInputPin(I2C_PORT,
        I2C_SDA_PIN | I2C_SCL_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    /* I2C Master config at 100kHz */
    eUSCI_I2C_MasterConfig i2cConfig;
    i2cConfig.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    i2cConfig.i2cClk = 12000000;
    i2cConfig.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    i2cConfig.byteCounterThreshold = 0;
    i2cConfig.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

    I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    I2C_enableModule(EUSCI_B0_BASE);
}

bool i2c_write_reg16(uint8_t addr, uint16_t reg, uint8_t data)
{
    uint32_t timeout;

    I2C_setSlaveAddress(EUSCI_B0_BASE, addr);

    /* Send START + address + write */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    /* Wait for start to complete */
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;

    /* Check for NACK */
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG)
    {
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        EUSCI_B0->IFG &= ~EUSCI_B_IFG_NACKIFG;
        return false;
    }

    /* Send register address high byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;

    /* Send register address low byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;
    EUSCI_B0->TXBUF = reg & 0xFF;

    /* Send data byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;
    EUSCI_B0->TXBUF = data;

    /* Wait for TX complete */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
    {
        /* Wait */
    }

    /* Send STOP */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout)
    {
        /* Wait */
    }

    return true;
}

bool i2c_write_reg16_multi(uint8_t addr, uint16_t reg, const uint8_t *data, uint16_t len)
{
    uint32_t timeout;
    uint16_t i;

    I2C_setSlaveAddress(EUSCI_B0_BASE, addr);

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;

    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG)
    {
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        EUSCI_B0->IFG &= ~EUSCI_B_IFG_NACKIFG;
        return false;
    }

    /* Send register high byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;

    /* Send register low byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;
    EUSCI_B0->TXBUF = reg & 0xFF;

    /* Send all data bytes */
    for (i = 0; i < len; i++)
    {
        timeout = 10000;
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
        {
            /* Wait */
        }
        if (timeout == 0) return false;
        EUSCI_B0->TXBUF = data[i];
    }

    /* Wait for last byte to finish */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
    {
        /* Wait */
    }

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout)
    {
        /* Wait */
    }

    return true;
}

bool i2c_read_reg16(uint8_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    uint32_t timeout;
    uint16_t i;

    I2C_setSlaveAddress(EUSCI_B0_BASE, addr);

    /* Write register address first */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;

    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG)
    {
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        EUSCI_B0->IFG &= ~EUSCI_B_IFG_NACKIFG;
        return false;
    }

    /* Send register high byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;

    /* Send register low byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;
    EUSCI_B0->TXBUF = reg & 0xFF;

    /* Wait for TX complete */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout)
    {
        /* Wait */
    }

    /* Now switch to read mode with repeated start */
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout)
    {
        /* Wait */
    }
    if (timeout == 0) return false;

    /* Read bytes */
    for (i = 0; i < len; i++)
    {
        /* If last byte, send STOP */
        if (i == len - 1)
        {
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }

        timeout = 10000;
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout)
        {
            /* Wait */
        }
        if (timeout == 0) return false;

        data[i] = EUSCI_B0->RXBUF;
    }

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout)
    {
        /* Wait */
    }

    return true;
}

/* -----------------------------------------------------------------------------
 * VL53L1X Functions
 * ----------------------------------------------------------------------------- */
void vl53l1x_reset(void)
{
    GPIO_setAsOutputPin(XSHUT_PORT, XSHUT_PIN);
    GPIO_setOutputLowOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(50);
    GPIO_setOutputHighOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(50);
}

bool vl53l1x_check_id(void)
{
    uint8_t model_id;

    if (!i2c_read_reg16(VL53L1X_ADDR, VL53L1X_MODEL_ID, &model_id, 1))
    {
        return false;
    }

    return (model_id == VL53L1X_MODEL_ID_VALUE);
}

bool vl53l1x_wait_boot(void)
{
    uint8_t status;
    uint16_t timeout = 1000;

    while (timeout--)
    {
        if (i2c_read_reg16(VL53L1X_ADDR, VL53L1X_FIRMWARE_SYSTEM_STATUS, &status, 1))
        {
            if (status & 0x01)
            {
                return true;
            }
        }
        delay_ms(1);
    }
    return false;
}

bool vl53l1x_init(void)
{
    /* Write default configuration starting at register 0x002D */
    if (!i2c_write_reg16_multi(VL53L1X_ADDR, VL53L1X_MYSTERY_REG,
            VL53L1X_DEFAULT_CONFIG, sizeof(VL53L1X_DEFAULT_CONFIG)))
    {
        return false;
    }

    /* Start ranging */
    if (!i2c_write_reg16(VL53L1X_ADDR, VL53L1X_SYSTEM_START, 0x40))
    {
        return false;
    }

    /* Wait a bit for first measurement */
    delay_ms(100);

    return true;
}

bool vl53l1x_data_ready(void)
{
    uint8_t status;
    uint8_t polarity;

    /* Get interrupt polarity */
    if (!i2c_read_reg16(VL53L1X_ADDR, VL53L1X_GPIO_HV_MUX_CTRL, &polarity, 1))
    {
        return false;
    }
    polarity = (polarity & 0x10) >> 4;

    /* Check data ready */
    if (!i2c_read_reg16(VL53L1X_ADDR, VL53L1X_GPIO_TIO_HV_STATUS, &status, 1))
    {
        return false;
    }

    return ((status & 0x01) == polarity);
}

bool vl53l1x_read_distance(uint16_t *distance_mm)
{
    uint8_t data[2];

    if (!i2c_read_reg16(VL53L1X_ADDR, VL53L1X_RESULT_DISTANCE, data, 2))
    {
        return false;
    }

    *distance_mm = ((uint16_t)data[0] << 8) | data[1];
    return true;
}

void vl53l1x_clear_interrupt(void)
{
    i2c_write_reg16(VL53L1X_ADDR, VL53L1X_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

/* -----------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------- */
int main(void)
{
    uint16_t distance;
    bool sensor_ok;

    /* Stop watchdog */
    WDT_A_holdTimer();

    /* Set DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* Setup LED for debug */
    GPIO_setAsOutputPin(LED_PORT, LED_PIN);
    GPIO_setOutputLowOnPin(LED_PORT, LED_PIN);

    /* Initialize UART for Bluetooth */
    uart_init();

    /* Initialize I2C */
    i2c_init();

    /* Blink LED to show code started */
    GPIO_setOutputHighOnPin(LED_PORT, LED_PIN);
    delay_ms(200);
    GPIO_setOutputLowOnPin(LED_PORT, LED_PIN);
    delay_ms(200);

    uart_puts("S:Rover ToF Test Starting\n");

    /* Reset and initialize VL53L1X */
    uart_puts("S:Resetting VL53L1X...\n");
    vl53l1x_reset();

    uart_puts("S:Checking sensor ID...\n");
    if (!vl53l1x_check_id())
    {
        uart_puts("S:ERROR - VL53L1X not found!\n");
        while (1)
        {
            GPIO_toggleOutputOnPin(LED_PORT, LED_PIN);
            delay_ms(100);
        }
    }
    uart_puts("S:VL53L1X found (ID=0xEA)\n");

    uart_puts("S:Waiting for boot...\n");
    if (!vl53l1x_wait_boot())
    {
        uart_puts("S:ERROR - Boot timeout!\n");
        while (1)
        {
            GPIO_toggleOutputOnPin(LED_PORT, LED_PIN);
            delay_ms(200);
        }
    }
    uart_puts("S:Boot OK\n");

    uart_puts("S:Initializing sensor...\n");
    if (!vl53l1x_init())
    {
        uart_puts("S:ERROR - Init failed!\n");
        while (1)
        {
            GPIO_toggleOutputOnPin(LED_PORT, LED_PIN);
            delay_ms(300);
        }
    }
    uart_puts("S:Init OK - Starting measurements\n");

    sensor_ok = true;

    /* Main loop - read and output distance */
    while (1)
    {
        /* Wait for data ready */
        if (vl53l1x_data_ready())
        {
            if (vl53l1x_read_distance(&distance))
            {
                /* Output in dashboard format */
                uart_puts("D:");
                uart_print_int(distance);
                uart_puts("\n");

                /* Toggle LED on each reading */
                GPIO_toggleOutputOnPin(LED_PORT, LED_PIN);
            }
            else
            {
                uart_puts("S:Read error\n");
            }

            /* Clear interrupt for next reading */
            vl53l1x_clear_interrupt();
        }

        delay_ms(50);
    }
}
