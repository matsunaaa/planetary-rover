/*******************************************************************************
 * MPU6050 + ToF Integration
 * Single file for CCS - 12MHz SMCLK, C89, DriverLib + struct registers
 ******************************************************************************/

#include "driverlib.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Pin Definitions                                                            */
/* -------------------------------------------------------------------------- */
#define LED_RED_PORT        GPIO_PORT_P1
#define LED_RED_PIN         GPIO_PIN0

#define XSHUT_PORT          GPIO_PORT_P4
#define XSHUT_PIN           GPIO_PIN0

/* -------------------------------------------------------------------------- */
/* I2C Addresses                                                              */
/* -------------------------------------------------------------------------- */
#define TOF_ADDR            0x29
#define MPU6050_ADDR        0x68

/* -------------------------------------------------------------------------- */
/* MPU6050 Registers                                                          */
/* -------------------------------------------------------------------------- */
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_GYRO_CONFIG 0x1B

/* Gyro sensitivity: 131 LSB per deg/s at +/-250 deg/s range */
#define GYRO_SENSITIVITY    131.0f

/* -------------------------------------------------------------------------- */
/* Global Variables                                                           */
/* -------------------------------------------------------------------------- */
volatile float heading_deg = 0.0f;
volatile int16_t gyro_z_offset = 0;

/* -------------------------------------------------------------------------- */
/* Delay Function (busy loop, no SysTick)                                     */
/* -------------------------------------------------------------------------- */
void delay_ms(uint32_t ms)
{
    uint32_t i, j;
    for (i = 0; i < ms; i++)
    {
        for (j = 0; j < 3000; j++)
        {
            __no_operation();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* UART Functions (from your working code)                                    */
/* -------------------------------------------------------------------------- */
void uart_init(void)
{
    eUSCI_UART_Config uart;

    uart.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    uart.clockPrescalar = 6;
    uart.firstModReg = 8;
    uart.secondModReg = 32;
    uart.parity = EUSCI_A_UART_NO_PARITY;
    uart.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    uart.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    uart.uartMode = EUSCI_A_UART_MODE;
    uart.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    UART_initModule(EUSCI_A0_BASE, &uart);
    UART_enableModule(EUSCI_A0_BASE);
}

void uart_print(const char *str)
{
    uint32_t i;
    for (i = 0; str[i] != '\0'; i++)
    {
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A0->TXBUF = str[i];
    }
}

void uart_print_int(int32_t val)
{
    char buf[16];
    sprintf(buf, "%ld", (long)val);
    uart_print(buf);
}

void uart_print_float(float val, uint8_t decimals)
{
    char buf[32];
    int32_t int_part;
    int32_t frac_part;
    float frac_mult;
    uint8_t i;

    if (val < 0)
    {
        uart_print("-");
        val = -val;
    }

    int_part = (int32_t)val;
    frac_mult = 1.0f;
    for (i = 0; i < decimals; i++)
    {
        frac_mult *= 10.0f;
    }
    frac_part = (int32_t)((val - int_part) * frac_mult);

    sprintf(buf, "%ld.%0*ld", (long)int_part, decimals, (long)frac_part);
    uart_print(buf);
}

/* -------------------------------------------------------------------------- */
/* I2C Functions (struct-based, shared bus)                                   */
/* -------------------------------------------------------------------------- */
void i2c_init(void)
{
    eUSCI_I2C_MasterConfig i2c;

    i2c.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    i2c.i2cClk = 12000000;
    i2c.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    i2c.byteCounterThreshold = 0;
    i2c.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    I2C_initMaster(EUSCI_B0_BASE, &i2c);
    I2C_enableModule(EUSCI_B0_BASE);
}

uint8_t i2c_write_byte(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint32_t timeout;

    /* Set slave address */
    EUSCI_B0->I2CSA = dev_addr;

    /* Wait for bus ready */
    timeout = 10000;
    while ((EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY) && --timeout);
    if (timeout == 0) return 1;

    /* Start + TX mode */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    /* Wait for TX ready, send register address */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 2;
    EUSCI_B0->TXBUF = reg;

    /* Wait for TX ready, send data */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 3;
    EUSCI_B0->TXBUF = data;

    /* Wait for TX complete */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 4;

    /* Send stop */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    /* Wait for stop complete */
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);

    return 0;
}

uint8_t i2c_read_byte(uint8_t dev_addr, uint8_t reg, uint8_t *data)
{
    uint32_t timeout;

    /* Set slave address */
    EUSCI_B0->I2CSA = dev_addr;

    /* Wait for bus ready */
    timeout = 10000;
    while ((EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY) && --timeout);
    if (timeout == 0) return 1;

    /* Start + TX mode to send register address */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    /* Wait for TX ready */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 2;

    /* Send register address */
    EUSCI_B0->TXBUF = reg;

    /* Wait for TX complete */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 3;

    /* Repeated start + RX mode */
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    /* Wait for start sent */
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);
    if (timeout == 0) return 4;

    /* Send stop (single byte read) */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    /* Wait for RX data */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);
    if (timeout == 0) return 5;

    /* Read data */
    *data = EUSCI_B0->RXBUF;

    /* Wait for stop complete */
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);

    return 0;
}

uint8_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint32_t timeout;
    uint8_t i;

    /* Set slave address */
    EUSCI_B0->I2CSA = dev_addr;

    /* Wait for bus ready */
    timeout = 10000;
    while ((EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY) && --timeout);
    if (timeout == 0) return 1;

    /* Start + TX mode to send register address */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    /* Wait for TX ready */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 2;

    /* Send register address */
    EUSCI_B0->TXBUF = reg;

    /* Wait for TX complete */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 3;

    /* Repeated start + RX mode */
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    /* Wait for start sent */
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);
    if (timeout == 0) return 4;

    /* Read bytes */
    for (i = 0; i < len; i++)
    {
        /* If last byte, send stop */
        if (i == len - 1)
        {
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }

        /* Wait for RX data */
        timeout = 10000;
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);
        if (timeout == 0) return 5;

        buf[i] = EUSCI_B0->RXBUF;
    }

    /* Wait for stop complete */
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);

    return 0;
}

/* -------------------------------------------------------------------------- */
/* MPU6050 Functions                                                          */
/* -------------------------------------------------------------------------- */
uint8_t mpu6050_init(void)
{
    uint8_t who_am_i;
    uint8_t pwr_mgmt;
    uint8_t err;

    /* Read WHO_AM_I */
    err = i2c_read_byte(MPU6050_ADDR, MPU6050_WHO_AM_I, &who_am_i);
    uart_print("WHO_AM_I: ");
    uart_print_int(who_am_i);
    uart_print(" err=");
    uart_print_int(err);
    uart_print("\r\n");

    /* Read current power state */
    err = i2c_read_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, &pwr_mgmt);
    uart_print("PWR_MGMT_1 before: ");
    uart_print_int(pwr_mgmt);
    uart_print(" err=");
    uart_print_int(err);
    uart_print("\r\n");

    /* Reset device first */
    uart_print("Resetting device...\r\n");
    err = i2c_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x80);
    if (err != 0)
    {
        uart_print("Reset write failed\r\n");
        return 1;
    }
    delay_ms(100);

    /* Wake up - clear sleep bit */
    uart_print("Waking up...\r\n");
    err = i2c_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    if (err != 0)
    {
        uart_print("Wake write failed\r\n");
        return 2;
    }
    delay_ms(50);

    /* Verify wake */
    err = i2c_read_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, &pwr_mgmt);
    uart_print("PWR_MGMT_1 after: ");
    uart_print_int(pwr_mgmt);
    uart_print(" err=");
    uart_print_int(err);
    uart_print("\r\n");

    /* Should be 0x00 now (not 0x40 which means sleep) */
    if (pwr_mgmt & 0x40)
    {
        uart_print("ERROR: Still in sleep mode!\r\n");
        return 3;
    }

    /* Gyro config */
    err = i2c_write_byte(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);
    delay_ms(10);

    uart_print("IMU ready\r\n");
    return 0;
}

int16_t mpu6050_read_gyro_z(void)
{
    uint8_t buf[2];
    uint8_t err;
    int16_t raw;

    err = i2c_read_bytes(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, buf, 2);
    if (err != 0)
    {
        return 0;
    }

    raw = (int16_t)((buf[0] << 8) | buf[1]);
    return raw;
}

void mpu6050_calibrate(uint16_t samples)
{
    int32_t sum = 0;
    uint16_t i;

    uart_print("Calibrating gyro (keep still)...\r\n");

    for (i = 0; i < samples; i++)
    {
        sum += mpu6050_read_gyro_z();
        delay_ms(2);
    }

    gyro_z_offset = (int16_t)(sum / samples);

    uart_print("Gyro Z offset: ");
    uart_print_int(gyro_z_offset);
    uart_print("\r\n");
}

void mpu6050_update_heading(float dt_seconds)
{
    int16_t raw;
    float gyro_dps;

    raw = mpu6050_read_gyro_z() - gyro_z_offset;

    /* Convert to degrees per second */
    gyro_dps = (float)raw / GYRO_SENSITIVITY;

    /* Integrate to get heading */
    heading_deg += gyro_dps * dt_seconds;

    /* Wrap to 0-360 */
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;
    while (heading_deg < 0.0f) heading_deg += 360.0f;
}

/* -------------------------------------------------------------------------- */
/* ToF Functions (keep your existing code, abbreviated here)                  */
/* -------------------------------------------------------------------------- */
void tof_xshut_reset(void)
{
    GPIO_setAsOutputPin(XSHUT_PORT, XSHUT_PIN);
    GPIO_setOutputLowOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(50);
    GPIO_setOutputHighOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(50);
}

/* Add your existing ToF read functions here */
/* ... */

/* -------------------------------------------------------------------------- */
/* Test Main - MPU6050 Only                                                   */
/* -------------------------------------------------------------------------- */
int main(void)
{
    uint8_t err;
    uint32_t loop_count = 0;

    /* Stop watchdog */
    WDT_A_holdTimer();

    /* Set DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* Init LED */
    GPIO_setAsOutputPin(LED_RED_PORT, LED_RED_PIN);
    GPIO_setOutputLowOnPin(LED_RED_PORT, LED_RED_PIN);

    /* Init UART */
    uart_init();
    uart_print("\r\n=== MPU6050 Test ===\r\n");

    /* Init I2C */
    i2c_init();

    /* Init MPU6050 */
    err = mpu6050_init();
    if (err != 0)
    {
        uart_print("MPU6050 init failed!\r\n");
        while(1)
        {
            GPIO_toggleOutputOnPin(LED_RED_PORT, LED_RED_PIN);
            delay_ms(100);
        }
    }

    /* Calibrate gyro (500 samples @ 2ms = 1 second) */
    mpu6050_calibrate(500);

    uart_print("Rotate the robot...\r\n");

    /* Add this test in main, after init and calibration */
    uart_print("Raw gyro test (rotate board!):\r\n");
    {
        int16_t raw;
        uint8_t buf[2];
        uint8_t err;
        int i;

        for (i = 0; i < 20; i++)
        {
            err = i2c_read_bytes(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, buf, 2);

            uart_print("err=");
            uart_print_int(err);
            uart_print(" buf[0]=");
            uart_print_int(buf[0]);
            uart_print(" buf[1]=");
            uart_print_int(buf[1]);

            raw = (int16_t)((buf[0] << 8) | buf[1]);
            uart_print(" raw=");
            uart_print_int(raw);
            uart_print("\r\n");

            delay_ms(200);
        }
    }

    /* Main loop - read gyro at ~50Hz */
    while(1)
    {
        /* Update heading (dt = 20ms = 0.02s) */
        mpu6050_update_heading(0.02f);

        /* Print every 10 loops (5Hz) */
        if (loop_count % 10 == 0)
        {
            uart_print("Heading: ");
            uart_print_float(heading_deg, 1);
            uart_print(" deg\r\n");

            GPIO_toggleOutputOnPin(LED_RED_PORT, LED_RED_PIN);
        }

        loop_count++;
        delay_ms(20);
    }
}
