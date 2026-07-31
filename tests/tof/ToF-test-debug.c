/* VL53L1X ToF Test - MSP432
 * Uses 12MHz DCO, XSHUT reset, Adafruit init sequence
 */

#include "driverlib.h"
#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* PINS                                                                      */
/*===========================================================================*/

#define XSHUT_PORT  GPIO_PORT_P4
#define XSHUT_PIN   GPIO_PIN0

#define VL53L1X_ADDR 0x29

/*===========================================================================*/
/* Adafruit init sequence                                                    */
/*===========================================================================*/

static const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
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
/* UART CONFIG (Working from debug log)                                      */
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
/* I2C CONFIG (Working from debug log)                                       */
/*===========================================================================*/

const eUSCI_I2C_MasterConfig i2cConfig = {
    EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
    12000000,
    EUSCI_B_I2C_SET_DATA_RATE_100KBPS,
    0,
    EUSCI_B_I2C_NO_AUTO_STOP
};

/*===========================================================================*/
/* DELAY (No SysTick)                                                         */
/*===========================================================================*/

void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 3000; j++)
            __no_operation();
}

/*===========================================================================*/
/* UART                                                                      */
/*===========================================================================*/

void UART_Init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    UART_enableModule(EUSCI_A0_BASE);
}

void UART_Print(const char *str) {
    while (*str) {
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A0->TXBUF = *str++;
    }
}

void UART_PrintNum(uint16_t num) {
    char buf[6];
    int i = 0;

    if (num == 0) {
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A0->TXBUF = '0';
        return;
    }

    while (num > 0) {
        buf[i++] = '0' + (num % 10);
        num /= 10;
    }

    while (i > 0) {
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A0->TXBUF = buf[--i];
    }
}

void UART_PrintHex(uint8_t val) {
    const char hex[] = "0123456789ABCDEF";
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = hex[(val >> 4) & 0x0F];
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = hex[val & 0x0F];
}

/*===========================================================================*/
/* LED                                                                       */
/*===========================================================================*/

void LED_Init(void) {
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void LED_On(void) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void LED_Off(void) {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void LED_Blink(int times) {
    int i;
    for (i = 0; i < times; i++) {
        LED_On();
        delay_ms(100);
        LED_Off();
        delay_ms(100);
    }
}

/*===========================================================================*/
/* I2C (Direct register, more reliable)                                      */
/*===========================================================================*/

void I2C_Init(void) {
    /* P1.6 SDA, P1.7 SCL */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    I2C_enableModule(EUSCI_B0_BASE);
}

bool I2C_WriteReg8(uint16_t reg, uint8_t data) {
    uint32_t timeout;

    /* Set slave address */
    EUSCI_B0->I2CSA = VL53L1X_ADDR;

    /* Wait for bus free */
    timeout = 10000;
    while ((EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY) && --timeout);
    if (timeout == 0) return false;

    /* TX mode, START */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    /* Wait START done */
    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);
    if (timeout == 0) return false;

    /* Check NACK */
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG) {
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        return false;
    }

    /* Send reg high byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;

    /* Send reg low byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = reg & 0xFF;

    /* Send data */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = data;

    /* Wait TX done */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);

    { volatile uint32_t d = 0; for (d = 0; d < 500; d++) __no_operation(); }

    /* STOP */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);

    return true;
}

uint8_t I2C_ReadReg8(uint16_t reg) {
    uint32_t timeout;
    uint8_t data = 0;

    EUSCI_B0->I2CSA = VL53L1X_ADDR;

    /* Wait bus free */
    timeout = 10000;
    while ((EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY) && --timeout);
    if (timeout == 0) return 0;

    /* TX mode, START */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);
    if (timeout == 0) return 0;

    /* Send reg address */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = reg & 0xFF;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);

    /* Small delay to let last byte finish shifting (~100us at 100kHz) */
    { volatile uint32_t d = 0; for (d = 0; d < 500; d++) __no_operation(); }

    /* Repeated START, RX mode */
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);

    /* STOP (for single byte read) */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    /* Wait for RX */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);

    data = EUSCI_B0->RXBUF;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);

    return data;
}

uint16_t I2C_ReadReg16(uint16_t reg) {
    uint32_t timeout;
    uint8_t hi = 0, lo = 0;

    EUSCI_B0->I2CSA = VL53L1X_ADDR;

    timeout = 10000;
    while ((EUSCI_B0->STATW & EUSCI_B_STATW_BBUSY) && --timeout);
    if (timeout == 0) return 0;

    /* TX mode, START */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);

    /* Send reg address */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    EUSCI_B0->TXBUF = reg & 0xFF;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);

    { volatile uint32_t d = 0; for (d = 0; d < 500; d++) __no_operation(); }

    /* Repeated START, RX mode */
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);

    /* Read first byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);
    hi = EUSCI_B0->RXBUF;

    /* STOP before last byte */
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    /* Read second byte */
    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);
    lo = EUSCI_B0->RXBUF;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);

    return ((uint16_t)hi << 8) | lo;
}

/*===========================================================================*/
/* XSHUT RESET (Required per debug log)                                       */
/*===========================================================================*/

void VL53L1X_Reset(void) {
    GPIO_setAsOutputPin(XSHUT_PORT, XSHUT_PIN);
    GPIO_setOutputLowOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(50);
    GPIO_setOutputHighOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(50);
}

/*===========================================================================*/
/* VL53L1X                                                                   */
/*===========================================================================*/

bool VL53L1X_Init(void) {
    uint8_t modelId;
    uint8_t bootState;
    int i;

    /* Check model ID (should be 0xEA) */
    modelId = I2C_ReadReg8(0x010F);

    UART_Print("Model ID: 0x");
    UART_PrintHex(modelId);
    UART_Print("\r\n");

    if (modelId != 0xEA) {
        UART_Print("Wrong ID!\r\n");
        return false;
    }

    /* Wait for boot (0x00E5 should become 0x03) */
    UART_Print("Waiting boot...\r\n");
    for (i = 0; i < 100; i++) {
        bootState = I2C_ReadReg8(0x00E5);
        if (bootState == 0x03) break;
        delay_ms(10);
    }

    if (bootState != 0x03) {
        UART_Print("Boot fail: 0x");
        UART_PrintHex(bootState);
        UART_Print("\r\n");
        return false;
    }

    UART_Print("Boot OK\r\n");

    /* Write Adafruit init sequence starting at 0x002D */
    UART_Print("Init seq...\r\n");
    for (i = 0; i < sizeof(VL51L1X_DEFAULT_CONFIGURATION); i++) {
        if (!I2C_WriteReg8(0x002D + i, VL51L1X_DEFAULT_CONFIGURATION[i])) {
            UART_Print("Init write fail at ");
            UART_PrintNum(i);
            UART_Print("\r\n");
            return false;
        }
    }

    UART_Print("Init done\r\n");

    /* Start continuous ranging */
    I2C_WriteReg8(0x0086, 0x01);  /* Clear interrupt */
    I2C_WriteReg8(0x0087, 0x40);  /* Start ranging */

    delay_ms(50);

    return true;
}

uint16_t VL53L1X_GetDistance(void) {
    uint8_t dataReady;
    uint16_t distance;

    /* Check data ready (bit 0 of 0x0031) */
    dataReady = I2C_ReadReg8(0x0031);

    if (!(dataReady & 0x01)) {
        return 0;  /* Not ready yet */
    }

    /* Read distance from 0x0096 */
    distance = I2C_ReadReg16(0x0096);

    /* Clear interrupt */
    I2C_WriteReg8(0x0086, 0x01);

    return distance;
}

/*===========================================================================*/
/* MAIN                                                                      */
/*===========================================================================*/

int main(void) {
    uint16_t distance = 0;
    uint16_t status = 0;
    int retry = 0;
    int found = 0;
    uint8_t addr;

    /* Stop watchdog */
    WDT_A_holdTimer();

    /* 12MHz DCO */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* Init LED */
    LED_Init();
    LED_Blink(3);

    /* Init UART & I2C */
    UART_Init();
    delay_ms(100);

    UART_Print("\r\n=== VL53L1X Debug Test ===\r\n");
    UART_Print("12MHz DCO OK\r\n");

    /* Test XSHUT pin */
    UART_Print("Testing XSHUT (P4.0)...\r\n");
    GPIO_setAsOutputPin(XSHUT_PORT, XSHUT_PIN);
    GPIO_setOutputLowOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(10);
    UART_Print("XSHUT LOW\r\n");
    GPIO_setOutputHighOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(10);
    UART_Print("XSHUT HIGH\r\n");

    I2C_Init();
    delay_ms(100);
    UART_Print("I2C init OK\r\n");

    /* Probe I2C address 0x29 */
    UART_Print("Probing 0x29...\r\n");
    EUSCI_B0->I2CSA = VL53L1X_ADDR;
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_NACKIFG;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;
    {
        uint32_t tout = 20000;
        while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --tout);
        if (tout == 0) {
            UART_Print("TXSTT timeout!\r\n");
        }
    }
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG) {
        UART_Print("NACK - no device at 0x29\r\n");
        EUSCI_B0->IFG &= ~EUSCI_B_IFG_NACKIFG;
    } else {
        UART_Print("ACK - device at 0x29\r\n");
    }
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    {
        uint32_t tout = 20000;
        while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --tout);
    }

    /* Reset & Init sensor */
    VL53L1X_Reset();
    delay_ms(100);

    if (VL53L1X_Init()) {
        LED_On();
        UART_Print("Init SUCCESS\r\n");
    } else {
        LED_Blink(10);
        while (1);
    }

    delay_ms(500);

    /* Main loop */
    while (1) {
        distance = 0;

        /* Poll up to 5 times (50ms total) waiting for data ready bit */
        for (retry = 0; retry < 5; retry++) {
            distance = VL53L1X_GetDistance();
            if (distance > 0) break;
            delay_ms(10);
        }

        if (distance > 0) {
            status = 1;  /* Ready / Valid measurement */
        } else {
            status = 0;  /* Not ready */
        }

        /* CSV Output format: D,dist,status\r\n */
        UART_Print("D,");
        UART_PrintNum(distance);
        UART_Print(",");
        UART_PrintNum(status);
        UART_Print("\r\n");

        delay_ms(50);
    }
}
