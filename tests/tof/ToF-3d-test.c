#include "msp.h"
#include "driverlib.h"
#include <stdint.h>
#include <stdbool.h>

#define VL53L1X_ADDR 0x29

#define XSHUT_PORT GPIO_PORT_P4
#define XSHUT_PIN GPIO_PIN0

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

void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 3000; j++)
            __no_operation();
}

void UART_Init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    const eUSCI_UART_Config uartConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        6, 8, 32,
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };

    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    UART_enableModule(EUSCI_A0_BASE);
}

void UART_Tx(char c) {
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = c;
}

void UART_Print(const char *s) {
    while (*s) UART_Tx(*s++);
}

void UART_PrintNum(uint16_t num) {
    char buf[6];
    int i = 0;
    if (num == 0) { UART_Tx('0'); return; }
    while (num > 0) { buf[i++] = '0' + (num % 10); num /= 10; }
    while (i > 0) UART_Tx(buf[--i]);
}

uint8_t UART_RxReady(void) {
    return (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG) ? 1 : 0;
}

char UART_Rx(void) {
    return EUSCI_A0->RXBUF;
}

void I2C_Init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    const eUSCI_I2C_MasterConfig i2cConfig = {
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
        12000000,
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,
        0,
        EUSCI_B_I2C_NO_AUTO_STOP
    };

    I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    I2C_enableModule(EUSCI_B0_BASE);
}

bool I2C_WriteReg8(uint16_t reg, uint8_t data) {
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
    if (timeout == 0) return false;
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return false;
    EUSCI_B0->TXBUF = reg & 0xFF;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return false;
    EUSCI_B0->TXBUF = data;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);

    return true;
}

uint8_t I2C_ReadReg8(uint16_t reg) {
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
    if (timeout == 0) return 0;
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 0;
    EUSCI_B0->TXBUF = reg & 0xFF;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 0;

    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);
    if (timeout == 0) return 0;

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

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 0;
    EUSCI_B0->TXBUF = (reg >> 8) & 0xFF;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 0;
    EUSCI_B0->TXBUF = reg & 0xFF;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) && --timeout);
    if (timeout == 0) return 0;

    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --timeout);

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);
    if (timeout == 0) return 0;
    hi = EUSCI_B0->RXBUF;

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    timeout = 10000;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) && --timeout);
    if (timeout == 0) return 0;
    lo = EUSCI_B0->RXBUF;

    timeout = 10000;
    while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP) && --timeout);

    return ((uint16_t)hi << 8) | lo;
}

void ToF_Reset(void) {
    GPIO_setAsOutputPin(XSHUT_PORT, XSHUT_PIN);
    GPIO_setOutputLowOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(50);
    GPIO_setOutputHighOnPin(XSHUT_PORT, XSHUT_PIN);
    delay_ms(50);
}

int ToF_Init(void) {
    uint8_t modelId, bootState;
    int i;

    modelId = I2C_ReadReg8(0x010F);
    if (modelId != 0xEA) {
        UART_Print("ID FAIL\r\n");
        return -1;
    }

    for (i = 0; i < 100; i++) {
        bootState = I2C_ReadReg8(0x00E5);
        if (bootState == 0x03) break;
        delay_ms(10);
    }
    if (bootState != 0x03) {
        UART_Print("BOOT FAIL\r\n");
        return -2;
    }

    for (i = 0; i < sizeof(VL51L1X_DEFAULT_CONFIGURATION); i++) {
        if (!I2C_WriteReg8(0x002D + i, VL51L1X_DEFAULT_CONFIGURATION[i])) {
            UART_Print("INIT FAIL\r\n");
            return -3;
        }
    }

    I2C_WriteReg8(0x0086, 0x01);
    I2C_WriteReg8(0x0087, 0x40);

    delay_ms(50);
    return 0;
}

uint16_t ToF_Read(uint8_t *status) {
    uint8_t dataReady;

    dataReady = I2C_ReadReg8(0x0031);
    if (!(dataReady & 0x01)) {
        *status = 0xFF;
        return 0;
    }

    *status = I2C_ReadReg8(0x0089) & 0x1F;
    uint16_t distance = I2C_ReadReg16(0x0096);
    I2C_WriteReg8(0x0086, 0x01);

    return distance;
}

int main(void) {
    uint16_t distance;
    uint8_t status;
    uint32_t point_id = 0;
    uint8_t capturing = 0;

    WDT_A_holdTimer();
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

    UART_Init();
    delay_ms(100);
    UART_Print("RDY\r\n");

    I2C_Init();
    delay_ms(100);

    ToF_Reset();
    delay_ms(100);

    if (ToF_Init() == 0) {
        UART_Print("TOF OK\r\n");
    } else {
        UART_Print("FAIL\r\n");
        while (1) {
            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
            delay_ms(200);
        }
    }

    while (1) {
        if (UART_RxReady()) {
            char c = UART_Rx();
            if (c == 's' || c == 'S') {
                capturing = 1;
                point_id = 0;
                UART_Print("!START\r\n");
            } else if (c == 'x' || c == 'X') {
                capturing = 0;
                UART_Print("!STOP\r\n");
            } else if (c == 'c' || c == 'C') {
                point_id = 0;
                UART_Print("!CLEAR\r\n");
            } else if (c == '?') {
                UART_Print("!PING\r\n");
            }
        }

        distance = ToF_Read(&status);

        if (capturing) {
            UART_Print("D,");
            UART_PrintNum(distance);
            UART_Print(",");
            UART_PrintNum(status);
            UART_Print("\r\n");

            if (status == 0 && distance > 20 && distance < 1300) {
                UART_Print("P,");
                UART_PrintNum(point_id++);
                UART_Print(",");
                UART_PrintNum(distance);
                UART_Print("\r\n");
                GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }
        }

        delay_ms(20);
    }
}
