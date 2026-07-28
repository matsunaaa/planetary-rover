#include "driverlib.h"
#include "msp.h"

void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 3000; j++)
            __no_operation();
}

void usb_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    eUSCI_UART_Config cfg = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        6, 8, 0x20,
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };
    UART_initModule(EUSCI_A0_BASE, &cfg);
    UART_enableModule(EUSCI_A0_BASE);
}

void usb_print(const char *s) {
    while (*s) {
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A0->TXBUF = *s++;
    }
}

void usb_char(char c) {
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = c;
}

void bt_init(void) {
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST;
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;
    EUSCI_A2->BRW = 19;
    EUSCI_A2->MCTLW = (8 << 4) | (0x55 << 8) | EUSCI_A_MCTLW_OS16;
    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
}

void bt_send(const char *s) {
    while (*s) {
        while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A2->TXBUF = *s++;
    }
}

void bt_read_response(void) {
    uint32_t timeout = 0;
    char c;

    while (timeout < 500000) {
        if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG) {
            c = EUSCI_A2->RXBUF;
            if (c >= 32 && c < 127) {
                usb_char(c);
            } else if (c == '\r') {
                usb_print("<CR>");
            } else if (c == '\n') {
                usb_print("<LF>\r\n");
            }
            timeout = 0;
        }
        timeout++;
    }
}

void at_cmd(const char *cmd) {
    usb_print(">> ");
    usb_print(cmd);
    usb_print("\r\n<< ");

    while (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG) {
        (void)EUSCI_A2->RXBUF;
    }

    bt_send(cmd);
    bt_send("\r\n");
    bt_read_response();
    usb_print("\r\n");
}

int main(void) {
    WDT_A_holdTimer();

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

    delay_ms(500);
    usb_init();
    bt_init();
    delay_ms(100);

    usb_print("\r\n=== HC-05 Configuration ===\r\n\r\n");

    /* Query current settings */
    at_cmd("AT");
    at_cmd("AT+VERSION");
    at_cmd("AT+NAME");
    at_cmd("AT+UART");
    at_cmd("AT+PSWD");
    at_cmd("AT+ROLE");

    usb_print("--- Current settings above ---\r\n\r\n");
    delay_ms(1000);

    /* Configure for project */
    usb_print("--- Configuring... ---\r\n\r\n");

    at_cmd("AT+NAME=ROVER3D");           /* Set name */
    at_cmd("AT+UART=115200,0,0");        /* 115200 baud, 1 stop, no parity */
    at_cmd("AT+PSWD=1234");              /* PIN 1234 */

    usb_print("--- Verify new settings ---\r\n\r\n");

    at_cmd("AT+NAME");
    at_cmd("AT+UART");

    usb_print("\r\n=== DONE ===\r\n");
    usb_print("Power cycle HC-05 (remove EN/KEY from HIGH)\r\n");
    usb_print("Then it will use 115200 baud for data mode\r\n");

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    while (1) {
        delay_ms(1000);
    }
}
