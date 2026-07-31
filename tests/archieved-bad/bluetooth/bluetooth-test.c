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

void usb_int(int32_t val) {
    char buf[12];
    int i = 0;
    if (val < 0) { usb_print("-"); val = -val; }
    if (val == 0) { usb_print("0"); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i--) {
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A0->TXBUF = buf[i];
    }
}

/* HC-05 at 115200 baud (data mode) */
void bt_init(void) {
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST;
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;

    /* 115200 from 12MHz: BRW=6, UCxBRF=8, UCxBRS=0x20 */
    EUSCI_A2->BRW = 6;
    EUSCI_A2->MCTLW = (8 << 4) | (0x20 << 8) | EUSCI_A_MCTLW_OS16;

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
}

void bt_print(const char *s) {
    while (*s) {
        while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A2->TXBUF = *s++;
    }
}

void bt_int(int32_t val) {
    char buf[12];
    int i = 0;
    if (val < 0) { bt_print("-"); val = -val; }
    if (val == 0) { bt_print("0"); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i--) {
        while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A2->TXBUF = buf[i];
    }
}

int main(void) {
    int32_t count = 0;

    WDT_A_holdTimer();

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    delay_ms(500);
    usb_init();
    bt_init();
    delay_ms(100);

    usb_print("\r\n=== HC-05 Data Mode Test ===\r\n");
    usb_print("Pair laptop to ROVER3D, open serial terminal at 115200\r\n\r\n");

    while (1) {
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

        /* Send to USB (debug) */
        usb_print("Count: ");
        usb_int(count);
        usb_print("\r\n");

        /* Send to Bluetooth */
        bt_print("P,");
        bt_int(count);
        bt_print(",");
        bt_int(count * 2);
        bt_print(",");
        bt_int(count * 3);
        bt_print("\r\n");

        count++;
        delay_ms(500);
    }
}
