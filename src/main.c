#include "msp.h"
#include "config.h"
#include "encoders.h"
#include "odometry.h"
#include "motors.h"
#include <stdio.h>

// ========== Simple UART for Debug ==========
void UART_init(void)
{
    P1->SEL0 |= BIT2 | BIT3;
    P1->SEL1 &= ~(BIT2 | BIT3);

    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST;
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;

    // 115200 baud @ 3MHz SMCLK
    EUSCI_A0->BRW = 26;
    EUSCI_A0->MCTLW = 0;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
}

void UART_print(char *str)
{
    while (*str)
    {
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A0->TXBUF = *str++;
    }
}

void UART_printInt(int32_t val)
{
    char buf[16];
    sprintf(buf, "%ld", val);
    UART_print(buf);
}

void UART_printFloat(float val)
{
    char buf[16];
    sprintf(buf, "%.1f", val);
    UART_print(buf);
}

// ========== Delay ==========
void delay_ms(uint32_t ms)
{
    // Rough delay at 3MHz default clock
    volatile uint32_t i;
    for (i = 0; i < ms * 750; i++);
}

// ========== Main ==========
int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    UART_init();
    Odometry_init();    // Also inits encoders
    Motors_init();

    __enable_irq();

    UART_print("\r\n=== RSLK Motor + Encoder Test ===\r\n");
    UART_print("Starting in 2 seconds...\r\n\r\n");
    delay_ms(2000);

    Motors_enable();

    // Test 1: Drive forward for 2 seconds
    UART_print("Test 1: Forward\r\n");
    Motors_forward(3000);   // 30% speed

    for (int i = 0; i < 20; i++)
    {
        delay_ms(100);
        Odometry_update();

        RobotPose pose = Odometry_getPose();
        UART_print("X:");
        UART_printFloat(pose.x_mm);
        UART_print(" Y:");
        UART_printFloat(pose.y_mm);
        UART_print(" Th:");
        UART_printFloat(pose.theta_rad * 57.3f);
        UART_print("deg\r\n");
    }

    Motors_stop();
    delay_ms(1000);

    // Test 2: Rotate right for 1 second
    UART_print("\r\nTest 2: Rotate Right\r\n");
    Motors_turnRight(3000);

    for (int i = 0; i < 10; i++)
    {
        delay_ms(100);
        Odometry_update();

        RobotPose pose = Odometry_getPose();
        UART_print("Th:");
        UART_printFloat(pose.theta_rad * 57.3f);
        UART_print("deg\r\n");
    }

    Motors_stop();
    Motors_disable();

    UART_print("\r\n=== Test Complete ===\r\n");

    RobotPose final_pose = Odometry_getPose();
    UART_print("Final Position: X=");
    UART_printFloat(final_pose.x_mm);
    UART_print("mm Y=");
    UART_printFloat(final_pose.y_mm);
    UART_print("mm Theta=");
    UART_printFloat(final_pose.theta_rad * 57.3f);
    UART_print("deg\r\n");

    while (1)
    {
        // Done - could add continuous monitoring here
        delay_ms(1000);
    }
}
