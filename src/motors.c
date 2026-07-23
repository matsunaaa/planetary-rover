#include "motors.h"
#include "config.h"
#include "msp.h"

/*
 * RSLK Motor Pin Mapping (from your working code):
 *
 * LEFT MOTOR:
 *   PWM:  P2.5 (TA0.2)
 *   DIR:  P3.6
 *   nSLP: P3.7 (active high? - keep low to enable based on your code)
 *
 * RIGHT MOTOR:
 *   PWM:  P2.4 (TA0.1)
 *   DIR:  P3.0
 *   nSLP: P3.5 (active high? - keep low to enable based on your code)
 *
 * Timer_A0 in Up/Down mode, period = 15000
 * CCR1 = Right PWM (P2.4)
 * CCR2 = Left PWM (P2.5)
 */

#define PWM_PERIOD 15000

void Motors_init(void)
{
    // ===== Direction Pins: P3.0 (right), P3.6 (left) =====
    P3->DIR |= BIT0 | BIT6;         // Output
    P3->OUT |= BIT0 | BIT6;         // High = forward (from your working code)

    // ===== nSLP Pins: P3.5 (right), P3.7 (left) =====
    // Your working code sets these LOW for forward
    P3->DIR |= BIT5 | BIT7;         // Output
    P3->OUT &= ~(BIT5 | BIT7);      // Low = enabled

    // ===== PWM Pins: P2.4 (TA0.1), P2.5 (TA0.2) =====
    P2->DIR |= BIT4 | BIT5;         // Output
    P2->SEL0 |= BIT4 | BIT5;        // Timer function
    P2->SEL1 &= ~(BIT4 | BIT5);     // Timer function

    // ===== Timer_A0 Configuration =====
    // Up/Down mode, SMCLK/4
    // SMCLK = 3MHz default, /4 = 750kHz
    // Period 15000 → ~25Hz PWM (same as your code)

    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |   // SMCLK
                    TIMER_A_CTL_ID__4 |          // Divide by 4
                    TIMER_A_CTL_MC__UPDOWN |     // Up/Down mode
                    TIMER_A_CTL_CLR;             // Clear

    TIMER_A0->CCR[0] = PWM_PERIOD;              // Period

    // CCR1 = Right motor (P2.4) - Toggle/Reset mode
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_6; // Toggle/Set
    TIMER_A0->CCR[1] = 0;                        // Start at 0

    // CCR2 = Left motor (P2.5) - Toggle/Reset mode
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_6; // Toggle/Set
    TIMER_A0->CCR[2] = 0;                        // Start at 0
}

void Motors_enable(void)
{
    // Already enabled in init
}

void Motors_disable(void)
{
    Motors_stop();
}

// Set right motor PWM (0 to 15000)
static void PWM_setRight(uint16_t duty)
{
    if (duty >= PWM_PERIOD) duty = PWM_PERIOD - 1;
    TIMER_A0->CCR[1] = duty;
}

// Set left motor PWM (0 to 15000)
static void PWM_setLeft(uint16_t duty)
{
    if (duty >= PWM_PERIOD) duty = PWM_PERIOD - 1;
    TIMER_A0->CCR[2] = duty;
}

void Motors_forward(uint16_t speed)
{
    // From your working code: motor_forward
    P3->OUT &= ~(BIT5 | BIT7);      // nSLP low
    P3->OUT |= BIT0 | BIT6;         // DIR high
    PWM_setRight(speed);
    PWM_setLeft(speed);
}

void Motors_backward(uint16_t speed)
{
    // From your working code: motor_backward
    P3->OUT |= BIT5 | BIT7;         // nSLP high
    P3->OUT |= BIT0 | BIT6;         // DIR high
    PWM_setRight(speed);
    PWM_setLeft(speed);
}

void Motors_turnLeft(uint16_t speed)
{
    // From your working code: motor_left
    P3->OUT &= ~BIT5;               // Right nSLP low
    P3->OUT |= BIT7;                // Left nSLP high
    P3->OUT |= BIT0 | BIT6;         // DIR high
    PWM_setRight(speed);
    PWM_setLeft(speed);
}

void Motors_turnRight(uint16_t speed)
{
    // From your working code: motor_right
    P3->OUT |= BIT5;                // Right nSLP high
    P3->OUT &= ~BIT7;               // Left nSLP low
    P3->OUT |= BIT0 | BIT6;         // DIR high
    PWM_setRight(speed);
    PWM_setLeft(speed);
}

void Motors_stop(void)
{
    PWM_setRight(0);
    PWM_setLeft(0);
}

void Motors_setLeft(int16_t speed)
{
    if (speed >= 0)
    {
        P3->OUT &= ~BIT7;           // nSLP low (forward)
        P3->OUT |= BIT6;            // DIR high
        PWM_setLeft(speed);
    }
    else
    {
        P3->OUT |= BIT7;            // nSLP high (backward)
        P3->OUT |= BIT6;            // DIR high
        PWM_setLeft(-speed);
    }
}

void Motors_setRight(int16_t speed)
{
    if (speed >= 0)
    {
        P3->OUT &= ~BIT5;           // nSLP low (forward)
        P3->OUT |= BIT0;            // DIR high
        PWM_setRight(speed);
    }
    else
    {
        P3->OUT |= BIT5;            // nSLP high (backward)
        P3->OUT |= BIT0;            // DIR high
        PWM_setRight(-speed);
    }
}
