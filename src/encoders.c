#include "encoders.h"
#include "config.h"
#include "msp.h"

/*
 * RSLK Quadrature Encoder Decoding
 *
 * Quadrature provides direction sensing:
 * - Channel A triggers interrupt
 * - Read Channel B to determine direction
 *
 * If A rises and B is low  → Forward
 * If A rises and B is high → Backward
 *
 * Pin mapping:
 * Left:  A = P10.5, B = P5.2
 * Right: A = P10.4, B = P5.0
 *
 * Note: P10 doesn't have interrupt capability on MSP432.
 * We'll use Timer_A capture or polling workaround.
 *
 * ALTERNATIVE: Use TimerA input capture on P5 pins (B channels)
 * and read A pins for direction. P5.0 and P5.2 support interrupts.
 */

// Encoder counts (volatile - modified in ISR)
static volatile int32_t left_count = 0;
static volatile int32_t right_count = 0;
static volatile int32_t left_last = 0;
static volatile int32_t right_last = 0;

void Encoders_init(void)
{
    // ========== Configure Encoder Pins as Inputs ==========

    // Left Encoder: A = P10.5, B = P5.2
    P10->DIR &= ~BIT5;      // Input
    P10->REN |= BIT5;       // Enable resistor
    P10->OUT |= BIT5;       // Pull-up

    P5->DIR &= ~BIT2;       // Input
    P5->REN |= BIT2;        // Enable resistor
    P5->OUT |= BIT2;        // Pull-up

    // Right Encoder: A = P10.4, B = P5.0
    P10->DIR &= ~BIT4;      // Input
    P10->REN |= BIT4;       // Enable resistor
    P10->OUT |= BIT4;       // Pull-up

    P5->DIR &= ~BIT0;       // Input
    P5->REN |= BIT0;        // Enable resistor
    P5->OUT |= BIT0;        // Pull-up

    // ========== Configure Interrupts on B Channels (P5) ==========
    // P5.0 (Right B) and P5.2 (Left B) support interrupts
    // We'll trigger on B edges and read A for direction

    P5->IES &= ~(BIT0 | BIT2);   // Rising edge initially
    P5->IFG &= ~(BIT0 | BIT2);   // Clear flags
    P5->IE  |= (BIT0 | BIT2);    // Enable interrupts

    // Enable Port 5 interrupt in NVIC
    NVIC_SetPriority(PORT5_IRQn, 2);
    NVIC_EnableIRQ(PORT5_IRQn);
}

int32_t Encoders_getLeftCount(void)
{
    return left_count;
}

int32_t Encoders_getRightCount(void)
{
    return right_count;
}

void Encoders_resetCounts(void)
{
    __disable_irq();
    left_count = 0;
    right_count = 0;
    left_last = 0;
    right_last = 0;
    __enable_irq();
}

int32_t Encoders_getLeftDelta(void)
{
    __disable_irq();
    int32_t delta = left_count - left_last;
    left_last = left_count;
    __enable_irq();
    return delta;
}

int32_t Encoders_getRightDelta(void)
{
    __disable_irq();
    int32_t delta = right_count - right_last;
    right_last = right_count;
    __enable_irq();
    return delta;
}

/*
 * Port 5 ISR
 *
 * Quadrature decoding:
 * When B rises, check A:
 *   - A high → one direction
 *   - A low  → other direction
 *
 * We also toggle edge detection to catch both rising and falling
 * for higher resolution (4x decoding could be added later)
 */
void PORT5_IRQHandler(void)
{
    uint8_t flags = P5->IFG;

    // Left Encoder (B = P5.2, A = P10.5)
    if (flags & BIT2)
    {
        uint8_t b_state = P5->IN & BIT2;
        uint8_t a_state = P10->IN & BIT5;

        // Determine direction based on A/B phase relationship
        if (b_state)  // B just rose
        {
            if (a_state)
                left_count--;   // Backward
            else
                left_count++;   // Forward
        }
        else  // B just fell
        {
            if (a_state)
                left_count++;   // Forward
            else
                left_count--;   // Backward
        }

        // Toggle edge detection for next interrupt
        P5->IES ^= BIT2;
        P5->IFG &= ~BIT2;
    }

    // Right Encoder (B = P5.0, A = P10.4)
    if (flags & BIT0)
    {
        uint8_t b_state = P5->IN & BIT0;
        uint8_t a_state = P10->IN & BIT4;

        if (b_state)  // B just rose
        {
            if (a_state)
                right_count++;  // Forward (reversed from left due to motor orientation)
            else
                right_count--;  // Backward
        }
        else  // B just fell
        {
            if (a_state)
                right_count--;  // Backward
            else
                right_count++;  // Forward
        }

        // Toggle edge detection
        P5->IES ^= BIT0;
        P5->IFG &= ~BIT0;
    }
}
