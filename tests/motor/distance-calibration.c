/* Distance Calibration - Simple Version
 *
 * How to use:
 * 1. Flash code
 * 2. Robot drives forward, stops, waits, repeats
 * 3. Measure distance traveled
 * 4. Adjust DRIVE_TIME_MS until you get 30cm
 * 5. Calculate: CM_PER_MS = 30.0 / DRIVE_TIME_MS
 */

#include "driverlib.h"
#include <stdint.h>

/*===========================================================================*/
/* CALIBRATION CONSTANTS - TWEAK THESE                                        */
/*===========================================================================*/

#define SPEED_PCT       40      /* Speed as % of max (20-60 reasonable) */
#define DRIVE_TIME_MS   1000    /* How long to drive - ADJUST THIS */
#define PAUSE_TIME_MS   3000    /* Time between runs to measure/reset */

#define TIMER_PERIOD    15000   /* PWM period - don't change */
#define RIGHT_RATIO     96      /* Right = Left * 96/100 */

/*===========================================================================*/
/* DELAY                                                                      */
/*===========================================================================*/

void delay_ms(uint32_t ms)
{
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 3000; j++)
            __no_operation();
}

/*===========================================================================*/
/* PWM SETUP                                                                  */
/*===========================================================================*/

Timer_A_UpDownModeConfig upDownConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    TIMER_PERIOD,
    TIMER_A_TAIE_INTERRUPT_DISABLE,
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
    TIMER_A_DO_CLEAR
};

Timer_A_CompareModeConfig pwmRight =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_1,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,
    0
};

Timer_A_CompareModeConfig pwmLeft =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_2,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,
    0
};

void PWM_Set(uint16_t left, uint16_t right)
{
    pwmLeft.compareValue = left;
    pwmRight.compareValue = right;
    Timer_A_initCompare(TIMER_A0_BASE, &pwmLeft);
    Timer_A_initCompare(TIMER_A0_BASE, &pwmRight);
}

/*===========================================================================*/
/* MOTOR CONTROL                                                              */
/*===========================================================================*/

void Motor_Forward(uint16_t speedPct)
{
    uint16_t leftDuty = (TIMER_PERIOD * speedPct) / 100;
    uint16_t rightDuty = (leftDuty * RIGHT_RATIO) / 100;

    /* Direction LOW = forward */
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);
    /* Wake motors */
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);

    PWM_Set(leftDuty, rightDuty);
}

void Motor_Stop(void)
{
    PWM_Set(0, 0);
}

/*===========================================================================*/
/* MAIN                                                                       */
/*===========================================================================*/

int main(void)
{
    WDT_A_holdTimer();

    /* 12MHz clock */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* LED */
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Motor direction/sleep pins */
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);  /* Sleep initially */

    /* PWM pins */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
        GPIO_PIN4 | GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    Timer_A_configureUpDownMode(TIMER_A0_BASE, &upDownConfig);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UPDOWN_MODE);
    Timer_A_initCompare(TIMER_A0_BASE, &pwmRight);
    Timer_A_initCompare(TIMER_A0_BASE, &pwmLeft);

    /* Initial pause - time to place robot */
    delay_ms(2000);

    while (1)
    {
        /* LED ON = driving */
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

        Motor_Forward(SPEED_PCT);
        delay_ms(DRIVE_TIME_MS);
        Motor_Stop();

        /* LED OFF = stopped, measure now */
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

        /* Wait for you to measure and reset position */
        delay_ms(PAUSE_TIME_MS);
    }
}
