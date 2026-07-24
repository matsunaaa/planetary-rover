#include "motors.h"
#include "config.h"
#include "msp.h"
#include "driverlib.h"

#define TIMER_PERIOD 15000  // Same as your working code

/* Timer_A UpDown Configuration */
Timer_A_UpDownModeConfig upDownConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_4,
    TIMER_PERIOD,
    TIMER_A_TAIE_INTERRUPT_DISABLE,
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
    TIMER_A_DO_CLEAR
};

/* PWM Compare Configs */
Timer_A_CompareModeConfig compareConfig_PWM1 =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_1,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,
    0
};

Timer_A_CompareModeConfig compareConfig_PWM2 =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_2,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,
    0
};

static void PWM_duty1(uint16_t duty1)
{
    if (duty1 >= TIMER_PERIOD) return;
    compareConfig_PWM1.compareValue = duty1;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);
}

static void PWM_duty2(uint16_t duty2)
{
    if (duty2 >= TIMER_PERIOD) return;
    compareConfig_PWM2.compareValue = duty2;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM2);
}

void Motors_init(void)
{
    // PWM pins: P2.4 (right), P2.5 (left)
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P2,
        GPIO_PIN4 | GPIO_PIN5,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    // Direction pins: P3.0 and P3.6
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);

    // ENSLEEP pins: P3.5 and P3.7 (directly drive low to keep enabled)
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);

    // Configure Timer_A0 for PWM
    Timer_A_configureUpDownMode(TIMER_A0_BASE, &upDownConfig);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UPDOWN_MODE);

    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM2);
}

void Motors_enable(void)
{
    // Already enabled via P3.5/P3.7 low in init
}

void Motors_disable(void)
{
    Motors_stop();
}

void Motors_setLeft(int16_t speed)
{
    uint16_t duty;

    if (speed >= 0)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);  // Forward
        duty = (speed > TIMER_PERIOD) ? TIMER_PERIOD - 1 : speed;
    }
    else
    {
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);   // Backward (check if inverted)
        duty = (-speed > TIMER_PERIOD) ? TIMER_PERIOD - 1 : -speed;
    }
    PWM_duty2(duty);  // Left = PWM2 = P2.5
}

void Motors_setRight(int16_t speed)
{
    uint16_t duty;

    if (speed >= 0)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);  // Forward
        duty = (speed > TIMER_PERIOD) ? TIMER_PERIOD - 1 : speed;
    }
    else
    {
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);   // Backward (check if inverted)
        duty = (-speed > TIMER_PERIOD) ? TIMER_PERIOD - 1 : -speed;
    }
    PWM_duty1(duty);  // Right = PWM1 = P2.4
}

void Motors_forward(uint16_t speed)
{
    // From your working code
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
    PWM_duty1(speed);
    PWM_duty2(speed);
}

void Motors_backward(uint16_t speed)
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
    PWM_duty1(speed);
    PWM_duty2(speed);
}

void Motors_turnLeft(uint16_t speed)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
    PWM_duty1(speed);
    PWM_duty2(speed);
}

void Motors_turnRight(uint16_t speed)
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
    PWM_duty1(speed);
    PWM_duty2(speed);
}

void Motors_stop(void)
{
    PWM_duty1(0);
    PWM_duty2(0);
}
