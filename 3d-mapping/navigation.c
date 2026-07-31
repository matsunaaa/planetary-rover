/* rover3d.c
 * MSP432 autonomous rover: drives, scans with pan-tilt ToF, avoids obstacles.
 * All servo movements are gradual (1 deg / 30ms) — no snaps.
 *
 * Pins:
 *   P1.0  - LED (heartbeat)
 *   P1.3  - USB UART TX
 *   P2.4  - Left motor PWM (TA0.1)
 *   P2.5  - Right motor PWM (TA0.2)
 *   P2.6  - Pan servo (TA0.3)
 *   P2.7  - Tilt servo (TA0.4)
 *   P3.0  - Right motor SLP
 *   P3.3  - ESP32 UART TX (EUSCI_A2)
 *   P3.5  - Right motor DIR
 *   P3.6  - Left motor SLP
 *   P3.7  - Left motor DIR
 *
 * Navigation: Drive forward, scan, check obstacles, turn if blocked.
 * Periodically does 180 turn to scan behind.
 */

#include "driverlib.h"
#include <stdint.h>
#include <math.h>

/*===========================================================================*/
/* CONFIGURATION                                                             */
/*===========================================================================*/

#define PAN_MIN         0
#define PAN_MAX         110
#define PAN_CENTER      55
#define TILT_MIN        37
#define TILT_MAX        180
#define TILT_HORIZONTAL 90

#define SMOOTH_DELAY_MS 30
#define SCAN_STEP       5
#define SCAN_SETTLE_MS  40

#define BASE_PWM        9000
#define RIGHT_RATIO     96
#define SPEED_CM_PER_MS 0.038f

#define DRIVE_DIST_CM       30
#define OBSTACLE_DIST_MM    300
#define TURN_ANGLE_DEG      45
#define SCANS_BEFORE_180    4

#define SERVO_PERIOD    30000
#define SERVO_MIN       1500
#define SERVO_MAX       3000
#define MOTOR_PERIOD    30000

static float rover_x = 0.0f;
static float rover_y = 0.0f;
static float rover_heading = 0.0f;
static int scan_count = 0;
static uint16_t cur_pan = 90;
static uint16_t cur_tilt = 90;

/*===========================================================================*/
/* DELAY                                                                     */
/*===========================================================================*/

void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 3000; j++)
            __no_operation();
}

/*===========================================================================*/
/* UART - USB Debug (EUSCI_A0, P1.2/P1.3)                                   */
/*===========================================================================*/

const eUSCI_UART_Config uartCfg = {
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,
    6, 8, 32,
    EUSCI_A_UART_NO_PARITY,
    EUSCI_A_UART_LSB_FIRST,
    EUSCI_A_UART_ONE_STOP_BIT,
    EUSCI_A_UART_MODE,
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

void uart_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A0_BASE, &uartCfg);
    UART_enableModule(EUSCI_A0_BASE);
}

void uart_putc(char c) {
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = c;
}

void uart_print(const char *s) {
    while (*s) uart_putc(*s++);
}

void uart_int(int32_t val) {
    char buf[12];
    int i = 0;
    if (val < 0) { uart_putc('-'); val = -val; }
    if (val == 0) { uart_putc('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}

/*===========================================================================*/
/* UART - ESP32 (EUSCI_A2, P3.2/P3.3)                                       */
/*===========================================================================*/

void esp_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A2_BASE, &uartCfg);
    UART_enableModule(EUSCI_A2_BASE);
}

void esp_putc(char c) {
    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A2->TXBUF = c;
}

void esp_print(const char *s) {
    while (*s) esp_putc(*s++);
}

void esp_int(int32_t val) {
    char buf[12];
    int i = 0;
    if (val < 0) { esp_putc('-'); val = -val; }
    if (val == 0) { esp_putc('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) esp_putc(buf[--i]);
}

/*===========================================================================*/
/* Dual output helpers                                                       */
/*===========================================================================*/

void send_msg(const char *msg) {
    uart_print(msg); uart_print("\r\n");
    esp_print(msg); esp_print("\r\n");
}

void send_servo_pos(int pan, int tilt) {
    uart_print("P,"); uart_int(pan);
    uart_putc(','); uart_int(tilt);
    uart_print("\r\n");
    esp_print("P,"); esp_int(pan);
    esp_putc(','); esp_int(tilt);
    esp_print("\r\n");
}

void send_rover_pos(void) {
    int x_mm = (int)(rover_x * 10);
    int y_mm = (int)(rover_y * 10);
    int h_deg = (int)rover_heading;
    uart_print("R,"); uart_int(x_mm);
    uart_putc(','); uart_int(y_mm);
    uart_putc(','); uart_int(h_deg);
    uart_print("\r\n");
    esp_print("R,"); esp_int(x_mm);
    esp_putc(','); esp_int(y_mm);
    esp_putc(','); esp_int(h_deg);
    esp_print("\r\n");
}

/*===========================================================================*/
/* MOTOR CONTROL                                                             */
/*===========================================================================*/

#define LEFT_DIR_PORT   GPIO_PORT_P3
#define LEFT_DIR_PIN    GPIO_PIN7
#define LEFT_SLP_PORT   GPIO_PORT_P3
#define LEFT_SLP_PIN    GPIO_PIN6
#define RIGHT_DIR_PORT  GPIO_PORT_P3
#define RIGHT_DIR_PIN   GPIO_PIN5
#define RIGHT_SLP_PORT  GPIO_PORT_P3
#define RIGHT_SLP_PIN   GPIO_PIN0

void motor_init(void) {
    GPIO_setAsOutputPin(LEFT_DIR_PORT, LEFT_DIR_PIN);
    GPIO_setAsOutputPin(RIGHT_DIR_PORT, RIGHT_DIR_PIN);
    GPIO_setAsOutputPin(LEFT_SLP_PORT, LEFT_SLP_PIN);
    GPIO_setAsOutputPin(RIGHT_SLP_PORT, RIGHT_SLP_PIN);
    GPIO_setOutputHighOnPin(LEFT_SLP_PORT, LEFT_SLP_PIN);
    GPIO_setOutputHighOnPin(RIGHT_SLP_PORT, RIGHT_SLP_PIN);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
        GPIO_PIN4 | GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setOutputLowOnPin(LEFT_DIR_PORT, LEFT_DIR_PIN);
    GPIO_setOutputLowOnPin(RIGHT_DIR_PORT, RIGHT_DIR_PIN);
}

void motor_set_pwm(uint16_t left, uint16_t right) {
    right = (right * RIGHT_RATIO) / 100;
    if (left > MOTOR_PERIOD) left = MOTOR_PERIOD;
    if (right > MOTOR_PERIOD) right = MOTOR_PERIOD;
    TIMER_A0->CCR[1] = left;
    TIMER_A0->CCR[2] = right;
}

void motor_forward(uint16_t pwm) {
    GPIO_setOutputLowOnPin(LEFT_DIR_PORT, LEFT_DIR_PIN);
    GPIO_setOutputLowOnPin(RIGHT_DIR_PORT, RIGHT_DIR_PIN);
    motor_set_pwm(pwm, pwm);
}

void motor_backward(uint16_t pwm) {
    GPIO_setOutputHighOnPin(LEFT_DIR_PORT, LEFT_DIR_PIN);
    GPIO_setOutputHighOnPin(RIGHT_DIR_PORT, RIGHT_DIR_PIN);
    motor_set_pwm(pwm, pwm);
}

void motor_stop(void) {
    motor_set_pwm(0, 0);
}

void motor_turn_left(uint16_t pwm) {
    GPIO_setOutputHighOnPin(LEFT_DIR_PORT, LEFT_DIR_PIN);
    GPIO_setOutputLowOnPin(RIGHT_DIR_PORT, RIGHT_DIR_PIN);
    motor_set_pwm(pwm, pwm);
}

void motor_turn_right(uint16_t pwm) {
    GPIO_setOutputLowOnPin(LEFT_DIR_PORT, LEFT_DIR_PIN);
    GPIO_setOutputHighOnPin(RIGHT_DIR_PORT, RIGHT_DIR_PIN);
    motor_set_pwm(pwm, pwm);
}

/*===========================================================================*/
/* SERVO CONTROL                                                             */
/*===========================================================================*/

uint16_t deg_to_servo_pulse(uint16_t deg) {
    if (deg > 180) deg = 180;
    return SERVO_MIN + ((uint32_t)deg * (SERVO_MAX - SERVO_MIN)) / 180;
}

void servo_set_pan(uint16_t deg) {
    cur_pan = deg;
    TIMER_A0->CCR[3] = deg_to_servo_pulse(deg);
}

void servo_set_tilt(uint16_t deg) {
    cur_tilt = deg;
    TIMER_A0->CCR[4] = deg_to_servo_pulse(deg);
}

void smooth_pan_to(uint16_t target) {
    if (target == cur_pan) return;
    int16_t step = (target > cur_pan) ? 1 : -1;
    while (cur_pan != target) {
        servo_set_pan(cur_pan + step);
        delay_ms(SMOOTH_DELAY_MS);
    }
}

void smooth_tilt_to(uint16_t target) {
    if (target == cur_tilt) return;
    int16_t step = (target > cur_tilt) ? 1 : -1;
    while (cur_tilt != target) {
        servo_set_tilt(cur_tilt + step);
        delay_ms(SMOOTH_DELAY_MS);
    }
}

void servo_center(void) {
    smooth_pan_to(PAN_CENTER);
    smooth_tilt_to(TILT_HORIZONTAL);
}

/*===========================================================================*/
/* COMBINED TIMER INIT (Motors + Servos on TA0)                             */
/*===========================================================================*/

void timer_init(void) {
    TIMER_A0->CTL = 0;

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
        GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7,
        GPIO_PRIMARY_MODULE_FUNCTION);

    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |
                    TIMER_A_CTL_ID__8 |
                    TIMER_A_CTL_CLR;

    TIMER_A0->CCR[0] = SERVO_PERIOD;

    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[1] = 0;

    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[2] = 0;

    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[3] = deg_to_servo_pulse(90);

    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[4] = deg_to_servo_pulse(90);

    TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;
}

/*===========================================================================*/
/* MOTION FUNCTIONS WITH ODOMETRY                                            */
/*===========================================================================*/

#define PI 3.14159265f

void update_position(float dist_cm, float delta_heading_deg) {
    rover_heading += delta_heading_deg;
    while (rover_heading >= 360.0f) rover_heading -= 360.0f;
    while (rover_heading < 0.0f) rover_heading += 360.0f;
    float heading_rad = rover_heading * PI / 180.0f;
    rover_x += dist_cm * sinf(heading_rad);
    rover_y += dist_cm * cosf(heading_rad);
}

void drive_forward_cm(float cm) {
    uint32_t time_ms;
    if (cm <= 0) return;
    time_ms = (uint32_t)(cm / SPEED_CM_PER_MS);
    send_msg("Drive fwd");
    motor_forward(BASE_PWM);
    delay_ms(time_ms);
    motor_stop();
    update_position(cm, 0);
    send_rover_pos();
}

void drive_backward_cm(float cm) {
    uint32_t time_ms;
    if (cm <= 0) return;
    time_ms = (uint32_t)(cm / SPEED_CM_PER_MS);
    send_msg("Drive back");
    motor_backward(BASE_PWM);
    delay_ms(time_ms);
    motor_stop();
    update_position(-cm, 0);
    send_rover_pos();
}

void turn_deg(float degrees) {
    uint32_t time_ms;
    float abs_deg;
    #define TURN_MS_PER_DEG 5.5f
    abs_deg = degrees;
    if (abs_deg < 0) abs_deg = -abs_deg;
    time_ms = (uint32_t)(abs_deg * TURN_MS_PER_DEG);
    if (degrees > 0) {
        send_msg("Turn right");
        motor_turn_right(BASE_PWM);
    } else {
        send_msg("Turn left");
        motor_turn_left(BASE_PWM);
    }
    delay_ms(time_ms);
    motor_stop();
    update_position(0, degrees);
    send_rover_pos();
}

/*===========================================================================*/
/* SCANNING                                                                  */
/*===========================================================================*/

int check_obstacle(void) {
    smooth_pan_to(PAN_CENTER);
    smooth_tilt_to(TILT_HORIZONTAL);
    send_servo_pos(PAN_CENTER, TILT_HORIZONTAL);
    delay_ms(50);
    return 0;
}

void do_full_scan(void) {
    int pan, tilt;
    int pan_dir;

    send_msg("Scan start");

    smooth_pan_to(PAN_MIN);
    smooth_tilt_to(TILT_MIN);

    pan_dir = 1;

    for (tilt = TILT_MIN; tilt <= TILT_MAX; tilt += SCAN_STEP) {
        smooth_tilt_to((uint16_t)tilt);
        smooth_pan_to((pan_dir == 1) ? PAN_MIN : PAN_MAX);

        if (pan_dir == 1) {
            for (pan = PAN_MIN; pan <= PAN_MAX; pan += SCAN_STEP) {
                servo_set_pan((uint16_t)pan);
                delay_ms(SCAN_SETTLE_MS);
                send_servo_pos(pan, tilt);
            }
        } else {
            for (pan = PAN_MAX; pan >= PAN_MIN; pan -= SCAN_STEP) {
                servo_set_pan((uint16_t)pan);
                delay_ms(SCAN_SETTLE_MS);
                send_servo_pos(pan, tilt);
            }
        }

        pan_dir = -pan_dir;
    }

    servo_center();
    send_msg("Scan done");
    scan_count++;
}

void do_quick_scan(void) {
    int pan;

    send_msg("Quick scan");

    smooth_tilt_to(TILT_HORIZONTAL);
    smooth_pan_to(PAN_MIN);

    for (pan = PAN_MIN; pan <= PAN_MAX; pan += SCAN_STEP) {
        servo_set_pan((uint16_t)pan);
        delay_ms(SCAN_SETTLE_MS);
        send_servo_pos(pan, TILT_HORIZONTAL);
    }

    servo_center();
    send_msg("Quick done");
}

/*===========================================================================*/
/* AUTONOMOUS NAVIGATION                                                     */
/*===========================================================================*/

typedef enum {
    STATE_INIT,
    STATE_SCAN,
    STATE_DRIVE,
    STATE_TURN,
    STATE_TURN_180,
    STATE_DONE
} RoverState;

static RoverState state = STATE_INIT;
static int turns_since_180 = 0;

void nav_step(void) {
    switch (state) {
        case STATE_INIT:
            send_msg("=== Rover3D Start ===");
            send_rover_pos();
            smooth_pan_to(PAN_CENTER);
            smooth_tilt_to(TILT_HORIZONTAL);
            delay_ms(500);
            state = STATE_SCAN;
            break;

        case STATE_SCAN:
            do_full_scan();
            if (turns_since_180 >= SCANS_BEFORE_180)
                state = STATE_TURN_180;
            else
                state = STATE_DRIVE;
            break;

        case STATE_DRIVE:
            drive_forward_cm(DRIVE_DIST_CM);
            delay_ms(200);
            do_quick_scan();
            state = STATE_SCAN;
            break;

        case STATE_TURN:
            if ((scan_count % 2) == 0)
                turn_deg(TURN_ANGLE_DEG);
            else
                turn_deg(-TURN_ANGLE_DEG);
            delay_ms(200);
            turns_since_180++;
            state = STATE_SCAN;
            break;

        case STATE_TURN_180:
            send_msg("Turn 180");
            turn_deg(180);
            delay_ms(300);
            turns_since_180 = 0;
            state = STATE_SCAN;
            break;

        case STATE_DONE:
            motor_stop();
            send_msg("Done");
            delay_ms(1000);
            break;
    }
}

/*===========================================================================*/
/* MAIN                                                                      */
/*===========================================================================*/

int main(void) {
    WDT_A_holdTimer();
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    uart_init();
    esp_init();
    delay_ms(100);

    send_msg("Rover3D init");

    motor_init();
    timer_init();
    delay_ms(500);

    send_msg("Motors ready");
    send_msg("Servos ready");

    smooth_pan_to(90);
    smooth_tilt_to(90);
    delay_ms(500);
    smooth_pan_to(PAN_CENTER);
    smooth_tilt_to(TILT_HORIZONTAL);
    delay_ms(500);

    send_msg("Starting nav");

    while (1) {
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        nav_step();
        delay_ms(500);
    }
}
