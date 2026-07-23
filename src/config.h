#ifndef CONFIG_H_
#define CONFIG_H_

// ============== RSLK PHYSICAL PARAMETERS ==============
#define WHEEL_DIAMETER_MM       70.0f
#define WHEELBASE_MM            140.0f
#define ENCODER_COUNTS_PER_REV  360     // Quadrature decoded

// Derived constants
#define WHEEL_CIRCUMFERENCE_MM  (WHEEL_DIAMETER_MM * 3.14159f)
#define MM_PER_COUNT            (WHEEL_CIRCUMFERENCE_MM / ENCODER_COUNTS_PER_REV)

// ============== MOTOR PINS (RSLK) ==============
// Left Motor
#define LEFT_PWM_PORT           P3
#define LEFT_PWM_PIN            BIT7
#define LEFT_DIR_PORT           P3
#define LEFT_DIR_PIN            BIT6
#define LEFT_SLP_PORT           P4
#define LEFT_SLP_PIN            BIT3

// Right Motor
#define RIGHT_PWM_PORT          P3
#define RIGHT_PWM_PIN           BIT5
#define RIGHT_DIR_PORT          P3
#define RIGHT_DIR_PIN           BIT4
#define RIGHT_SLP_PORT          P4
#define RIGHT_SLP_PIN           BIT1

// ============== ENCODER PINS (RSLK) ==============
// Left Encoder (Quadrature)
#define LEFT_ENC_A_PORT         P10
#define LEFT_ENC_A_PIN          BIT5
#define LEFT_ENC_B_PORT         P5
#define LEFT_ENC_B_PIN          BIT2

// Right Encoder (Quadrature)
#define RIGHT_ENC_A_PORT        P10
#define RIGHT_ENC_A_PIN         BIT4
#define RIGHT_ENC_B_PORT        P5
#define RIGHT_ENC_B_PIN         BIT0

// ============== UART/BLUETOOTH ==============
// EUSCI_A0: P1.2 (RX), P1.3 (TX) - USB debug
// EUSCI_A2: P3.2 (RX), P3.3 (TX) - Bluetooth (if using)
#define DEBUG_UART              EUSCI_A0
#define BT_UART                 EUSCI_A2

#define UART_BAUD_RATE          115200

// ============== SERVO PINS (Choose available pins) ==============
// Using Timer_A PWM outputs
#define PAN_SERVO_PORT          P2
#define PAN_SERVO_PIN           BIT4    // TA0.1
#define TILT_SERVO_PORT         P2
#define TILT_SERVO_PIN          BIT5    // TA0.2

// ============== I2C PINS (ToF + IMU) ==============
// EUSCI_B1: P6.4 (SDA), P6.5 (SCL)
#define I2C_SDA_PORT            P6
#define I2C_SDA_PIN             BIT4
#define I2C_SCL_PORT            P6
#define I2C_SCL_PIN             BIT5

// ============== SCAN PARAMETERS ==============
#define PAN_MIN_ANGLE           -45
#define PAN_MAX_ANGLE           45
#define PAN_STEP_DEGREES        5
#define TILT_MIN_ANGLE          -30
#define TILT_MAX_ANGLE          30
#define TILT_STEP_DEGREES       10
#define SCAN_INTERVAL_MM        200

// ============== SYSTEM ==============
#define SYSTEM_CLOCK_HZ         48000000
#define PWM_PERIOD              10000    // For 4.8kHz motor PWM

// ============== DEBUG ==============
#define DEBUG_ENCODERS          1
#define DEBUG_ODOMETRY          1
#define DEBUG_MOTORS            1

#endif /* CONFIG_H_ */
