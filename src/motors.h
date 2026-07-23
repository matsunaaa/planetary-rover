#ifndef MOTORS_H_
#define MOTORS_H_

#include <stdint.h>
#include <stdbool.h>

// Initialize motor GPIO and PWM
void Motors_init(void);

// Enable/disable motor driver (sleep pin)
void Motors_enable(void);
void Motors_disable(void);

// Set individual motor speeds (-10000 to +10000)
// Positive = forward, Negative = backward
void Motors_setLeft(int16_t speed);
void Motors_setRight(int16_t speed);

// Convenience functions
void Motors_forward(uint16_t speed);
void Motors_backward(uint16_t speed);
void Motors_turnLeft(uint16_t speed);
void Motors_turnRight(uint16_t speed);
void Motors_stop(void);

#endif /* MOTORS_H_ */
