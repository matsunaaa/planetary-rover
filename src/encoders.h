#ifndef ENCODERS_H_
#define ENCODERS_H_

#include <stdint.h>
#include <stdbool.h>

// Initialize encoder GPIO and interrupts
void Encoders_init(void);

// Get current counts (signed - positive = forward)
int32_t Encoders_getLeftCount(void);
int32_t Encoders_getRightCount(void);

// Reset counts to zero
void Encoders_resetCounts(void);

// Get counts since last call and reset delta
int32_t Encoders_getLeftDelta(void);
int32_t Encoders_getRightDelta(void);

#endif /* ENCODERS_H_ */
