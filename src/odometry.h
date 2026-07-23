#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdint.h>

typedef struct {
    float x_mm;
    float y_mm;
    float theta_rad;
} RobotPose;

void Odometry_init(void);
void Odometry_update(void);
RobotPose Odometry_getPose(void);
void Odometry_reset(void);
float Odometry_getDistanceTraveled(void);
void Odometry_resetDistanceTraveled(void);

#endif /* ODOMETRY_H_ */
