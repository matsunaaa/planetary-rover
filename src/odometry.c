#include "odometry.h"
#include "encoders.h"
#include "config.h"
#include <math.h>

static RobotPose pose = {0.0f, 0.0f, 0.0f};
static float distance_traveled = 0.0f;

void Odometry_init(void)
{
    Encoders_init();
    Odometry_reset();
}

void Odometry_reset(void)
{
    pose.x_mm = 0.0f;
    pose.y_mm = 0.0f;
    pose.theta_rad = 0.0f;
    distance_traveled = 0.0f;
    Encoders_resetCounts();
}

void Odometry_update(void)
{
    int32_t left_delta = Encoders_getLeftDelta();
    int32_t right_delta = Encoders_getRightDelta();

    float left_mm = left_delta * MM_PER_COUNT;
    float right_mm = right_delta * MM_PER_COUNT;

    float center_mm = (left_mm + right_mm) / 2.0f;
    float delta_theta = (right_mm - left_mm) / WHEELBASE_MM;

    float mid_theta = pose.theta_rad + delta_theta / 2.0f;

    pose.x_mm += center_mm * cosf(mid_theta);
    pose.y_mm += center_mm * sinf(mid_theta);
    pose.theta_rad += delta_theta;

    while (pose.theta_rad > 3.14159f)  pose.theta_rad -= 2.0f * 3.14159f;
    while (pose.theta_rad < -3.14159f) pose.theta_rad += 2.0f * 3.14159f;

    distance_traveled += fabsf(center_mm);
}

RobotPose Odometry_getPose(void)
{
    return pose;
}

float Odometry_getDistanceTraveled(void)
{
    return distance_traveled;
}

void Odometry_resetDistanceTraveled(void)
{
    distance_traveled = 0.0f;
}
