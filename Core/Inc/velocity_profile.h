#ifndef VELOCITY_PROFILE_H
#define VELOCITY_PROFILE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float max_velocity;
    float max_acceleration;
    float max_deceleration;
} MotionConstraints;

typedef struct {
    MotionConstraints constraints;
    float target_distance;
    float direction;
    float peak_velocity;
    float accel_time;
    float cruise_time;
    float decel_time;
    float total_time;
    float accel_distance;
    float cruise_distance;
    float decel_distance;
} VelocityProfile;

void VelocityProfile_Init(VelocityProfile *profile, MotionConstraints constraints);

void VelocityProfile_Plan(VelocityProfile *profile, float distance);

void VelocityProfile_Sample(const VelocityProfile *profile, float time, float *position_out, float *velocity_out);

uint8_t VelocityProfile_IsComplete(const VelocityProfile *profile, float time);

#ifdef __cplusplus
}
#endif

#endif /* VELOCITY_PROFILE_H */
