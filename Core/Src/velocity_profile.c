#include "velocity_profile.h"

#include <math.h>
#include <stddef.h>

#ifndef FLT_EPSILON
#define FLT_EPSILON (1.19209290e-7F)
#endif

static float clamp_positive(float value, float fallback)
{
    return (value > FLT_EPSILON) ? value : fallback;
}

void VelocityProfile_Init(VelocityProfile *profile, MotionConstraints constraints)
{
    if (profile == NULL)
    {
        return;
    }

    profile->constraints = constraints;
    profile->target_distance = 0.0f;
    profile->direction = 1.0f;
    profile->peak_velocity = 0.0f;
    profile->accel_time = 0.0f;
    profile->cruise_time = 0.0f;
    profile->decel_time = 0.0f;
    profile->total_time = 0.0f;
    profile->accel_distance = 0.0f;
    profile->cruise_distance = 0.0f;
    profile->decel_distance = 0.0f;
}

void VelocityProfile_Plan(VelocityProfile *profile, float distance)
{
    if (profile == NULL)
    {
        return;
    }

    profile->direction = (distance >= 0.0f) ? 1.0f : -1.0f;
    profile->target_distance = (distance >= 0.0f) ? distance : -distance;

    MotionConstraints c = profile->constraints;
    float vmax = clamp_positive(c.max_velocity, 0.0f);
    float accel = clamp_positive(c.max_acceleration, FLT_EPSILON);
    float decel = clamp_positive(c.max_deceleration, FLT_EPSILON);

    if (profile->target_distance <= FLT_EPSILON || vmax <= FLT_EPSILON)
    {
        profile->peak_velocity = 0.0f;
        profile->accel_time = 0.0f;
        profile->cruise_time = 0.0f;
        profile->decel_time = 0.0f;
        profile->total_time = 0.0f;
        profile->accel_distance = 0.0f;
        profile->cruise_distance = 0.0f;
        profile->decel_distance = 0.0f;
        return;
    }

    float accel_distance = (vmax * vmax) / (2.0f * accel);
    float decel_distance = (vmax * vmax) / (2.0f * decel);

    if ((accel_distance + decel_distance) >= profile->target_distance)
    {
        /* Triangular profile: peak velocity is limited by distance. */
        float numerator = 2.0f * profile->target_distance * accel * decel;
        float denom = accel + decel;
        float peak_v = sqrtf(numerator / denom);

        profile->peak_velocity = peak_v;
        profile->accel_time = peak_v / accel;
        profile->decel_time = peak_v / decel;
        profile->cruise_time = 0.0f;
        profile->accel_distance = (peak_v * peak_v) / (2.0f * accel);
        profile->decel_distance = (peak_v * peak_v) / (2.0f * decel);
        profile->cruise_distance = 0.0f;
    }
    else
    {
        /* Trapezoidal profile: reach max velocity and maintain cruise. */
        float cruise_distance = profile->target_distance - (accel_distance + decel_distance);

        profile->peak_velocity = vmax;
        profile->accel_time = vmax / accel;
        profile->decel_time = vmax / decel;
        profile->cruise_time = cruise_distance / vmax;
        profile->accel_distance = accel_distance;
        profile->cruise_distance = cruise_distance;
        profile->decel_distance = decel_distance;
    }

    profile->total_time = profile->accel_time + profile->cruise_time + profile->decel_time;
}

void VelocityProfile_Sample(const VelocityProfile *profile, float time, float *position_out, float *velocity_out)
{
    if (profile == NULL)
    {
        return;
    }

    float t = (time >= 0.0f) ? time : 0.0f;
    float accel = clamp_positive(profile->constraints.max_acceleration, FLT_EPSILON);
    float decel = clamp_positive(profile->constraints.max_deceleration, FLT_EPSILON);

    float pos = 0.0f;
    float vel = 0.0f;

    if (t <= profile->accel_time)
    {
        vel = accel * t;
        pos = 0.5f * accel * t * t;
    }
    else if (t <= (profile->accel_time + profile->cruise_time))
    {
        float t_cruise = t - profile->accel_time;
        vel = profile->peak_velocity;
        pos = profile->accel_distance + profile->peak_velocity * t_cruise;
    }
    else if (t <= profile->total_time)
    {
        float t_decel = t - (profile->accel_time + profile->cruise_time);
        vel = profile->peak_velocity - decel * t_decel;
        if (vel < 0.0f)
        {
            vel = 0.0f;
        }
        pos = profile->accel_distance + profile->cruise_distance + (profile->peak_velocity * t_decel) - (0.5f * decel * t_decel * t_decel);
    }
    else
    {
        pos = profile->target_distance;
        vel = 0.0f;
    }

    pos *= profile->direction;
    vel *= profile->direction;

    if (position_out != NULL)
    {
        *position_out = pos;
    }

    if (velocity_out != NULL)
    {
        *velocity_out = vel;
    }
}

uint8_t VelocityProfile_IsComplete(const VelocityProfile *profile, float time)
{
    if (profile == NULL)
    {
        return 1U;
    }

    return (time >= profile->total_time) ? 1U : 0U;
}
