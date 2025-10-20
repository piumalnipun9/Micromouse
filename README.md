# Micromouse

STM32 configuration files for micromouse project

## Core Modules

- `Core/Inc/velocity_profile.h` and `Core/Src/velocity_profile.c`: trapezoidal and triangular velocity profile planning helpers for drivetrain control.
- `Core/Inc/ir_filter.h` and `Core/Src/ir_filter.c`: reusable median filter tuned for smoothing IR sensor ADC readings.
- `Core/Inc/ir_sensors.h` and `Core/Src/ir_sensors.c`: convenience wrappers that associate each IR channel with its median filter state.
- `Core/Src/main.c`: HAL-based entry point wiring the velocity profile planner, IR sensor sampling, and motor PWM outputs.
