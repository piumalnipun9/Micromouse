#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <stdint.h>

#include "ir_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IR_SENSOR_FRONT_LEFT = 0,
    IR_SENSOR_FRONT_RIGHT,
    IR_SENSOR_LEFT,
    IR_SENSOR_RIGHT,
    IR_SENSOR_DIAGONAL_LEFT,
    IR_SENSOR_DIAGONAL_RIGHT,
    IR_SENSOR_COUNT
} IRSensorId;

typedef struct {
    IRMedianFilter filter;
    uint16_t last_raw;
    uint16_t last_filtered;
} IRSensorChannel;

typedef struct {
    IRSensorChannel channels[IR_SENSOR_COUNT];
    uint8_t window_size;
} IRSensorArray;

void IRSensorArray_Init(IRSensorArray *array, uint8_t window_size, uint16_t seed);

uint16_t IRSensorArray_Update(IRSensorArray *array, IRSensorId id, uint16_t sample);

uint16_t IRSensorArray_GetFiltered(const IRSensorArray *array, IRSensorId id);

uint16_t IRSensorArray_GetRaw(const IRSensorArray *array, IRSensorId id);

#ifdef __cplusplus
}
#endif

#endif /* IR_SENSORS_H */
