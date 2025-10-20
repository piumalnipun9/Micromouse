#include "ir_sensors.h"

#include <stddef.h>

void IRSensorArray_Init(IRSensorArray *array, uint8_t window_size, uint16_t seed)
{
    if (array == NULL)
    {
        return;
    }

    array->window_size = window_size;

    for (uint8_t i = 0U; i < IR_SENSOR_COUNT; ++i)
    {
        IRSensorChannel *channel = &array->channels[i];
        IRMedianFilter_Init(&channel->filter, window_size, seed);
        channel->last_raw = seed;
        channel->last_filtered = seed;
    }
}

uint16_t IRSensorArray_Update(IRSensorArray *array, IRSensorId id, uint16_t sample)
{
    if ((array == NULL) || (id >= IR_SENSOR_COUNT))
    {
        return sample;
    }

    IRSensorChannel *channel = &array->channels[id];
    channel->last_raw = sample;
    channel->last_filtered = IRMedianFilter_Apply(&channel->filter, sample);
    return channel->last_filtered;
}

uint16_t IRSensorArray_GetFiltered(const IRSensorArray *array, IRSensorId id)
{
    if ((array == NULL) || (id >= IR_SENSOR_COUNT))
    {
        return 0U;
    }

    return array->channels[id].last_filtered;
}

uint16_t IRSensorArray_GetRaw(const IRSensorArray *array, IRSensorId id)
{
    if ((array == NULL) || (id >= IR_SENSOR_COUNT))
    {
        return 0U;
    }

    return array->channels[id].last_raw;
}
