#include "ir_filter.h"

#include <stddef.h>

static uint8_t sanitise_window_size(uint8_t requested)
{
    if (requested < 3U)
    {
        requested = 3U;
    }

    if (requested > IR_MEDIAN_FILTER_MAX_WINDOW)
    {
        requested = IR_MEDIAN_FILTER_MAX_WINDOW;
    }

    if ((requested & 1U) == 0U)
    {
        requested -= 1U;
        if (requested < 3U)
        {
            requested = 3U;
        }
    }

    return requested;
}

void IRMedianFilter_Init(IRMedianFilter *filter, uint8_t window_size, uint16_t seed)
{
    if (filter == NULL)
    {
        return;
    }

    filter->window_size = sanitise_window_size(window_size);
    filter->index = 0U;
    filter->count = filter->window_size;

    for (uint8_t i = 0U; i < filter->window_size; ++i)
    {
        filter->buffer[i] = seed;
    }

    for (uint8_t i = filter->window_size; i < IR_MEDIAN_FILTER_MAX_WINDOW; ++i)
    {
        filter->buffer[i] = seed;
    }
}

uint16_t IRMedianFilter_Apply(IRMedianFilter *filter, uint16_t sample)
{
    if (filter == NULL)
    {
        return sample;
    }

    uint8_t size = (filter->window_size == 0U) ? 1U : filter->window_size;

    filter->buffer[filter->index] = sample;
    filter->index = (filter->index + 1U) % size;

    if (filter->count < size)
    {
        filter->count += 1U;
    }

    uint8_t length = (filter->count < size) ? filter->count : size;
    uint16_t sorted[IR_MEDIAN_FILTER_MAX_WINDOW];

    for (uint8_t i = 0U; i < length; ++i)
    {
        sorted[i] = filter->buffer[i];
    }

    for (uint8_t i = 1U; i < length; ++i)
    {
        uint16_t key = sorted[i];
        uint8_t j = i;
        while ((j > 0U) && (sorted[j - 1U] > key))
        {
            sorted[j] = sorted[j - 1U];
            j -= 1U;
        }
        sorted[j] = key;
    }

    uint8_t median_index = length / 2U;
    return sorted[median_index];
}
