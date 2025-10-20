#ifndef IR_FILTER_H
#define IR_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IR_MEDIAN_FILTER_MAX_WINDOW 9U

typedef struct {
    uint16_t buffer[IR_MEDIAN_FILTER_MAX_WINDOW];
    uint8_t index;
    uint8_t count;
    uint8_t window_size;
} IRMedianFilter;

void IRMedianFilter_Init(IRMedianFilter *filter, uint8_t window_size, uint16_t seed);

uint16_t IRMedianFilter_Apply(IRMedianFilter *filter, uint16_t sample);

#ifdef __cplusplus
}
#endif

#endif /* IR_FILTER_H */
