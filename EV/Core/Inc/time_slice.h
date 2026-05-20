#ifndef __TIME_SLICE_H__
#define __TIME_SLICE_H__

#include "main.h"
#include "SM.h"
#include "PS2.h"
#include "font.h"
#include "oled.h"
#include "math.h"

typedef struct {
    uint32_t in_interval;
    uint32_t in_last_time;
    void(*in_callback)(void);
} time_slice;

void time_slice_Init(time_slice* poll, uint32_t interval, void(*callback)(void));
void time_slice_run(time_slice* poll);
void PS2_collect_callback(void);
void OLED_callback(void);

#endif 


