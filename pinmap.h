/*
 * pinmap.h
 *
 * 
 * Author: Berkay Arslan
 */

#ifndef PINMAP_H_
#define PINMAP_H_

#include <stdio.h>

enum state_type { INPUT, OUTPUT, ALTERNATE, ANALOG };

typedef struct PIN_STRUCT{
    char const type;
    int state;
    int num;
    int adc_channel;
} PIN;

typedef struct TIM_STRUCT{
    int num;
    int TRGx;
    int APBENR_type;
    int APBENR_num;
    int priority;
    int PSC;
    int ARR;
} TIM;

#endif /* PINMAP_H_ */
