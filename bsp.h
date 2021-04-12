/*
 * bsp.h
 *
 * 
 * Author: Berkay Arslan
 */

#ifndef BSP_H_
#define BSP_H_

#include "stm32g0xx.h"
#include "pinmap.h"

void BSP_init_PIN(PIN);
void BSP_init_TIM(TIM);
void BSP_init_ADC(PIN, TIM, int);
void BSP_start_adc_conversion(void);
void BSP_init_UART(PIN, PIN, uint32_t);
void BSP_init_I2C(PIN, PIN);
void write_memory_I2C(uint8_t, uint16_t,uint8_t*, int);
void random_read_I2C(uint8_t, uint16_t, uint8_t*, int);
int _write(int fd, char *ptr, int len);
void printChar(uint8_t);
void print(char*);

#endif /* BSP_H_ */
