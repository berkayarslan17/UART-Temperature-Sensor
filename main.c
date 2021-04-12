/*
 * main.c
 *
 *
 * Author: Berkay Arslan
 */

#include "bsp.h"
#include "pinmap.h"
#include "stm32g0xx.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define SIZE 64

//  *************** Global Variables ***************
// ADC Variables
unsigned int adc_result, k, i;
// value of R on board
const float R1 = 3000;
// For converting voltage to temperature value
float logR2, R2, T, temp_val, filtered_temp_val;
const float c1 = 0.001129148, c2 = 0.000234125,
            c3 = 0.0000000876741; // steinhart-hart coefficients for thermistor
// raw and filtered data
float u[SIZE], x[SIZE];

// UART variables
_Bool write_flag = 1;
_Bool start_flag = 1;
uint8_t message_val, keep_data;
char message[SIZE];
char listener[] = "AT+TEMP?";
int result;

// Structures
struct Queue {
  float data[SIZE];
  unsigned int head;
  unsigned int size;
};

// Declare Structures
struct Queue temp_queue;

// *************** Declare Functions ***************
unsigned int get_adc_result(void);
float calculate_temperature(unsigned int adc_result);
float filtered_temperature(float raw_temperature);
void init_queue(void);
void queue_enqueue(float value);
void _printChar(uint8_t);
void uart_tx(uint8_t);
uint8_t uart_rx(void);

// *************** Initialize the pins and timers ***************

PIN PA_0 = {
    .type = 'A',
    .state = ANALOG,
    .num = 0,
    .adc_channel = 0,
};

PIN PA_2 = {
    .type = 'A',
    .state = ALTERNATE,
    .num = 2,
    .adc_channel = 1,
};

PIN PA_3 = {
    .type = 'A',
    .state = ALTERNATE,
    .num = 3,
    .adc_channel = 1,
};

TIM TIM_3 = {
    .num = 3,
    .TRGx = 3,
    .APBENR_type = 1,
    .APBENR_num = 1,
    .priority = 0,
    .PSC = 0,
    .ARR = 1600000, // Sample with 10Hz
};

// Get the analog result from ADC data register.
unsigned int get_adc_result(void) { return (uint8_t)(ADC1->DR); }

// Convert the voltage value to celsius in terms of using Steinhart-Hart
// equation.
float calculate_temperature(unsigned int adc_result) {
  R2 = R1 * (1023.0 / (float)adc_result - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  T = T - 273.15;
  return T;
}
// Filter the raw data and return the filtered data.
float filtered_temperature(float raw_temperature) {
  // Initialize the first value with raw temperature
  if (k == 0) {
    x[k] = raw_temperature;
  }
  if (k >= SIZE) {
    k = 0;
  } else
    u[k] = raw_temperature;
  x[k + 1] = 0.75 * x[k] + 0.25 * u[k];
  k++;
  return x[k];
}
// Initialize the temp_queue.
void init_queue(void) {
  temp_queue.head = 0;
  temp_queue.size = 64;
}
// Enqueue the data with using LIFO method.
void queue_enqueue(float value) {
  temp_queue.data[temp_queue.head] = value;
  temp_queue.head = (temp_queue.head + 1) % temp_queue.size;
}
// Print a character to console.
void _printChar(uint8_t letter) {
  USART2->TDR = (uint16_t)letter;
  keep_data = letter;
  while (!(USART2->ISR & (1 << 6)))
    ;
}
// This function lets the user to write on UART console.
void uart_tx(uint8_t c) {
  while (keep_data != c)
    _printChar(c);
}
// Recieve the data that has been written on the UART console.
uint8_t uart_rx(void) {
  uint8_t data = (uint8_t)USART2->RDR;
  return data;
}
// Samples the data with 10Hz.
void ADC_COMP_IRQHandler(void) {
  // Get analog results from A0 pin
  adc_result = get_adc_result();

  // Convert the voltage to temperature by using steinhart-hart equation
  temp_val = calculate_temperature(adc_result);

  // Filter the temperature data
  filtered_temp_val = filtered_temperature(temp_val);

  // Save the data to memory with LIFO method
  queue_enqueue(filtered_temp_val);
}

int main(void) {
  // Initialize the system.
  BSP_init_UART(PA_2, PA_3, 9600);
  BSP_init_ADC(PA_0, TIM_3, 1);
  BSP_start_adc_conversion();
  init_queue();

  while (1) {
    // Writing Mode
    if (write_flag == 1) {
      // transmit the data to console
      uart_tx(uart_rx());
      message_val = uart_rx();
      // At first, save the message in first index for the sake of my algorithm.
      if (start_flag == 1 && message[i] != message_val) {
        message[i] = message_val;
        start_flag = 0;
      }
      // If start flag is closed, then save the values to the other indexes.
      else {
        // When you write something, save the message to the array.
        if (i < SIZE && message[i] != message_val) {
          i++;
          message[i] = message_val;

          // If the enter key is pressed, turn the array to string
          if (message[i] == 13) {
            message[i++] = 0;
            i--;
            start_flag = 1; // Open the start flag for writing "+Temp:" at the
                            // begin of the line in the Reading Mode.
            write_flag = 0; // Don't write to the console anymore.
          }
        }
      }
    }
    // Reading Mode
    else {
      result = strcmp(message, listener);
      // If the "AT+TEMP?" text is written, show the temperature data
      if (result == 0) {
        // Add "+TEMP:" in the beginning.
        if (start_flag == 1) {
          printf("\r\n+TEMP: ");
          start_flag = 0;
          i = 0; // Reset the index counter.
        }
        // Print the queue data on the UART console.
        while (i < SIZE && start_flag == 0) {
          printf("%.1f, ", temp_queue.data[i]);
          i++;

          if (i == SIZE) {
            printf("\r\n");
            write_flag = 1; // When it has the max value, give permission for
                            // writing to console.
            start_flag = 1;
            i = 0;
          }
        }
      }
    }
  }
  return 0;
}
