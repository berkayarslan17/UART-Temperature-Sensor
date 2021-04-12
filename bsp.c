/* BSP.c
 *
 * BOARD SUPPORT PACKAGE for STM32G031K8 Microprocessor
 *
 *Author: Berkay Arslan
 *
 */
#include "bsp.h"
#include "pinmap.h"
#include "stm32g0xx.h"
// Initialize the pins in the case of their structures.
void BSP_init_PIN(PIN currentPIN) {
  if (currentPIN.type == 'A') {
    switch (currentPIN.state) {
    case INPUT:
      RCC->IOPENR |= (1U << 0);
      GPIOA->MODER &= ~(3U << 2 * currentPIN.num);
      break;
    case OUTPUT:
      RCC->IOPENR |= (1U << 0);
      RCC->IOPENR |= (2U << 0);
      GPIOA->MODER &= ~(3U << 2 * currentPIN.num);
      GPIOA->MODER |= (1U << 2 * currentPIN.num);
      break;
    case ALTERNATE:
      RCC->IOPENR |= (1U << 0);
      GPIOA->MODER &= ~(3U << 2 * currentPIN.num);
      GPIOA->MODER |= (2U << 2 * currentPIN.num);
      break;
    case ANALOG:
      RCC->IOPENR |= (1U << 0);
      GPIOA->MODER &= ~(3U << 2 * currentPIN.num);
      GPIOA->MODER |= (3U << 2 * currentPIN.num);
      break;
    default:
      break;
    }
  }

  else if (currentPIN.type == 'B') {
    switch (currentPIN.state) {
    case INPUT:
      RCC->IOPENR |= (1U << 0);
      GPIOB->MODER &= ~(3U << 2 * currentPIN.num);
      break;
    case OUTPUT:
      RCC->IOPENR |= (2U << 0);
      GPIOB->MODER &= ~(3U << 2 * currentPIN.num);
      GPIOB->MODER |= (1U << 2 * currentPIN.num);
      break;
    case ALTERNATE:
      GPIOB->MODER &= ~(3U << 2 * currentPIN.num);
      GPIOB->MODER |= (2U << 2 * currentPIN.num);
      break;
    case ANALOG:
      GPIOB->MODER &= ~(3U << 2 * currentPIN.num);
      GPIOB->MODER |= (3U << 2 * currentPIN.num);
      break;
    default:
      break;
    }
  }

  else {
    switch (currentPIN.state) {
    case INPUT:
      RCC->IOPENR |= (1U << 0);
      GPIOC->MODER &= ~(3U << 2 * currentPIN.num);
      break;
    case OUTPUT:
      RCC->IOPENR |= (2U << 0);
      GPIOC->MODER &= ~(3U << 2 * currentPIN.num);
      GPIOC->MODER |= (1U << 2 * currentPIN.num);
      break;
    case ALTERNATE:
      GPIOC->MODER &= ~(3U << 2 * currentPIN.num);
      GPIOC->MODER |= (2U << 2 * currentPIN.num);
      break;
    case ANALOG:
      GPIOC->MODER &= ~(3U << 2 * currentPIN.num);
      GPIOC->MODER |= (3U << 2 * currentPIN.num);
      break;
    default:
      break;
    }
  }
}

// Initialize the timers in the case of their structures.
void BSP_init_TIM(TIM currentTIM) {
  if (currentTIM.APBENR_type == 1) {
    RCC->APBENR1 |= (1U << currentTIM.APBENR_num);
  }

  else /* if APBENR2 */
  {
    RCC->APBENR2 |= (1U << currentTIM.APBENR_num);
  }

  switch (currentTIM.num) {
  case 1:
    TIM1->CR1 = 0;
    TIM1->CR1 |= (1U << 7);

    TIM1->CNT = 0;
    TIM1->PSC = currentTIM.PSC;
    TIM1->ARR = currentTIM.ARR;

    TIM1->DIER |= (1U << 0);
    TIM1->CR1 |= (1U << 0);

    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, currentTIM.priority);
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    break;

  case 2:
    TIM2->CR1 = 0;
    TIM2->CR1 |= (1U << 7);

    TIM2->CNT = 0;
    TIM2->PSC = currentTIM.PSC;
    TIM2->ARR = currentTIM.ARR;

    TIM2->DIER |= (1U << 0);
    TIM2->CR1 |= (1U << 0);

    NVIC_SetPriority(TIM2_IRQn, currentTIM.priority);
    NVIC_EnableIRQ(TIM2_IRQn);
    break;

  case 3:
    TIM3->CR1 = 0;
    TIM3->CR1 |= (1U << 7);

    TIM3->CNT = 0;
    TIM3->PSC = currentTIM.PSC;
    TIM3->ARR = currentTIM.ARR;

    TIM3->DIER |= (1U << 0);
    TIM3->CR1 |= (1U << 0);
    TIM3->CR2 |= (2U << 4); // MMS REGISTER UPDATE MODE

    break;

  case 14:
    TIM14->CR1 = 0;
    TIM14->CR1 |= (1U << 7);

    TIM14->CNT = 0;
    TIM14->PSC = currentTIM.PSC;
    TIM14->ARR = currentTIM.ARR;

    TIM14->DIER |= (1U << 0);
    TIM14->CR1 |= (1U << 0);

    NVIC_SetPriority(TIM14_IRQn, currentTIM.priority);
    NVIC_EnableIRQ(TIM14_IRQn);
    break;

  case 16:
    TIM16->CR1 = 0;
    TIM16->CR1 |= (1U << 7);

    TIM16->CNT = 0;
    TIM16->PSC = currentTIM.PSC;
    TIM16->ARR = currentTIM.ARR;

    TIM16->DIER |= (1U << 0);
    TIM16->CR1 |= (1U << 0);

    NVIC_SetPriority(TIM16_IRQn, currentTIM.priority);
    NVIC_EnableIRQ(TIM16_IRQn);
    break;
  }
}
void BSP_start_adc_conversion(void) {
  ADC1->CR |= (1U << 2);
  while (!(ADC1->ISR & (1U << 2)))
    ;
}

void BSP_init_ADC(PIN currentPIN, TIM currentTIM, int priority) {
  BSP_init_PIN(currentPIN); // Choosing pin
  BSP_init_TIM(currentTIM); // Choosing timer

  RCC->APBENR2 |= (1U << 20); /* Enable ADC clock */
  ADC1->CR |= (1U << 28);     /* Enable the regulator */
  delay(100);                 /* Wait until it is initialized  */

  ADC1->CR |= (1U << 31); /* Enable ADC Calibration  */
  while (!(ADC1->ISR & (1 << 11)))
    ; /* Wait until it is initialized */

  ADC1->IER |= (1U << 2); // end of conversion interrupt enable

  ADC1->CFGR1 |= (2U << 3); // 8-bit resolution

  ADC1->CFGR1 &= ~(1U << 13); // Single conversion mode
  ADC1->CFGR1 |= (1U << 16);  // Discontinuous mode

  ADC1->CFGR1 |= (currentTIM.TRGx << 6); // pick EXTSEL
  ADC1->CFGR1 |=
      (1U << 10); // Hardware trigger detection on the rising edge in EXTEN

  ADC1->CHSELR |= (1U << currentPIN.adc_channel); // ADC channel selection (PA0)
  ADC1->CFGR1 |= (1U << 23);
  ADC1->CFGR1 |= (0U << 26);

  ADC1->CR |= (1U << 0); /* Enable ADC */
  while (!(ADC1->ISR & (1 << 0)))
    ; /* Until ADC ready */

  NVIC_SetPriority(ADC1_IRQn, priority);
  NVIC_EnableIRQ(ADC1_IRQn);
}

void USART2_IRQHandler(void) {
  uint8_t data = USART2->RDR;
  // printChar(data);
}

// Initialize the UART with PA2 and PA3 pin and concrete baud rate.
void BSP_init_UART(PIN UART_TX_PIN, PIN UART_RX_PIN, uint32_t baud) {

  BSP_init_PIN(UART_TX_PIN);
  BSP_init_PIN(UART_RX_PIN);

  GPIOA->AFR[0] &= ~(0xF << 4 * UART_TX_PIN.num);
  GPIOA->AFR[0] |= (1 << 4 * UART_TX_PIN.num);

  GPIOA->AFR[0] &= ~(0xF << 4 * UART_RX_PIN.num);
  GPIOA->AFR[0] |= (1 << 4 * UART_RX_PIN.num);

  RCC->APBENR1 |= (1U << 17);

  USART2->CR1 = 0;          // clearing control register
  USART2->CR1 |= (1U << 3); // transmitter en
  USART2->CR1 |= (1U << 2); // receiver en
  USART2->CR1 |= (1U << 5); // setting band rate
  USART2->BRR = (uint16_t)(SystemCoreClock / baud);
  USART2->CR1 |= (1U); // usart en

  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_EnableIRQ(USART2_IRQn);
}

void BSP_init_I2C(PIN SCL_PIN, PIN SDA_PIN) {
  BSP_init_PIN(SCL_PIN);
  BSP_init_PIN(SDA_PIN);

  if (SCL_PIN.type == 'A') {
    GPIOA->PUPDR |= (1U << 2 * SCL_PIN.num);
    GPIOA->OTYPER &= ~(1U << SCL_PIN.num);
    GPIOA->OTYPER |= (1U << SCL_PIN.num);

    if (SCL_PIN.num < 8) {
      GPIOA->AFR[0] &= ~(0xFU << 4 * SCL_PIN.num);
      GPIOA->AFR[0] |= (6U << 4 * SCL_PIN.num);
    } else {
      GPIOA->AFR[1] &= ~(0xFU << 4 * (SCL_PIN.num - 8));
      GPIOA->AFR[1] |= (6U << 4 * (SCL_PIN.num - 8));
    }
  }

  else {
    GPIOB->PUPDR |= (1U << 2 * SCL_PIN.num);
    GPIOB->OTYPER &= ~(1U << SCL_PIN.num);
    GPIOB->OTYPER |= (1U << SCL_PIN.num);

    if (SCL_PIN.num < 8) {
      GPIOB->AFR[0] &= ~(0xFU << 4 * SCL_PIN.num);
      GPIOB->AFR[0] |= (6U << 4 * SCL_PIN.num);
    } else {
      GPIOB->AFR[1] &= ~(0xFU << 4 * (SCL_PIN.num - 8));
      GPIOB->AFR[1] |= (6U << 4 * (SCL_PIN.num - 8));
    }
  }

  if (SDA_PIN.type == 'A') {
    GPIOA->PUPDR |= (1U << 2 * SDA_PIN.num);
    GPIOA->OTYPER &= ~(1U << SDA_PIN.num);
    GPIOA->OTYPER |= (1U << SDA_PIN.num);

    if (SDA_PIN.num < 8) {
      GPIOA->AFR[0] &= ~(0xFU << 4 * SDA_PIN.num);
      GPIOA->AFR[0] |= (6U << 4 * SDA_PIN.num);
    } else {
      GPIOA->AFR[1] &= ~(0xFU << 4 * (SDA_PIN.num - 8));
      GPIOA->AFR[1] |= (6U << 4 * (SDA_PIN.num - 8));
    }
  }

  else {
    GPIOB->PUPDR |= (1U << 2 * SDA_PIN.num);
    GPIOB->OTYPER &= ~(1U << SDA_PIN.num);
    GPIOB->OTYPER |= (1U << SDA_PIN.num);

    if (SDA_PIN.num < 8) {
      GPIOB->AFR[0] &= ~(0xFU << 4 * SDA_PIN.num);
      GPIOB->AFR[0] |= (6U << 4 * SDA_PIN.num);
    } else {
      GPIOB->AFR[1] &= ~(0xFU << 4 * (SDA_PIN.num - 8));
      GPIOB->AFR[1] |= (6U << 4 * (SDA_PIN.num - 8));
    }
  }

  // Enable I2C
  RCC->APBENR1 |= (1U << 21);
  I2C1->CR1 = 0;
  I2C1->CR1 |= (1U << 7);
  I2C1->TIMINGR |= (3 << 2 * 8);
  I2C1->TIMINGR |= (0x13 << 0);
  I2C1->TIMINGR |= (0xF << 8);
  I2C1->TIMINGR |= (0x2 << 1 * 6);
  I2C1->TIMINGR |= (0x4 << 2 * 0);
  I2C1->CR1 |= (1U << 0);
  NVIC_SetPriority(I2C1_IRQn, 1);
  NVIC_EnableIRQ(I2C1_IRQn);
}

void write_memory_I2C(uint8_t devAddr, uint16_t memAddr, uint8_t *data,
                      int size) {
  I2C1->CR2 = 0;
  I2C1->CR2 |= (uint32_t)(devAddr << 1);
  I2C1->CR2 |= (uint32_t)((size + 2) << 16);
  I2C1->CR2 |= (1U << 25); // Autoend
  I2C1->CR2 |= (1U << 13); // Generate start

  while (!(I2C1->ISR & (1 << 1)))
    ; // high address byte
  I2C1->TXDR = (uint32_t)(memAddr >> 8);

  while (!(I2C1->ISR & (1 << 1)))
    ; // low address byte
  I2C1->TXDR = (uint32_t)(memAddr & 0xFF);

  for (int i = 0; i < size; i++) {
    while (!(I2C1->ISR & (1 << 1)))
      ;
    I2C1->TXDR = (*data++); // send data
  }
}

void random_read_I2C(uint8_t devAddr, uint16_t memAddr, uint8_t *data,
                     int size) {
  uint8_t regAddrH = ((uint8_t)((0xFF00 & memAddr) >> 8));
  uint8_t regAddrL = ((uint8_t)(0x00FF & memAddr));

  I2C1->CR2 = 0;
  I2C1->CR2 |= (uint32_t)(devAddr << 1);
  I2C1->CR2 |= (2U << 16); // Number of bytes
  I2C1->CR2 |= (1U << 13); // Generate Start

  while (!(I2C1->ISR & (1 << 1)))
    ;
  I2C1->TXDR = (uint32_t)regAddrH;

  while (!(I2C1->ISR & (1 << 1)))
    ;
  I2C1->TXDR = (uint32_t)regAddrL;

  while (!(I2C1->ISR & (1 << 6)))
    ; // until the transmission complete

  I2C1->CR2 = 0;
  I2C1->CR2 |= (uint32_t)(devAddr << 1);
  I2C1->CR2 |= (1U << 10);             // Read mode
  I2C1->CR2 |= (uint32_t)(size << 16); // Number of bytes
  I2C1->CR2 |= (1U << 15);             // NACK
  I2C1->CR2 |= (1U << 25);             // AUTOEND
  I2C1->CR2 |= (1U << 13);             // Generate start

  for (int i = 0; i < size; i++) {
    while (!(I2C1->ISR & (1 << 2)))
      ;
    data[i] = (uint8_t)I2C1->RXDR;
  }
}

void printChar(uint8_t c) {
  USART2->TDR = (uint16_t)c;
  while (!(USART2->ISR & (1 << 6)))
    ;
}
// This function helps the printf function working.
int _write(int fd, char *ptr, int len) {
  (void)fd;
  for (int i = 0; i < len; i++) {
    printChar(ptr[i]);
  }
  return len;
}

void print(char *buf) {
  int len = 0;
  while (buf[len++] != '\0')
    ;
  _write(0, buf, len);
}
