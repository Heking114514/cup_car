#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
typedef struct { volatile uint32_t TDR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
#endif
