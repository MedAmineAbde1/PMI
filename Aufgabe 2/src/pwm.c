#include <uart.h>
#include <stdio.h>
#include <stm32l0xx.h>
#include <stdarg.h>

void pwmen(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN ;


    //GPIOA -> 
}