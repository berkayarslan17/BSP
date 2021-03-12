/* BSP.c
 *
 * BOARD SUPPORT PACKAGE for STM32G031K8 Microprocessor
 *
 *Author: Berkay Arslan
 *
 */
#include "stm32g0xx.h"
#include "bsp.h"
#include "pinmap.h"

void init_PIN(PIN currentPIN)
{
    switch(currentPIN.type)
    {
        case A:
            RCC->IOPENR |= (1U << 0);
            break;
        case B:
            RCC->IOPENR |= (2U << 0);
            break;
        case C:
            RCC->IOPENR |= (4U << 0);
            break;
        default:
            break
    }
    switch(currentPIN.state)
    {
        case INPUT:
            RCC->IOPENR |= (1U << 0);
            GPIOA->MODER &= ~(3U << 2 * currentPIN.num);
            break;
        case OUTPUT:
            RCC->IOPENR |= (2U << 0);
            GPIOA->MODER &= ~(3U << 2 * currentPIN.num);
			GPIOA->MODER |= (1U << 2 * currentPIN.num);
            break;
        default:
            break
    }
    switch(currentPIN.type)
    {
        case A:
            RCC->IOPENR |= (1U << 0);
            break;
        case B:
            RCC->IOPENR |= (2U << 0);
            break;
        case C:
            RCC->IOPENR |= (4U << 0);
            break;
        default:
            break;
    }
}