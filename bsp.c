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
    if(currentPIN.type == 'A')
    {
        switch(currentPIN.state)
        {
            case INPUT:
                RCC->IOPENR |= (1U << 0);
                GPIOA->MODER &= ~(3U << 2 * currentPIN.num);
                break;
            case OUTPUT:
                RCC->IOPENR |= (2U << 0);
                GPIOA->MODER &= ~(3U << 2 * currentPIN.num);
                GPIOA->MODER |= (1U << 2  * currentPIN.num);
                break;
            case ALTERNATE:
                GPIOA->MODER &= ~(3U << 2 * currentPIN.num);
                GPIOA->MODER |= (2U << 2  * currentPIN.num);
                break;
            case ANALOG:
                GPIOA->MODER &= ~(3U << 2 * currentPIN.num);
                GPIOA->MODER |= (3U << 2  * currentPIN.num);
                break;
            default:
                break;
        }
    }

    else if(currentPIN.type == 'B')
    {
        switch(currentPIN.state)
        {
            case INPUT:
                RCC->IOPENR |= (1U << 0);
                GPIOB->MODER &= ~(3U << 2 * currentPIN.num);
                break;
            case OUTPUT:
                RCC->IOPENR |= (2U << 0);
                GPIOB->MODER &= ~(3U << 2 * currentPIN.num);
                GPIOB->MODER |= (1U << 2  * currentPIN.num);
                break;
            case ALTERNATE:
                GPIOB->MODER &= ~(3U << 2 * currentPIN.num);
                GPIOB->MODER |= (2U << 2  * currentPIN.num);
                break;
            case ANALOG:
                GPIOB->MODER &= ~(3U << 2 * currentPIN.num);
                GPIOB->MODER |= (3U << 2  * currentPIN.num);
                break;
            default:
                break;
        }
    }

    else
    {
        switch(currentPIN.state)
        {
            case INPUT:
                RCC->IOPENR |= (1U << 0);
                GPIOC->MODER &= ~(3U << 2 * currentPIN.num);
                break;
            case OUTPUT:
                RCC->IOPENR |= (2U << 0);
                GPIOC->MODER &= ~(3U << 2 * currentPIN.num);
                GPIOC->MODER |= (1U << 2  * currentPIN.num);
                break;
            case ALTERNATE:
                GPIOC->MODER &= ~(3U << 2 * currentPIN.num);
                GPIOC->MODER |= (2U << 2  * currentPIN.num);
                break;
            case ANALOG:
                GPIOC->MODER &= ~(3U << 2 * currentPIN.num);
                GPIOC->MODER |= (3U << 2  * currentPIN.num);
                break;
            default:
                break;
        }
    }
}
