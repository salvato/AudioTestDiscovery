#ifndef __MAIN_H
#define __MAIN_H


#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"
#include "stm32f4xx_it.h"

// Defines for LEDs lighting
#define LED3_TOGGLE      0x03
#define LED4_TOGGLE      0x04
#define LED6_TOGGLE      0x06
#define LEDS_OFF         0x07
#define STOP_TOGGLE      0x00


// Definition for ADCx's Channel
#define ADCx                            ADC1
#define ADCx_CHANNEL                    ADC_CHANNEL_1

// Definition for ADCx Channel Pin
#define ADCx_CHANNEL_PIN                GPIO_PIN_1
#define ADCx_CHANNEL_GPIO_PORT          GPIOA

// Definition for ADCx clock resources
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()               __HAL_RCC_DMA2_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

// Definition for ADCx's DMA
#define ADCx_DMA_CHANNEL                DMA_CHANNEL_0
#define ADCx_DMA_STREAM                 DMA2_Stream0

// Definition for ADCx's NVIC
#define ADCx_DMA_IRQn                   DMA2_Stream0_IRQn
#define ADCx_DMA_IRQHandler             DMA2_Stream0_IRQHandler


// Definition for TIMx clock resources
//#define TIMx                            TIM2
//#define TIMx_CLK_ENABLE()               __HAL_RCC_TIM2_CLK_ENABLE()
//#define TIMx_IRQn                       TIM2_IRQn
//#define TIMx_IRQHandler                 TIM2_IRQHandler

void Error_Handler(void);

#endif //__MAIN_H
