#include "main.h"

extern TIM_HandleTypeDef hTimLed;
extern I2S_HandleTypeDef hAudioOutI2s;
extern I2S_HandleTypeDef hAudioInI2s;
extern ADC_HandleTypeDef hAdc;


//*****************************************************************************
//            Cortex-M4 Processor Exceptions Handlers                         *
//*****************************************************************************

/**
 * @brief  NMI exception handler.
 */
void
NMI_Handler(void) {
}


/**
  * @brief  Hard Fault exception handler.
  */
void
HardFault_Handler(void) {
    // Go to infinite loop when Hard Fault exception occurs
    while (1) {
    }
}


/**
  * @brief  Memory Manage exception handler.
  */
void
MemManage_Handler(void) {
    // Go to infinite loop when Memory Manage exception occurs
    while (1) {
    }
}


/**
  * @brief Bus Fault exception handler.
  */
void
BusFault_Handler(void) {
    // Go to infinite loop when Bus Fault exception occurs
    while (1) {
    }
}


/**
  * @brief  Usage Fault exception handler.
  */
void
UsageFault_Handler(void) {
    // Go to infinite loop when Usage Fault exception occurs
    while (1) {
    }
}


/**
  * @brief  SVCall exception handler.
  */
void
SVC_Handler(void) {
}


/**
  * @brief Debug Monitor exception handler.
  */
void
DebugMon_Handler(void) {
}


/**
  * @brief PendSVC exception handler.
  */
void
PendSV_Handler(void) {
}


/**
  * @brief SysTick Handler.
  */
void
SysTick_Handler(void) {
    HAL_IncTick();
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s).                                              */
/******************************************************************************/

/**
  * @brief  This function handles External line 0 interrupt request.
  */
void
EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}


/**
  * @brief  This function handles External line 1 interrupt request.
  */
void
EXTI1_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}


/**
  * @brief  This function handles main I2S interrupt.
  */
void
I2S3_IRQHandler(void) {
    HAL_DMA_IRQHandler(hAudioOutI2s.hdmatx);
}


/**
  * @brief  This function handles DMA Stream interrupt request.
  */
void
I2S2_IRQHandler(void) {
    HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}


/**
* @brief  This function handles ADC DMA interrupt request.
*/
void
ADCx_DMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(hAdc.DMA_Handle);
}

