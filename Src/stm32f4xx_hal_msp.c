#include "main.h"


/**
  * @brief ADC MSP Initialization
  *        This function configures the hardware resources used:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param hadc: ADC handle pointer
  */
void
HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
    GPIO_InitTypeDef          GPIO_InitStruct;
    static DMA_HandleTypeDef  hdma_adc;

    ADCx_CHANNEL_GPIO_CLK_ENABLE(); // Enable GPIO clock
    ADCx_CLK_ENABLE(); // ADC Periph clock enable
    DMAx_CLK_ENABLE(); // Enable DMA2 clock

    // Configure peripheral GPIO
    // ADC Channel GPIO pin configuration
    GPIO_InitStruct.Pin  = ADCx_CHANNEL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADCx_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

    // Configure the DMA streams
    // Set the parameters to be configured
    hdma_adc.Instance = ADCx_DMA_STREAM;

    hdma_adc.Init.Channel             = ADCx_DMA_CHANNEL;
    hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.MemDataAlignment    = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode                = DMA_CIRCULAR;
    hdma_adc.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_adc.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_adc.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_adc.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    hdma_adc.XferCpltCallback         = NULL;
    hdma_adc.XferHalfCpltCallback     = NULL;
    hdma_adc.XferM1HalfCpltCallback   = NULL;

    HAL_DMA_Init(&hdma_adc);
    
    // Associate the initialized DMA handle to the the ADC handle
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc);

    // Configure the NVIC for DMA
    // NVIC configuration for DMA transfer complete interrupt
    HAL_NVIC_SetPriority(ADCx_DMA_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ADCx_DMA_IRQn);
}


/**
  * @brief ADC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  */
void
HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc) {
    UNUSED(hadc);
    static DMA_HandleTypeDef  hdma_adc;

    // Reset peripherals
    ADCx_FORCE_RESET();
    ADCx_RELEASE_RESET();

    // Disable peripherals and GPIO Clocks
    // De-initialize the ADC Channel GPIO pin
    HAL_GPIO_DeInit(ADCx_CHANNEL_GPIO_PORT, ADCx_CHANNEL_PIN);

    // Disable the DMA Streams
    // De-Initialize the DMA Stream associate to transmission process
    HAL_DMA_DeInit(&hdma_adc);
    
    // Disable the NVIC for DMA
    HAL_NVIC_DisableIRQ(ADCx_DMA_IRQn);
}

