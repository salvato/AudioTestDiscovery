//===============================================================
// To start debugging: on an external console issue
// $ openocd -f board/stm32f4discovery.cfg &
//===============================================================
// CONFIGURE_PIN
// MUST be connected to GND at all the Remote Stations
//===============================================================


#include "main.h"
#include "tm_stm32_nrf24l01.h"
#include "string.h"
#include "stdlib.h"


//===============================================================
// Used Resources
//===============================================================
//  SPI3 (I2S3)     CS43L22     (Audio OUT)
//  DMA1 CH0        CS43L22     (Audio OUT)
//  DMA1 Stream7    CS43L22     (Audio OUT)
//  SPI2 (I2S2)     MP45DT02    (Audio IN)
//  DMA1 CH0        MP45DT02    (Audio IN)
//  DMA1 Stream3    MP45DT02    (Audio IN)
//  ADC1 CH1        STM32F4     (Adc IN)
//  DMA2 CH0        STM32F4     (Adc IN)
//  DMA2 Stream0    STM32F4     (Adc IN)
//  TIM2            STM32F4     (Adc Trigger)
//  SPI1            NRF24L01    (Radio RxTx)
//===============================================================

//===============================================================
// CONFIGURE_PIN
// MUST be connected to GND at all the Remote Stations
//===============================================================
// PC1      CONFIGURE_PIN
//===============================================================

//===============================================================
// Leds Pinout
//===============================================================
// PD12     Green
// PD13     Orange
// PD14     Red
// PD15     Blue
//===============================================================

//===============================================================
// Push Button Pinout
//===============================================================
// PA0
//===============================================================

//===============================================================
// CS43L22 Pinout (use I2S3)
//===============================================================
// PA4      LRCK
// PA10     IRQ
// PB6      SCL
// PB9      SDA
// PC7      MCLK
// PC10     SCLK
// PC12     SDIN
// PD4      RESET
//===============================================================

//===============================================================
// MP45DT02 Microphone Pinout
//===============================================================
// PB10     CLK
// PC3      DOUT
//===============================================================

//===============================================================
// nRF24L01+ Pinout using SPI1 on Pins Pack 1
// See file tm_stm32_nrf24l01.h lines 127-128
// nRF24L01+   STM32Fxxx     DESCRIPTION
//===============================================================
// (1) GND       GND         Ground
// (2) VCC       3.3V        3.3V
// (3) CE        PD8         RF activated pin
// (4) CSN       PD7         Chip select pin for SPI
// (5) SCK       PA5         SCK pin for SPI1
// (6) MOSI      PA7         MOSI pin for SPI1
// (7) MISO      PA6         MISO pin for SPI1
// (8) IRQ       PA10        Interrupt pin. Goes low when active.
//===============================================================

//===============================================================
// ADC
//===============================================================
// PA1      ADC1 Channel1
//===============================================================

//===============================================================
// MUST be connected to GND at all the Remote Stations
//===============================================================
#define CONFIGURE_PIN  GPIO_PIN_1
#define CONFIGURE_PORT GPIOC

// LEDs by Colour
#define LED_GREEN   LED4
#define LED_ORANGE  LED3
#define LED_RED     LED5
#define LED_BLUE    LED6

#define TM_NRF24L01_SetRxAddress TM_NRF24L01_SetMyAddress
// nRF24L01+ Interrupt pin settings (Not set in the library file)
#define NRF24_IRQ_PORT    GPIOA
#define NRF24_IRQ_PIN     GPIO_PIN_10
#define NRF24_IRQ_CHAN    EXTI15_10_IRQn


// nRF transmission status
TM_NRF24L01_Transmit_Status_t transmissionStatus;
TM_NRF24L01_IRQ_t NRF24_IRQ;

// nRF Addresses
const uint64_t pipes[14] = {
                             0xABCDABCD71LL,
                             0x544d52687CLL,
                             0x544d526832LL,
                             0x544d52683CLL,
                             0x544d526846LL,
                             0x544d526850LL,
                             0x544d52685ALL,
                             0x544d526820LL,
                             0x544d52686ELL,
                             0x544d52684BLL,
                             0x544d526841LL,
                             0x544d526855LL,
                             0x544d52685FLL,
                             0x544d526869LL
                           };

const uint8_t nRFChannel = 76;
uint8_t isBaseStation;

TIM_HandleTypeDef  hTimLed;
TIM_OC_InitTypeDef sConfigLed;
TIM_HandleTypeDef  Tim2Handle;
GPIO_InitTypeDef   GPIO_InitStructure = {0};
ADC_HandleTypeDef  hAdc;


#define AudioFreq DEFAULT_AUDIO_IN_FREQ

__IO uint16_t  adcDataIn[2*PCM_OUT_SIZE];
uint16_t       Audio_Out_Buffer[2*2*PCM_OUT_SIZE]; // Double Buffer Stereo
uint16_t       pdmDataIn[INTERNAL_BUFF_SIZE];
uint16_t       pcmDataOut[2*PCM_OUT_SIZE];

uint8_t        Volume = 65;   // % of Max

static uint16_t indx;
static uint16_t chunk;


// Private function prototypes
static void SystemClock_Config(void);
static void InitLeds();
static void InitConfigPin();
static void InitRadio(uint8_t isBaseStation);
static void InitADC();
static void Init_TIM2_Adc(void);


int
main(void) {
    HAL_Init();
    SystemClock_Config();// Configure System Clock (168 MHz)

    InitConfigPin();
    InitLeds();

    isBaseStation = HAL_GPIO_ReadPin(CONFIGURE_PORT, CONFIGURE_PIN);
    if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, Volume, DEFAULT_AUDIO_IN_FREQ) != AUDIO_OK) {
        Error_Handler();
    }

    InitRadio(isBaseStation);

    if(isBaseStation) { // Base Station: LED_GREEN on
        BSP_LED_On(LED_GREEN);
        chunk = 0;
        InitADC(ADCx, ADCx_CHANNEL);
        Init_TIM2_Adc();
        // Enables ADC DMA request after last transfer and enables ADC peripheral
        if(HAL_ADC_Start_DMA(&hAdc, (uint32_t*)adcDataIn, 2*PCM_OUT_SIZE) != HAL_OK) {
            Error_Handler();
        }
        if(HAL_TIM_Base_Start(&Tim2Handle) != HAL_OK) {
            Error_Handler();
        }
    }

    else { // Remote Station: LED_ORANGE on
        BSP_LED_On(LED_ORANGE);
        // Configure the I2S at 1024 KHz as an input clock for MEMS microphone
        BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
        BSP_AUDIO_IN_Record((uint16_t*)&pdmDataIn[0], INTERNAL_BUFF_SIZE);
    }

// ToDo - FROM HERE:
    BSP_AUDIO_OUT_Play(&Audio_Out_Buffer[chunk*PCM_OUT_SIZE*2], 2*PCM_OUT_SIZE);
    while(1) {
    }
// ToDo - TO HERE !
}


void
InitADC() {
    hAdc.Instance = ADCx;
    HAL_ADC_MspInit(&hAdc);
    hAdc.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    hAdc.Init.Resolution            = ADC_RESOLUTION_12B;
    hAdc.Init.ScanConvMode          = DISABLE;
    hAdc.Init.ContinuousConvMode    = DISABLE;
    hAdc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hAdc.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hAdc.Init.DataAlign             = ADC_DATAALIGN_LEFT;
    hAdc.Init.NbrOfConversion       = 1;
    hAdc.Init.DMAContinuousRequests = ENABLE;
    hAdc.Init.EOCSelection          = DISABLE;
    if(HAL_ADC_Init(&hAdc) != HAL_OK) {
        Error_Handler();
    }
// Configure ADC regular channel
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel      = ADCx_CHANNEL;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    sConfig.Offset       = 0;
    if(HAL_ADC_ConfigChannel(&hAdc, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}


void
InitLeds() {
// Configure and switch OFF the board's LEDs
    BSP_LED_Init(LED_ORANGE);
    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_RED);
    BSP_LED_Init(LED_BLUE);
}


void
InitConfigPin() {
// Initialize CONFIGURE_PIN as Input with Pulldown
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStructure.Pin   = CONFIGURE_PIN;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
    HAL_GPIO_Init(CONFIGURE_PORT, &GPIO_InitStructure);
}


void
InitRadio(uint8_t isBase) {
// Initialize NRF24L01+ on channel "nRFChannel"
// and "AUDIO_BUFFER_SIZE"bytes of payload (Max 32bytes)
// By default 2Mbps data rate and 0dBm output power
// NRF24L01 goes to Rx mode by default
    TM_NRF24L01_Init(nRFChannel, PCM_OUT_SIZE);
// Set RF settings, Data rate to 2Mbps, Output power to -18dBm
    TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_2M, TM_NRF24L01_OutputPower_M18dBm);
// Set Rx & Tx Addresses, 5 bytes
    if(isBase) {
        TM_NRF24L01_SetRxAddress((uint8_t*)&pipes[0]);
        TM_NRF24L01_SetTxAddress((uint8_t*)&pipes[1]);
    }
    else {
        TM_NRF24L01_SetRxAddress((uint8_t*)&pipes[1]);
        TM_NRF24L01_SetTxAddress((uint8_t*)&pipes[0]);
    }

// Enable interrupts for nRF24L01+ IRQ pin
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Pin       = NRF24_IRQ_PIN;
    GPIO_InitStructure.Pull      = GPIO_PULLUP;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStructure.Mode      = GPIO_MODE_IT_FALLING;
    GPIO_InitStructure.Alternate = 0;
    HAL_GPIO_Init(NRF24_IRQ_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority(NRF24_IRQ_CHAN, 0x0F, 0);
    HAL_NVIC_EnableIRQ(NRF24_IRQ_CHAN);
}


static void
SystemClock_Config(void) {
// System Clock Configuration
//        The system Clock is configured as follow :
//           System Clock source            = PLL (HSE)
//           SYSCLK(Hz)                     = 168000000
//           HCLK(Hz)                       = 168000000
//           AHB Prescaler                  = 1
//           APB1 Prescaler                 = 4
//           APB2 Prescaler                 = 2
//           HSE Frequency(Hz)              = 8000000
//           PLL_M                          = 8
//           PLL_N                          = 336
//           PLL_P                          = 2
//           PLL_Q                          = 7
//           VDD(V)                         = 3.3
//           Main regulator output voltage  = Scale1 mode
//           Flash Latency(WS)              = 5
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    // Enable Power Control clock
    __HAL_RCC_PWR_CLK_ENABLE();

    // The voltage scaling allows optimizing the power consumption
    // when the device is clocked below the maximum system frequency,
    // to update the voltage scaling value regarding system frequency
    // refer to product datasheet.
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Enable HSE Oscillator and activate PLL with HSE as source
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Select PLL as system clock source and configure
    // the HCLK, PCLK1 and PCLK2 clocks dividers
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
    // on STM32F407x Revision Z devices prefetch is supported
    if (HAL_GetREVID() == 0x1001) {
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
}


// Timer2 provides periodic triggers to start ADC conversion
void
Init_TIM2_Adc(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

// TIM2 clock enable
    __HAL_RCC_TIM2_CLK_ENABLE();
// Configure the NVIC for TIM2
// Set Interrupt Group Priority
    HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);
// Enable the TIM2 global Interrupt
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

// Time base configuration
// Prescaler value to have a 400 KHz TIM2 counter clock
    const uint32_t counterClock = 1680000;
    uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock/2) / counterClock)-1;

    Tim2Handle.Instance = TIM2;

    Tim2Handle.Init.Period            = (counterClock/AudioFreq)-1;
    Tim2Handle.Init.Prescaler         = uwPrescalerValue;
    Tim2Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; // tDTS=tCK_INT
    Tim2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    Tim2Handle.Init.RepetitionCounter = 0;
    Tim2Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if(HAL_TIM_Base_Init(&Tim2Handle) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&Tim2Handle, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&Tim2Handle, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}


void
Error_Handler(void) {
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED5);
    BSP_LED_Off(LED6);
    while(1) {
        HAL_Delay(100);
        BSP_LED_Toggle(LED3);
        BSP_LED_Toggle(LED4);
        BSP_LED_Toggle(LED5);
        BSP_LED_Toggle(LED6);
    }
}


void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // @brief EXTI line detection callbacks.
    //       @param  GPIO_Pin: Specifies the pins connected EXTI line
    //       @retval None
    (void)GPIO_Pin;
/*
    if(GPIO_Pin == GPIO_PIN_0) {
        if(PbPressCheck == 0) {
            HAL_Delay(10);
            // Test on the command: Recording
            if(CmdIndex == CMD_RECORD) {
                RepeatState = REPEAT_ON;
                // Switch to Play command
                CmdIndex = CMD_PLAY;
            }
            // Test on the command: Playing
            else if(CmdIndex == CMD_PLAY) {
                // Switch to Record command
                CmdIndex = CMD_RECORD;
            }
            else {
                RepeatState = REPEAT_ON;
                // Default Command Index: Play command
                CmdIndex = CMD_PLAY;
            }
            PbPressCheck = 1;
        }
        else {
            PbPressCheck = 0;
        }
    }

    if(GPIO_Pin == GPIO_PIN_1) {
        if (PressCount == 1) {
            // Resume playing Wave status
            PauseResumeStatus = RESUME_STATUS;
            PressCount = 0;
        }
        else {
            // Pause playing Wave status
            PauseResumeStatus = PAUSE_STATUS;
            PressCount = 1;
        }
    }
*/
} 


// Interrupt handlers
void
EXTI_15_10_IRQHandler(uint16_t GPIO_Pin) {
    // Check for proper interrupt pin
    if(GPIO_Pin == NRF24_IRQ_PIN) {
        // Read interrupts
        TM_NRF24L01_Read_Interrupts(&NRF24_IRQ);

        // If data is ready on NRF24L01+
        if(NRF24_IRQ.F.DataReady) {
            // Get data from NRF24L01+
            TM_NRF24L01_GetData((uint8_t*)adcDataIn);
            // Start send
            BSP_LED_On(LED_GREEN);
            // Send it back, NRF goes automatically to TX mode
            TM_NRF24L01_Transmit((uint8_t*)adcDataIn);
            // Wait for data to be sent
            do {
                // Wait till sending
                transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
            } while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
            // Send done
            BSP_LED_Off(LED_GREEN);
            // Go back to RX mode
            TM_NRF24L01_PowerUpRx();
        }
        // Clear interrupts
        TM_NRF24L01_Clear_Interrupts();
    }
}


void
BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
//    BSP_LED_Toggle(LED_BLUE);
}


void
BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
    BSP_AUDIO_OUT_ChangeBuffer(&Audio_Out_Buffer[chunk*PCM_OUT_SIZE*2], 2*PCM_OUT_SIZE);
}


void
BSP_AUDIO_IN_TransferComplete_CallBack(void) {
//    BSP_LED_Toggle(LED_BLUE);
    // The output of the filter is a signed 16-bit value [-32768, 32767]
    BSP_AUDIO_IN_PDMToPCM(&pdmDataIn[INTERNAL_BUFF_SIZE/2], (uint16_t*)&pcmDataOut[0]);
    chunk = 1;
    for(indx=0; indx<2*PCM_OUT_SIZE; indx++) {
        Audio_Out_Buffer[2*PCM_OUT_SIZE + indx] = pcmDataOut[indx];
        if(pcmDataOut[indx] > 32768U)
            BSP_LED_Toggle(LED_RED);
    }
}


void
BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
//    BSP_LED_Toggle(LED_RED);
    // The output of the filter is a signed 16-bit value [-32768, 32767]
    BSP_AUDIO_IN_PDMToPCM(pdmDataIn, (uint16_t*)&pcmDataOut);
    chunk = 0;
    for(indx=0; indx<2*PCM_OUT_SIZE; indx++) {
        Audio_Out_Buffer[indx] = pcmDataOut[indx];
        if(pcmDataOut[indx] > 32768U)
            BSP_LED_Toggle(LED_RED);
    }
}


void
BSP_AUDIO_IN_Error_Callback(void) {
    Error_Handler();
}


// ADC Conversion complete callback
void
HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hAdc) {
    UNUSED(hAdc);
//    BSP_LED_Toggle(LED_BLUE);
    chunk = 1;
    for(indx=0; indx<PCM_OUT_SIZE; indx++) {
        Audio_Out_Buffer[(PCM_OUT_SIZE << 1) + (indx<<1)]   = adcDataIn[indx+PCM_OUT_SIZE]; // 1st Stereo Channel
        Audio_Out_Buffer[(PCM_OUT_SIZE << 1) + (indx<<1)+1] = adcDataIn[indx+PCM_OUT_SIZE]; // 2nd Stereo Channel
        if(adcDataIn[indx+PCM_OUT_SIZE] > 32768U)
            BSP_LED_Toggle(LED_RED);
    }
}


void
HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hAdc) {
    UNUSED(hAdc);
//    BSP_LED_Toggle(LED_RED);
    chunk = 0;
    for(indx=0; indx<PCM_OUT_SIZE; indx++) {
        Audio_Out_Buffer[(indx<<1)]   = adcDataIn[indx]; // 1st Stereo Channel
        Audio_Out_Buffer[(indx<<1)+1] = adcDataIn[indx]; // 2nd Stereo Channel
        if(adcDataIn[indx] > 32768U)
            BSP_LED_Toggle(LED_RED);
    }
}


void
HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc);
  Error_Handler();
}


#ifdef USE_FULL_ASSERT

void
assert_failed(uint8_t* file, uint32_t line) {
// @brief  Reports the name of the source file and the source line number
//         where the assert_param error has occurred.
//      @param  file: pointer to the source file name
//      @param  line: assert_param error line source number
//      @retval None
    // User can add his own implementation to report the file name and line number,
    // ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)

    // Infinite loop
    while(1)
    {}
}

#endif

