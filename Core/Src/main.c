/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Function Generator (SINE, SQUARE, TRIANGLE, SAWTOOTH)
  *                   Controlled via UART (Teraterm) digits 0-3, F, A, W, D, H
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_BUFFER_SIZE       256
#define MAX_DAC_SAMPLE_RATE   9000000UL
#define MIN_ACTIVE_SAMPLES    16
#define WAVEFORM_SINE         0
#define WAVEFORM_SQUARE       1
#define WAVEFORM_TRIANGLE     2
#define WAVEFORM_SAWTOOTH     3
#define PI                    3.14159265359f
#define MIN_FREQ              100
#define MAX_FREQ              100000
#define MIN_AMP               0
#define MAX_AMP               4095
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT];
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT];

#elif defined ( __CC_ARM )

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT];
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT];

#elif defined ( __GNUC__ )

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection")));
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));
#endif

ETH_TxPacketConfig TxConfig;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;
ETH_HandleTypeDef heth;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
__attribute__((aligned(32))) uint16_t dac_buffer[DAC_BUFFER_SIZE];
static uint16_t active_samples = DAC_BUFFER_SIZE;

uint32_t current_frequency = 1000;
volatile uint8_t current_waveform = WAVEFORM_SINE;
uint16_t current_amplitude = 4095u;
uint8_t dac_started = 0;

uint8_t uart_rx_buffer[1];
char uart_cmd_buffer[64];
uint8_t uart_cmd_index = 0;

volatile uint8_t need_rebuild = 0;
volatile uint8_t need_retune  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static void UpdateActiveSampleCount(void);
void GenerateSineWave(void);
void GenerateSquareWave(void);
void GenerateTriangleWave(void);
void GenerateSawtoothWave(void);
void UpdateWaveform(void);

void StopDACOutput(void);
HAL_StatusTypeDef StartDACOutput(void);
void RebuildWaveformAndRestart(void);
void RetuneFrequencyAndRestart(void);

void ProcessCommand(char *cmd);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
static void FlushDacBufferCache(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void UpdateActiveSampleCount(void)
{
  uint32_t max_samples = MAX_DAC_SAMPLE_RATE / current_frequency;
  if (max_samples > DAC_BUFFER_SIZE) max_samples = DAC_BUFFER_SIZE;
  if (max_samples < MIN_ACTIVE_SAMPLES) max_samples = MIN_ACTIVE_SAMPLES;
  active_samples = (uint16_t)max_samples;
}

void GenerateSineWave(void)
{
  for (uint16_t i = 0; i < active_samples; ++i)
  {
    float v = sinf(2.0f * PI * i / active_samples);
    float scaled = (v + 1.0f) * 0.5f * (float)current_amplitude;
    dac_buffer[i] = (uint16_t)scaled;
  }
}

void GenerateSquareWave(void)
{
  uint16_t half = active_samples / 2U;
  for (uint16_t i = 0; i < active_samples; ++i)
  {
    dac_buffer[i] = (i < half) ? current_amplitude : 0u;
  }
}

void GenerateTriangleWave(void)
{
  uint16_t half = active_samples / 2U;
  for (uint16_t i = 0; i < active_samples; ++i)
  {
    if (i < half)
    {
      dac_buffer[i] = (uint16_t)((uint32_t)i *
                                 (uint32_t)current_amplitude * 2u /
                                 active_samples);
    }
    else
    {
      dac_buffer[i] = (uint16_t)((uint32_t)current_amplitude -
                                 ((uint32_t)(i - half) *
                                  (uint32_t)current_amplitude * 2u /
                                  active_samples));
    }
  }
}

void GenerateSawtoothWave(void)
{
  for (uint16_t i = 0; i < active_samples; ++i)
  {
    dac_buffer[i] = (uint16_t)((uint32_t)i *
                               (uint32_t)current_amplitude /
                               (active_samples - 1u));
  }
}

static void FlushDacBufferCache(void)
{
#if defined (SCB_CleanDCache_by_Addr)
  uint32_t addr = (uint32_t)dac_buffer & ~0x1FU;
  uint32_t len  = ((active_samples * sizeof(uint16_t)) + 31U) & ~31U;
  SCB_CleanDCache_by_Addr((uint32_t *)addr, len);
#endif
}

void UpdateWaveform(void)
{
  switch (current_waveform)
  {
    case WAVEFORM_SINE:     GenerateSineWave();     break;
    case WAVEFORM_SQUARE:   GenerateSquareWave();   break;
    case WAVEFORM_TRIANGLE: GenerateTriangleWave(); break;
    case WAVEFORM_SAWTOOTH: GenerateSawtoothWave(); break;
    default:
      current_waveform = WAVEFORM_SINE;
      GenerateSineWave();
      break;
  }
  FlushDacBufferCache();
}

void StopDACOutput(void)
{
  if (dac_started)
  {
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim6);
    dac_started = 0;
  }
}

HAL_StatusTypeDef StartDACOutput(void)
{
  uint64_t samples_per_sec;
  uint32_t timer_period;
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint64_t timer_clk = pclk1;

  if (current_frequency < MIN_FREQ || current_frequency > MAX_FREQ)
    return HAL_ERROR;

  samples_per_sec = (uint64_t)current_frequency * (uint64_t)active_samples;
  if (samples_per_sec == 0) return HAL_ERROR;

  if (pclk1 != SystemCoreClock)
    timer_clk = (uint64_t)pclk1 * 2ULL;

  if (timer_clk <= samples_per_sec)
    timer_period = 1;
  else
    timer_period = (uint32_t)(timer_clk / samples_per_sec) - 1;

  if (timer_period == 0)     timer_period = 1;
  if (timer_period > 0xFFFF) timer_period = 0xFFFF;

  htim6.Instance->ARR = timer_period;
  htim6.Instance->CNT = 0;
  htim6.Instance->EGR = TIM_EGR_UG;

  FlushDacBufferCache();
  __HAL_DAC_CLEAR_FLAG(&hdac, DAC_FLAG_DMAUDR1);

  if (HAL_DAC_Start_DMA(&hdac,
                        DAC_CHANNEL_1,
                        (uint32_t *)dac_buffer,
                        active_samples,
                        DAC_ALIGN_12B_R) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
  {
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    return HAL_ERROR;
  }

  dac_started = 1;
  return HAL_OK;
}

void RebuildWaveformAndRestart(void)
{
  StopDACOutput();
  UpdateActiveSampleCount();
  UpdateWaveform();
  if (StartDACOutput() != HAL_OK)
  {
    Error_Handler();
  }
}

void RetuneFrequencyAndRestart(void)
{
  StopDACOutput();
  UpdateActiveSampleCount();
  if (StartDACOutput() != HAL_OK)
  {
    Error_Handler();
  }
}

void ProcessCommand(char *cmd)
{
  if (cmd == NULL || cmd[0] == '\0') return;

  if (cmd[0] == 'F' || cmd[0] == 'f')
  {
    uint32_t f = (uint32_t)atoi(&cmd[1]);
    if (f >= MIN_FREQ && f <= MAX_FREQ)
    {
      current_frequency = f;
      need_retune = 1;

      char msg[64];
      sprintf(msg, "OK: Freq=%lu Hz\r\n", current_frequency);
      HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
    else
    {
      char msg[64];
      sprintf(msg, "ERR: Freq range %d-%d\r\n", MIN_FREQ, MAX_FREQ);
      HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
  }
  else if (cmd[0] == 'A' || cmd[0] == 'a')
  {
    uint32_t a = (uint32_t)atoi(&cmd[1]);
    if (a >= MIN_AMP && a <= MAX_AMP)
    {
      current_amplitude = (uint16_t)a;
      need_rebuild = 1;

      char msg[64];
      sprintf(msg, "OK: Amplitude=%u (0..4095)\r\n", current_amplitude);
      HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
    else
    {
      char msg[64];
      sprintf(msg, "ERR: Amplitude range %d-%d\r\n", MIN_AMP, MAX_AMP);
      HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
  }
  else if (cmd[0] == 'W' || cmd[0] == 'w')
  {
    int w = cmd[1] - '0';
    if (w >= WAVEFORM_SINE && w <= WAVEFORM_SAWTOOTH)
    {
      current_waveform = (uint8_t)w;
      need_rebuild = 1;

      const char *names[] = {"SINE","SQUARE","TRIANGLE","SAWTOOTH"};
      char msg[48];
      sprintf(msg, "OK: Wave=%s\r\n", names[current_waveform]);
      HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
    else
    {
      char msg[] = "ERR: Waveform must be 0-3\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
  }
  else if (cmd[0] >= '0' && cmd[0] <= '3')
  {
    int w = cmd[0] - '0';
    current_waveform = (uint8_t)w;
    need_rebuild = 1;

    const char *names[] = {"SINE","SQUARE","TRIANGLE","SAWTOOTH"};
    char msg[64];
    sprintf(msg, "Waveform: %s\r\n", names[current_waveform]);
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  }
  else if (cmd[0] == 'D' || cmd[0] == 'd')
  {
    const char *names[] = {"SINE","SQUARE","TRIANGLE","SAWTOOTH"};
    char msg[96];
    sprintf(msg,
            "Freq:%lu Hz | Waveform:%s | Amplitude:%u\r\n",
            current_frequency, names[current_waveform], current_amplitude);
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  }
  else if (cmd[0] == 'H' || cmd[0] == 'h')
  {
    char msg[] =
      "=== Commands ===\r\n"
      "0-3        - Select waveform (0:SINE, 1:SQUARE, 2:TRIANGLE, 3:SAWTOOTH)\r\n"
      "F<Hz>      - Set frequency (100-100000)\r\n"
      "A<0-4095>  - Set amplitude (0-4095)\r\n"
      "W<0-3>     - Change waveform\r\n"
      "D          - Display current settings\r\n"
      "H          - Help\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  (void)GPIO_Pin;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    uint8_t c = uart_rx_buffer[0];
    if (c == '\r' || c == '\n')
    {
      if (uart_cmd_index > 0)
      {
        uart_cmd_buffer[uart_cmd_index] = '\0';
        ProcessCommand(uart_cmd_buffer);
        uart_cmd_index = 0;
      }
    }
    else if (uart_cmd_index < (sizeof(uart_cmd_buffer) - 1))
    {
      uart_cmd_buffer[uart_cmd_index++] = c;
    }

    HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1);
  }
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DAC_Init();
  MX_TIM6_Init();

  UpdateActiveSampleCount();
  UpdateWaveform();
  if (StartDACOutput() != HAL_OK)
    Error_Handler();

  HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1);

  {
    char startup_msg[] =
      "Function Generator Started\r\n"
      "Type 'H' for help\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)startup_msg,
                      strlen(startup_msg), HAL_MAX_DELAY);
  }

  while (1)
  {
    if (need_rebuild)
    {
      need_rebuild = 0;
      RebuildWaveformAndRestart();
    }
    else if (need_retune)
    {
      need_retune = 0;
      RetuneFrequencyAndRestart();
    }

    HAL_Delay(1);
  }
}

/* SystemClock_Config, MX_* init, Error_Handler, assert_failed remain unchanged */
/* ... (keep the rest of CubeMX-generated code as in your file) */



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */
  __HAL_LINKDMA(&hdac, DMA_Handle1, hdma_dac1);
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 98;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
