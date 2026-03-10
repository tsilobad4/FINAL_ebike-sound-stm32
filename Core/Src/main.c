/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t pulse_count = 0;
float rpm_filtered = 0.0f;
uint32_t last_rpm_time_ms = 0;
uint32_t last_pulse_count = 0;
volatile float freq_out = 150.0f;   // Hz
volatile float vol_out  = 0.0f;     // 0..1
volatile float throttle_val = 0.0f; // 0.0 to 1.0 scale

// audio streaming code
// audio buffer varaibles, what the DMA will play
#define AUDIO_FS   32000.0f			   // sample rate
#define FRAMES     256                 // stereo frames (even)
int16_t i2s_tx[FRAMES * 2];            // interleaved L,R : the actual samples that the DMA sends out
volatile float phase = 0.0f;

volatile uint32_t i2s_half = 0, i2s_full = 0;
#define TWO_PI 6.283185307f

//adding variables for torque kick
float throttle_old = 0.0f;
float bite_factor = 0.0f; // This will track the "punch"

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

//the pitch being set by the RPM
static void FillAudioFrames(int startFrame, int frameCount)
{
  const float two_pi = 6.283185307f;
  float f = freq_out;
  float base_vol = vol_out;
  float t_val = throttle_val;

  if (f < 20.0f) f = 20.0f;
  if (f > 1200.0f) f = 1200.0f;

  // Persistent trackers
  static float chime_timer = 0.0f;
  static float ph_c = 0.0f, ph_e = 0.0f, ph_g = 0.0f;
  static float ph1 = 0.0f, ph2 = 0.0f, ph3 = 0.0f, ph4 = 0.0f, ph8 = 0.0f;
  static float ph_sub = 0.0f, ph1_25 = 0.0f, ph1_5 = 0.0f; // Added ph1_25 here
  static float wobble_phase = 0.0f;

  for (int n = 0; n < frameCount; n++)
  {
    float x = 0.0f;

    // --- MODE 1: STARTUP CHIME ---
    if (chime_timer < 3.0f)
    {
      chime_timer += 1.0f / AUDIO_FS;
      ph_c += 261.63f / AUDIO_FS; if (ph_c >= 1.0f) ph_c -= 1.0f;
      ph_e += 329.63f / AUDIO_FS; if (ph_e >= 1.0f) ph_e -= 1.0f;
      ph_g += 391.99f / AUDIO_FS; if (ph_g >= 1.0f) ph_g -= 1.0f;

      float c_vol = (chime_timer < 0.4f) ? (chime_timer * 2.5f) : (1.0f - ((chime_timer - 0.4f) / 2.6f));
      float shimmer = 0.85f + 0.15f * sinf(two_pi * chime_timer * 2.0f);

      x = (sinf(two_pi * ph_c) + sinf(two_pi * ph_e) + sinf(two_pi * ph_g)) * 0.33f * c_vol * shimmer;

      int16_t sample = (int16_t)(x * 25000.0f);
      int idx = (startFrame + n) * 2;
      i2s_tx[idx + 0] = sample; i2s_tx[idx + 1] = sample;
    }
    // --- MODE 2: MOTOR RUNNING ---
    else
    {
      // 1. Update EVERY phase so we can swap sounds on the fly
      ph_sub += (0.5f * f) / AUDIO_FS;  if (ph_sub >= 1.0f) ph_sub -= 1.0f;
      ph1    += f / AUDIO_FS;           if (ph1 >= 1.0f) ph1 -= 1.0f;
      ph1_25 += (1.25f * f) / AUDIO_FS; if (ph1_25 >= 1.0f) ph1_25 -= 1.0f; // Fix: Added tracking
      ph1_5  += (1.5f * f) / AUDIO_FS;  if (ph1_5 >= 1.0f) ph1_5 -= 1.0f;
      ph2    += (2.0f * f) / AUDIO_FS;  if (ph2 >= 1.0f) ph2 -= 1.0f;
      ph3    += (3.0f * f) / AUDIO_FS;  if (ph3 >= 1.0f) ph3 -= 1.0f;
      ph4    += (4.0f * f) / AUDIO_FS;  if (ph4 >= 1.0f) ph4 -= 1.0f;
      ph8    += (8.0f * f) / AUDIO_FS;  if (ph8 >= 1.0f) ph8 -= 1.0f;

      // --- OPTION D: THE CHIME-MOTOR (The Angelic Choice) ---
      float sine_mix = (sinf(two_pi * ph1) + sinf(two_pi * ph1_25) + sinf(two_pi * ph1_5)) * 0.33f;

      // The Sharpness Addition (Sawtooth)
      float saw_wave = (ph1 * 2.0f) - 1.0f;

      // Crossfade: Sine (Smooth) -> Saw (Aggressive) based on throttle
      x = (sine_mix * (1.0f - (t_val * 0.45f))) + (saw_wave * (t_val * 0.45f));

      // 3. THE WOBBLE
      wobble_phase += (5.0f + (t_val * 20.0f)) / AUDIO_FS;
      if (wobble_phase >= 1.0f) wobble_phase -= 1.0f;
      float modulation = 1.0f - (0.2f * (0.5f + 0.5f * sinf(two_pi * wobble_phase)));

      // 4. FINAL OUTPUT
      int16_t sample = (int16_t)(x * base_vol * modulation * 28000.0f);
      int idx = (startFrame + n) * 2;
      i2s_tx[idx + 0] = sample; i2s_tx[idx + 1] = sample;
    }
  }
}

/* USER CODE END 0 */
// 1 to 4 v
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

  // starting the DMA
  FillAudioFrames(0, FRAMES);


  HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)i2s_tx, FRAMES * 2);			// start streaming this bugger out of I2S2 forever using DMA



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 1. THE AUTO-RESTART WATCHDOG
	        if (HAL_I2S_GetState(&hi2s2) != HAL_I2S_STATE_BUSY_TX)
	        {
	            HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)i2s_tx, FRAMES * 2);
	        }

	        // 2. READ THROTTLE (ADC) - Do this every loop
	        HAL_ADC_Start(&hadc1);
	        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	        {
	            uint32_t adc_raw = HAL_ADC_GetValue(&hadc1);
	            float target_throttle = (float)adc_raw / 4095.0f;

	            // Calculate how FAST the throttle is moving (The "Kick")
	            float throttle_diff = target_throttle - throttle_old;
	            if (throttle_diff > 0) {
	                // If twisting UP fast, add bite (scaled by how fast you move)
	                bite_factor += throttle_diff * 2.5f;
	            }
	            throttle_old = target_throttle;

	            // Decay the bite factor back to 0 (the "settle" effect)
	            bite_factor *= 0.95f;
	            if (bite_factor > 0.5f) bite_factor = 0.5f; // Cap the boost

	            // Smooth the throttle_val normally
	            throttle_val = (0.9f * throttle_val) + (0.1f * target_throttle);

	            // Final Volume = Steady volume + the "Bite" boost
	            vol_out = 0.15f + (throttle_val * 0.45f) + bite_factor;
	            if (vol_out > 0.9f) vol_out = 0.9f; // Don't clip!
	        }
	        HAL_ADC_Stop(&hadc1);
	        // 3. BIKE RPM & PITCH CALCULATION (Every 20ms)
	        uint32_t now = HAL_GetTick();
	        if (now - last_rpm_time_ms >= 20)
	        {
	            last_rpm_time_ms = now;

	            uint32_t current = pulse_count;
	            float delta = (float)(current - last_pulse_count);
	            last_pulse_count = current;

	            // Convert pulses to RPM (46 ticks per rev)
	            float rpm_actual = (delta * 3000.0f) / 46.0f;
	            rpm_filtered = (0.92f * rpm_filtered) + (0.08f * rpm_actual);

	            // Calculate frequency based on filtered RPM PLUS the Throttle Position
	            // Adding (throttle_val * 150.0f) makes the pitch jump when you twist the grip!
	            freq_out = 20.0f + (rpm_filtered * 0.2f) + (throttle_val * 150.0f);

	            if (freq_out > 1200.0f) freq_out = 1200.0f; // Slightly higher cap for the extra rev

	            // Print status for debugging
	            printf("RPM: %d | Freq: %d Hz | Thr: %d%%\r\n",
	                   (int)rpm_filtered, (int)freq_out, (int)(throttle_val * 100));
	        }
	            HAL_Delay(1); // tiny delay so UART isn't spammed


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_INPUT_Pin */
  GPIO_InitStruct.Pin = RPM_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RPM_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 	//Every time PA4 goes from low to high, generate an interrupt
{
  if (GPIO_Pin == RPM_INPUT_Pin)
  {
    pulse_count++;
  }
}

//DMA callbacks, keeping audio continuous
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI2) {
    i2s_half++;
    FillAudioFrames(0, FRAMES/2);
  }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI2) {
    i2s_full++;
    FillAudioFrames(FRAMES/2, FRAMES/2);
  }
}

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
#ifdef USE_FULL_ASSERT
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
