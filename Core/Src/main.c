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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <math.h>

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

// --- THROTTLE CALIBRATION ---
// Sound starts at 1.0V
#define THROTTLE_START_SOUND_VOLTS  1.0f
// 100% sound is reached at 4.2V (gives headroom below 4.5V max)
#define THROTTLE_FULL_SOUND_VOLTS   4.2f

// Optional: Simple Moving Average Filter for smoother throttle
#define THROTTLE_FILTER_SAMPLES      10

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

////the pitch being set by the RPM
static void FillAudioFrames(int startFrame, int frameCount)
{
  const float two_pi = 6.283185307f;
  float f_motor = (freq_out < 50.0f) ? 50.0f : freq_out;
  float base_vol = vol_out;
  float t_val = throttle_val;

  static float startup_timer = 0.0f;
  const float startup_duration = 4.0f;

  static float ph_pulse = 0.0f, ph_tone = 0.0f;
  static float ph1 = 0.0f, ph1_25 = 0.0f, ph_sub1 = 0.0f, ph_sub2 = 0.0f;

  for (int n = 0; n < frameCount; n++)
  {
    float x_motor = 0.0f;
    float final_x = 0.0f;

    // --- 1. GHOST GROWL MOTOR LOGIC ---
    ph1 += f_motor / AUDIO_FS; if (ph1 >= 1.0f) ph1 -= 1.0f;
    ph1_25 += (1.25f * f_motor) / AUDIO_FS; if (ph1_25 >= 1.0f) ph1_25 -= 1.0f;

    // Sub-Harmonic Saws at 0.5x (Keep 'em deep)
    ph_sub1 += (0.50f * f_motor) / AUDIO_FS; if (ph_sub1 >= 1.0f) ph_sub1 -= 1.0f;
    ph_sub2 += (0.50f * f_motor * 1.005f) / AUDIO_FS; if (ph_sub2 >= 1.0f) ph_sub2 -= 1.0f;

    // Clean Motor: This stays at full strength now
    float sine_mix = (sinf(two_pi * ph1) + sinf(two_pi * ph1_25) * 0.4f) * 0.7f;

    // The Raw Sawtooth
    float raw_saw_growl = ((ph_sub1 * 2.0f) - 1.0f + (ph_sub2 * 2.0f) - 1.0f) * 0.5f;

    // --- THE FIX: 15% CEILING & CUBED RAMP ---
    // Cubed mapping (t^3) makes the first half of the throttle pull nearly silent
    float saw_mix = (0.01f + (t_val * t_val * t_val)) * 0.08f;

    // No more fading the clean sound! We just add the saw on top.
    x_motor = sine_mix + (raw_saw_growl * saw_mix);

    // --- 2. OPTION 1: THE PULSAR SURGE (UNCHANGED) ---
    if (startup_timer < startup_duration)
    {
      startup_timer += 1.0f / AUDIO_FS;
      float progress = startup_timer / startup_duration;

      float pulse_rate = 2.0f + (48.0f * (progress * progress));
      ph_pulse += pulse_rate / AUDIO_FS; if (ph_pulse >= 1.0f) ph_pulse -= 1.0f;

      float env = powf(sinf(two_pi * ph_pulse * 0.5f + 1.5f), 12.0f);
      float tone_f = 60.0f - (10.0f * progress);
      ph_tone += tone_f / AUDIO_FS; if (ph_tone >= 1.0f) ph_tone -= 1.0f;

      float x_startup = (sinf(two_pi * ph_tone) + sinf(two_pi * ph_tone * 0.5f) * 0.6f) * env;
      final_x = (x_startup * (1.0f - progress)) + (x_motor * progress);
    }
    else { final_x = x_motor; }

    // --- 3. MASTER OUTPUT ---
    float active_vol = (base_vol < 0.45f) ? 0.45f : base_vol;
    int16_t sample = (int16_t)(final_x * active_vol * 32000.0f);

    int idx = (startFrame + n) * 2;
    i2s_tx[idx + 0] = sample; i2s_tx[idx + 1] = sample;
  }
}

//static void FillAudioFrames(int startFrame, int frameCount)
//{
//  static uint32_t counter = 0;
//
//  for (int n = 0; n < frameCount; n++)
//  {
//    counter++;
//
//    // Toggle every 40 samples (approx 400Hz beep)
//    // We use a safe volume of 8000
//    int16_t sample = ((counter / 40) % 2) ? 8000 : -8000;
//
//    int idx = (startFrame + n) * 2;
//    i2s_tx[idx + 0] = sample;
//    i2s_tx[idx + 1] = sample;
//  }
//}

/* USER CODE END 0 */

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
//  FillAudioFrames(0, FRAMES);
//
//
//  HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)i2s_tx, FRAMES * 2);			// start streaming this bugger out of I2S2 forever using DMA

  // 1. Clear the buffer
    for(int i=0; i<FRAMES*2; i++) { i2s_tx[i] = 0; }

    // 2. Initial fill
    FillAudioFrames(0, FRAMES);

    // 3. Wait for power to stabilize
    HAL_Delay(500);

    // 4. Start the DMA
    HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)i2s_tx, FRAMES * 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  float throttle_voltage = 0.0f;

	  // 1. THE AUTO-RESTART WATCHDOG (Keep this as is)
	      if (__HAL_I2S_GET_FLAG(&hi2s2, I2S_FLAG_OVR))
	      {
	          __HAL_I2S_CLEAR_OVRFLAG(&hi2s2);
	          HAL_I2S_DMAStop(&hi2s2);
	          HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)i2s_tx, FRAMES * 2);
	      }

	      // 2. READ & CALIBRATE THROTTLE
	      HAL_ADC_Start(&hadc1);
	      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	          uint32_t adc_raw = HAL_ADC_GetValue(&hadc1);

	          // --- CALIBRATION STEP ---
	          // Convert ADC (0-4095) to Voltage (0-3.3V)
	          throttle_voltage = ((float)adc_raw / 4095.0f) * 3.3f;

	          // Map 1.0V -> 0% and 3.0V+ -> 100%
	          // (Note: Using 3.0V as the target since 4.5V from the bike
	          // will be clamped to ~3.9V by our diode, and we want 100% to be easy to hit)
	          float target_throttle = 0.0f;

	          if (throttle_voltage <= 1.0f) {
	              target_throttle = 0.0f; // Deadzone: Still at 1V
	          }
	          else if (throttle_voltage >= 3.0f) {
	              target_throttle = 1.0f; // Ceiling: Full sound at 3V (safely below the 3.9V clamp)
	          }
	          else {
	              // Math: Scale the 1.0V to 3.0V range into 0.0 to 1.0
	              target_throttle = (throttle_voltage - 1.0f) / (3.0f - 1.0f);
	          }

	          // Constrain to 0.0 - 1.0 range
	          if (target_throttle > 1.0f) target_throttle = 1.0f;
	          if (target_throttle < 0.0f) target_throttle = 0.0f;

	          // --- BITE & SMOOTHING (Your existing logic) ---
	          float throttle_diff = target_throttle - throttle_old;
	          if (throttle_diff > 0) {
	              bite_factor += throttle_diff * 2.5f;
	          }
	          throttle_old = target_throttle;
	          bite_factor *= 0.95f;
	          if (bite_factor > 0.5f) bite_factor = 0.5f;

	          // Smooth the final throttle_val (Moving toward the calibrated target)
	          throttle_val = (0.9f * throttle_val) + (0.1f * target_throttle);

	          vol_out = 0.15f + (throttle_val * 0.45f) + bite_factor;
	          if (vol_out > 0.9f) vol_out = 0.9f;
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

	        	float rpm_actual = (delta * 3000.0f) / 46.0f;
	        	rpm_filtered = (0.92f * rpm_filtered) + (0.08f * rpm_actual);

	        	// --- FIXED PITCH LOGIC ---
	        	// 0 RPM = 50Hz, 700 RPM = 200Hz
	        	freq_out = 50.0f + (rpm_filtered * 0.2142f);

	        	// Safety Cap: Don't let the motor scream past 1000Hz
	        	if (freq_out > 1000.0f) freq_out = 1000.0f;

	        	// The Serial Monitor will now show a steady Freq even when you twist the grip
	        	printf("V_raw: %d | RPM: %d | Freq: %dHz | Thr: %d%%\r\n",
	        	       (int)(throttle_voltage * 100), (int)rpm_filtered, (int)freq_out, (int)(throttle_val * 100));
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
  HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
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
