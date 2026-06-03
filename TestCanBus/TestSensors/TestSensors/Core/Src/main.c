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
#include "ens160.h"
#include "mcp2515.h"
#include <stdio.h>
#include <string.h>
#include <math.h>     /* Thêm thư viện toán học */
#include <stdlib.h>   /* Thêm thư viện chuẩn */
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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//ENS160_Data  ens_data;    /* eco2 (ppm), tvoc (ppb)*/
//AHT21_Data   aht_data;    /* temp_x10, humi_x10 */              
//uint16_t adc_gp2y  = 0;
//uint16_t adc_mq7   = 0;
//uint16_t adc_mq135 = 0;
//uint8_t can_frame0[8];
//uint8_t can_frame1[6];
//char dbg[64];            /* THEM DONG NAY */

ENS160_Data  ens_data  = {0};
AHT21_Data   aht_data  = {0};
uint16_t adc_gp2y  = 0;
uint16_t adc_mq7   = 0;
uint16_t adc_mq135 = 0;
uint8_t can_frame0[8];
uint8_t can_frame1[6];
char dbg[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* Doc ADC 3 kenh tuan tu: GP2Y(PA0), MQ7(PA1), MQ135(PA2) */
static void Read_ADC_All(void);
/* Tao ham delay micro-giay dung DWT, dung cho GP2Y timing */
static void DWT_Init(void);
static void DWT_Delay_us(uint32_t us);
/* �oc GP2Y voi dung timing pulse LED */
static uint16_t Read_GP2Y(void);
/* Dong goi du lieu vao 2 CAN frame va gui */
static void CAN_SendFrames(void);

static void UART_Print(const char *msg);   /* THEM MOI */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	DWT_Init();
	//char dbg[64];

	/* --- Init ENS160 --- */
	if (ENS160_Init() != ENS160_OK) {
			UART_Print("ENS160 INIT FAIL\r\n");
			while (1) { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(200); }
	}
	UART_Print("ENS160 INIT OK\r\n");

	/* --- Init AHT21 --- */
	if (AHT21_Init() != ENS160_OK) {
			UART_Print("AHT21 INIT FAIL\r\n");
			while (1) { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(500); }
	}
	UART_Print("AHT21 INIT OK\r\n");

	/* --- Init MCP2515 --- */
	if (MCP2515_Init() != 1) {
			UART_Print("MCP2515 INIT FAIL\r\n");
			while (1) { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(100); }
	}
	UART_Print("MCP2515 INIT OK\r\n");
	UART_Print("--- Bat dau doc cam bien ---\r\n");
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			Read_ADC_All();
			//ENS160_Read(&ens_data);
			ENS160_Status ens_status = ENS160_Read(&ens_data);
			if (ens_status == ENS160_ERROR)
			{
					UART_Print("ENS160 READ ERROR\r\n");
			}
			AHT21_Read(&aht_data);
			/* THEM: log raw de kiem tra */
			sprintf(dbg, "Temp_x10:%d  Humi_x10:%u\r\n",
							aht_data.temp_x10, aht_data.humi_x10);
			UART_Print(dbg);
			/* Log gia tri cam bien */
			sprintf(dbg, "CO2:%u ppm  TVOC:%u ppb\r\n",
							ens_data.eco2, ens_data.tvoc);
			UART_Print(dbg);

			sprintf(dbg, "Temp:%d.%d C  Humi:%d.%d%%\r\n",
							aht_data.temp_x10 / 10, aht_data.temp_x10 % 10,
							aht_data.humi_x10 / 10, aht_data.humi_x10 % 10);
			UART_Print(dbg);

			sprintf(dbg, "GP2Y:%u  MQ7:%u  MQ135:%u\r\n",
							adc_gp2y, adc_mq7, adc_mq135);
			UART_Print(dbg);

			/* Gui CAN */
			CAN_SendFrames();

			UART_Print("---\r\n");
			HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GP2Y_LED_GPIO_Port, GP2Y_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MCP2515_CS_Pin */
  GPIO_InitStruct.Pin = MCP2515_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MCP2515_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GP2Y_LED_Pin */
  GPIO_InitStruct.Pin = GP2Y_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GP2Y_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* --- UART Print --- */
	static void UART_Print(const char *msg)
	{
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
	}

	/* --- DWT --- */
	static void DWT_Init(void)
	{
			CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
			DWT->CYCCNT = 0;
			DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
	}

	static void DWT_Delay_us(uint32_t us)
	{
			uint32_t start = DWT->CYCCNT;
			uint32_t ticks = us * (SystemCoreClock / 1000000UL);
			while ((DWT->CYCCNT - start) < ticks);
	}

	/* ==================================================================== */
	/* CÁC HÀM TÍNH TOÁN CẢM BIẾN (TỪ ARDUINO CHUYỂN SANG)                  */
	/* ==================================================================== */
	
	const float CURVE_K = 2.10609f; // Tính sẵn: log(9.0/3.0) / log(1095.0/650.0)

	static float Calc_CO_Ugm3(float adc_val) {
			if (adc_val < 1.0f) adc_val = 1.0f;
			float ppm = 3.0f * pow((adc_val / 650.0f), CURVE_K);
			if (ppm < 0) ppm = 0;
			return (ppm * 1145.0f); // Trả về ug/m3
	}

	static float Calc_NO2_Ugm3(float adc_val) {
			// Linear mapping theo công thức ESP32
			float no2 = 40.0f + (adc_val - 950.0f) * (200.0f - 40.0f) / (2500.0f - 950.0f);
			if (no2 < 0) no2 = 0;
			return no2;
	}

	static float Calc_Dust_PM25(uint16_t adc_val) {
			// Tính toán cho hệ 3.3V của STM32
			float voltage = adc_val * (3.3f / 4095.0f);
			float dustDensity = (voltage - 0.0f) / 0.5f; // mg/m3
			if (dustDensity < 0) dustDensity = 0;
			return dustDensity; 
	}

	/* ==================================================================== */
	/* HÀM ĐỌC ADC CÓ AVERAGING TƯƠNG TỰ ESP32                              */
	/* ==================================================================== */

	/* Đọc 1 kênh ADC nhiều lần và lấy trung bình để khử nhiễu cho MQ */
	static uint16_t Read_ADC_Avg(uint32_t channel, int samples)
	{
			ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel      = channel;
			sConfig.Rank         = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);

			uint32_t sum = 0;
			for (int i = 0; i < samples; i++) {
					HAL_ADC_Start(&hadc1);
					if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
							sum += HAL_ADC_GetValue(&hadc1);
					}
					HAL_ADC_Stop(&hadc1);
					HAL_Delay(5); // Delay 5ms giữa các lần đọc giống ESP32
			}
			return (uint16_t)(sum / samples);
	}

	static uint16_t Read_GP2Y(void)
	{
			uint16_t adc_val = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			DWT_Delay_us(280);

			ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel      = ADC_CHANNEL_0;
			sConfig.Rank         = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
					adc_val = HAL_ADC_GetValue(&hadc1);
			}
			HAL_ADC_Stop(&hadc1);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			DWT_Delay_us(9720);
			return adc_val;	
	}

	static void Read_ADC_All(void)
	{
			// GP2Y có timing riêng, đọc 1 lần
			adc_gp2y = Read_GP2Y();

			// MQ7 (Channel 1) - Lấy trung bình 20 lần
			adc_mq7 = Read_ADC_Avg(ADC_CHANNEL_1, 20);

			// MQ135 (Channel 2) - Lấy trung bình 20 lần
			adc_mq135 = Read_ADC_Avg(ADC_CHANNEL_2, 20);
	}

	/* ==================================================================== */
	/* JSON & CAN GỬI DỮ LIỆU ĐÃ CONVERT SANG ĐƠN VỊ THỰC TẾ              */
	/* ==================================================================== */
	static void CAN_SendJson(const char *json_str) 
	{
			uint8_t totalLen = strlen(json_str);
			if (totalLen == 0 || totalLen > 250) return;

			uint8_t seq = 0;
			uint8_t offset = 0;
			uint8_t data[8];

			while (offset < totalLen) {
					memset(data, 0, 8);
					data[0] = seq;
					data[1] = totalLen;

					uint8_t chunkLen = totalLen - offset;
					if (chunkLen > 6) chunkLen = 6;

					for (uint8_t i = 0; i < chunkLen; i++) {
							data[2 + i] = (uint8_t)json_str[offset + i];
					}

					uint8_t result = MCP2515_TransmitMsg(0x555, 8, data);
					if (!result) {
							UART_Print("CAN TX JSON FAIL\r\n");
							return;
					}
					offset += chunkLen;
					seq++;
					HAL_Delay(5);
			}
			UART_Print("CAN TX JSON OK\r\n");
	}

	/* MACRO an toàn để tách số thực thành số nguyên khi dùng sprintf */
	#define FLT_INT(x)   ((int)(x))
	#define FLT_FRAC1(x) (abs((int)(((x) - (int)(x)) * 10.0f)))
	#define FLT_FRAC3(x) (abs((int)(((x) - (int)(x)) * 1000.0f)))

	static void CAN_SendFrames(void)
	{
			char json_buffer[256];

			// 1. Áp dụng công thức chuyển đổi
			float final_pm25 = Calc_Dust_PM25(adc_gp2y);
			float final_co   = Calc_CO_Ugm3((float)adc_mq7);
			float final_no2  = Calc_NO2_Ugm3((float)adc_mq135);
			float battery_val = 85.0f; // Giá trị Pin mặc định 85.0%

			// 2. Build chuỗi JSON (Bổ sung key "battery" lên đầu chuỗi)
			snprintf(json_buffer, sizeof(json_buffer),
					"{\"battery\":%d.%d,\"pm25\":%d.%03d,\"co\":%d.%d,\"no2\":%d.%d,\"co2\":%u,\"tvoc\":%u,\"temp\":%d.%d,\"hum\":%d.%d}",
					FLT_INT(battery_val), FLT_FRAC1(battery_val),
					FLT_INT(final_pm25),  FLT_FRAC3(final_pm25),
					FLT_INT(final_co),    FLT_FRAC1(final_co),
					FLT_INT(final_no2),   FLT_FRAC1(final_no2),
					ens_data.eco2,
					ens_data.tvoc,
					aht_data.temp_x10 / 10, aht_data.temp_x10 % 10,
					aht_data.humi_x10 / 10, aht_data.humi_x10 % 10
			);

			// In ra Serial để theo dõi
			//sprintf(dbg, "JSON Data: %s\r\n", json_buffer);
			//UART_Print(dbg);
			UART_Print("JSON Data: ");
			UART_Print(json_buffer);
			UART_Print("\r\n");

			// 3. Gửi qua CAN
			CAN_SendJson(json_buffer);
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
