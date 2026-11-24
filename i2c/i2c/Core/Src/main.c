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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<math.h>
#include <string.h>
#include<stdio.h>
#define mpu_addr 0x68
#define who_am_i 0x75
#define power 0x6B
#define sample_rate 0x19
#define dlpf 0x1A
#define gyroscope_config 0x1B
#define accelerometer_config 0x1C
#define accelerometer_data 0x3B
#define gyroscope_data 0x43
uint8_t c;
uint8_t pwr=0x00;
uint8_t data;
uint8_t sampling_rate=0;
uint8_t filter=1;
uint8_t gyro_range =1;
uint8_t accel_range =(3<<3);
uint8_t gyro_config=0x18;
uint8_t accel_config = 0x18;
uint8_t tst;

uint8_t accel_buffer[6];
uint8_t gyro_buffer[6];
int16_t accel_raw_x;
int16_t accel_raw_y;
int16_t accel_raw_z;
int16_t gyro_raw_x;
int16_t gyro_raw_y;
int16_t gyro_raw_z;
float accel_x;
float accel_y;
float accel_z;
float gyro_x;
float gyro_y;
float gyro_z;
float gyro_res;
float accel_res;

float roll_accel;
float pitch_accel;
float roll_gyro;
float pitch_gyro;
float dt;
float gyro_cal_x,gyro_cal_y,gyro_cal_z;
float sum_x=0.0f;
float sum_y=0.0f;
float  sum_z=0.0f;
float gyro_offset_x,gyro_offset_y,gyro_offset_z;
float roll,pitch;
char tx_line[128];
/* USER CODE END Includes */
uint32_t current_time;
	  uint32_t previous_time = 0;
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void gyro_calibration()
{

	uint16_t c=0;
	sum_x = 0.0f;
	    sum_y = 0.0f;
	    sum_z = 0.0f;
	for(int i=1;i<=1000;i++)
	{
		HAL_I2C_Mem_Read(&hi2c1,mpu_addr<<1,gyroscope_data, 1, gyro_buffer, 6, 100);
		  gyro_raw_x=(int16_t)((gyro_buffer[0]<<8) | gyro_buffer[1]);
			  gyro_raw_y=(int16_t)((gyro_buffer[2]<<8) | gyro_buffer[3]);
			  gyro_raw_z=(int16_t)((gyro_buffer[4]<<8) | gyro_buffer[5]);
			  gyro_x=gyro_raw_x/16.4f;
			 	  gyro_y=gyro_raw_y/16.4f;
			 	  gyro_z=gyro_raw_z/16.4f;

		sum_x+=gyro_x;
		sum_y+=gyro_y;
		sum_z+=gyro_z;
		c++;
		HAL_Delay(2);
	}
	gyro_offset_x=sum_x/1000;
	gyro_offset_y=sum_y/1000;
	gyro_offset_z=sum_z/1000;
}
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

  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_I2C1_Init();
  HAL_Delay(100);
  HAL_I2C_Mem_Write(&hi2c1,mpu_addr<<1,power, 1, &pwr, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1,mpu_addr<<1,who_am_i, 1, &data, 1, 100);

  HAL_I2C_Mem_Write(&hi2c1,mpu_addr<<1,sample_rate, 1, &sampling_rate, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1,mpu_addr<<1,dlpf,1, &filter, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1,mpu_addr<<1,dlpf, 1, &tst, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1,mpu_addr<<1,gyroscope_config,1, &gyro_config, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1,mpu_addr<<1,accelerometer_config,1, &accel_config, 1, 100);
//  HAL_Delay(500);
 gyro_calibration();
//  HAL_I2C_Mem_Read(&hi2c1,mpu_addr<<1,acclerometer_data, 1, &accel_buffer, 6, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
	 HAL_I2C_Mem_Read(&hi2c1,mpu_addr<<1,accelerometer_data, 1, accel_buffer, 6, 100);
	 HAL_I2C_Mem_Read(&hi2c1,mpu_addr<<1,gyroscope_data, 1, gyro_buffer, 6, 100);
	  accel_raw_x=(int16_t)((accel_buffer[0]<<8) | accel_buffer[1]);
	  accel_raw_y=(int16_t)((accel_buffer[2]<<8) | accel_buffer[3]);
	  accel_raw_z=(int16_t)((accel_buffer[4]<<8) | accel_buffer[5]);

	  gyro_raw_x=(int16_t)((gyro_buffer[0]<<8) | gyro_buffer[1]);
	  gyro_raw_y=(int16_t)((gyro_buffer[2]<<8) | gyro_buffer[3]);
	  gyro_raw_z=(int16_t)((gyro_buffer[4]<<8) | gyro_buffer[5]);

	  accel_x=accel_raw_x/2048.0f;
	  accel_y=accel_raw_y/2048.0f;
	  accel_z=accel_raw_z/2048.0f;
	  gyro_x=gyro_raw_x/16.4f;
	  gyro_y=gyro_raw_y/16.4f;
	  gyro_z=gyro_raw_z/16.4f;
	  gyro_cal_x=gyro_x-gyro_offset_x;
	  gyro_cal_y=gyro_y-gyro_offset_y;
	  gyro_cal_z=gyro_z-gyro_offset_z;
	  accel_res=sqrtf((accel_x*accel_x)+(accel_y*accel_y )+(accel_z*accel_z));//magnitude
	  current_time = HAL_GetTick();
	     dt = (float)(current_time - previous_time) / 1000.0f;
	      previous_time = current_time;
	      if (dt <= 0.0f || dt > 0.2f) {
	          dt = 0.008f;
	      }
	  roll_accel=atan2f(accel_y,accel_z)*57.29578f;
	  pitch_accel=atan2f(-accel_x,(sqrtf(accel_y*accel_y+accel_z*accel_z)))*57.29578f;
	  roll_gyro +=gyro_cal_x*dt;
	  pitch_gyro +=gyro_cal_y*dt;

	  roll = 0.98f * (roll + gyro_cal_x * dt)
	         + 0.02f * roll_accel;

	  pitch = 0.98f * (pitch + gyro_cal_y * dt)
	          + 0.02f * pitch_accel;

//	  int n = snprintf(tx_line, sizeof(tx_line),
//	                   "{\"t\":%lu,\"ax\":%ld,\"ay\":%ld,\"az\":%ld,\"gx\":%ld,\"gy\":%ld,\"gz\":%ld}\r\n",
//	                   (unsigned long)HAL_GetTick(),
//	                   (long)(accel_x * 1000.0f), (long)(accel_y * 1000.0f), (long)(accel_z * 1000.0f),
//	                   (long)(gyro_cal_x  * 1000.0f), (long)(gyro_cal_y  * 1000.0f), (long)(gyro_cal_z  * 1000.0f));
//	          HAL_UART_Transmit(&huart2, (uint8_t *)tx_line, (uint16_t)n, 20);
	  int n = snprintf(tx_line, sizeof(tx_line),
	                       "{\"t\":%lu,\"roll\":%.2f,\"pitch\":%.2f,\"ax\":%ld,\"ay\":%ld,\"az\":%ld,\"gx\":%ld,\"gy\":%ld,\"gz\":%ld}\r\n",
	                       (unsigned long)current_time,
	                       roll, pitch,
	                       (long)(accel_x * 1000.0f), (long)(accel_y * 1000.0f), (long)(accel_z * 1000.0f),
	                       (long)(gyro_cal_x * 1000.0f), (long)(gyro_cal_y * 1000.0f), (long)(gyro_cal_z * 1000.0f));
	      HAL_UART_Transmit(&huart2, (uint8_t *)tx_line, (uint16_t)n, 20);

	  HAL_Delay(1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
