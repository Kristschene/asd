
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "tsl_user.h"
#include "tsl_conf.h"
#include "io_def.h"
#include "led.h"
#include "power.h"
#include "printer.h"
#include "rtc.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
#define LINEAR_DETECT (MyTKeysB[0].p_Data->StateId == TSL_STATEID_DETECT)
/* Private variables ---------------------------------------------------------*/

TSC_HandleTypeDef htsc;
enum eState state = eAwake;
uint32_t maintimer=0; // timer in 1 sek steps

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_ConfigHigh(void);
void SystemClock_ConfigLow(void);
static void MX_TSC_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  //__HAL_RCC_RTC_ENABLE();


  SystemClock_ConfigHigh();

  // Init RTC
  rtc_init();

  while(1)
  {
	  RTC_TimeTypeDef local_time = {0};
	  HAL_Delay(10000);
	  local_time =  rtc_get_time();
	  HAL_Delay(3000);
	  local_time =  rtc_get_time();
  }

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */


  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  char FirstRun = 1;

	  // STATE MACHINE
	  switch(state)
	  {


	  case eAwake:
		  maintimer = 0;
		  power_init();
		  tp_power_on();
		  led_init();
		  MX_TSC_Init();
		  tsl_user_Init();

		  led_enable(eSteady,eGreen);

		  while(LINEAR_DETECT==0)
		  {
			  tsl_user_Exec();
			  if(maintimer>= 20)
			  {

				  break;
			  }
		  }

		  if(maintimer>=20) state = eSleep;
		  else state = ePrint;


		  break;

	  case ePrint:
		  printer_init();

		  mot_power_on();

		  led_enable(eFlash,eGreen);

		  printer_print_message(" 03-09-2018 ");


		  while(printer_get_printer_status()!=0)
		  {
			  // Printer is busy
		  }
		  mot_power_on();

		  printer_drive_to_next_label();

		  mot_power_off();

		  state = eAwake;
		  break;


	  case eSleep:
		  tsl_user_Init();
		  maintimer = 5;


		  // testing
		  led_enable(eSteady,eRed);

		  while(LINEAR_DETECT==0)
		  {
			  tsl_user_Exec();
			  if(maintimer>=5)
			  {
				  if(FirstRun==1)
				  {
					  power_deact_all_periph_for_sleep();
					  HAL_SuspendTick();
					  HAL_RCC_DeInit();
					  SystemClock_ConfigLow();
					  HAL_SuspendTick();

					  //Enable Low Power Mode
					  HAL_PWREx_EnableLowPowerRunMode();

					  //MX_TSC_Init();
					  tsl_user_Init();
					  FirstRun = 0;
				  }

				  TSL_tim_ProcessIT();

			  }
		  }

		  // Disable Low Power Mode
		  HAL_PWREx_DisableLowPowerRunMode();
		  HAL_SuspendTick();
		  HAL_RCC_DeInit();
		  SystemClock_ConfigHigh();

		  state = eAwake;
	  		  break;


	  default:
		  led_enable(eSteady,eRed);
		  break;

	  }
  }
}


void MAIN_TimerIT(void)
{
	static int cnt=0;
	static enum eState eOldState;

	cnt++;
	if(cnt>=1000)
	{
		maintimer++;
		cnt = 0;
	}

	if(eOldState!=state) maintimer = 0;

	eOldState = state;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_ConfigHigh(void)
{

	  RCC_OscInitTypeDef RCC_OscInitStruct;
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;
	  RCC_PeriphCLKInitTypeDef PeriphClkInit;

	    /**Configure the main internal regulator output voltage
	    */
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	    /**Configure LSE Drive Capability
	    */
	  HAL_PWR_EnableBkUpAccess();

	  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	    /**Initializes the CPU, AHB and APB busses clocks
	    */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  RCC_OscInitStruct.HSICalibrationValue = 16;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	    /**Initializes the CPU, AHB and APB busses clocks
	    */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
		  _Error_Handler(__FILE__, __LINE__);
	  }

	    /**Configure the Systick interrupt time
	    */
	  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	    /**Configure the Systick
	    */
	  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void SystemClock_ConfigLow(void)
{

	  RCC_OscInitTypeDef RCC_OscInitStruct;
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;
	  RCC_PeriphCLKInitTypeDef PeriphClkInit;

	    /**Configure the main internal regulator output voltage
	    */
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	    /**Configure LSE Drive Capability
	    */
	  HAL_PWR_EnableBkUpAccess();

	  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	    /**Initializes the CPU, AHB and APB busses clocks
	    */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI | RCC_OSCILLATORTYPE_LSE;
	  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  RCC_OscInitStruct.MSICalibrationValue = 0;
	  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_1;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	    /**Initializes the CPU, AHB and APB busses clocks
	    */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
		  _Error_Handler(__FILE__, __LINE__);
	  }

	    /**Configure the Systick interrupt time
	    */
	  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	    /**Configure the Systick
	    */
	  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	}


/* TSC init function */
static void MX_TSC_Init(void)
{

    /**Configure the TSC peripheral 
    */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_4CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_4CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_16383;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TOUCH_SENS_PA0;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TOUCH_SAMPLE_PA3;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}



/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
