
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "can.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern CanTxMsgTypeDef TxMessage;

//uint16_t buffer[2];
uint32_t TPS1;			// variabili che arrivano dal CAN BUS
uint32_t TPS2;
uint32_t TPSe;
uint8_t M = 0;

uint32_t timeTPS=0;		// controlli TPS
uint8_t implTPS=0;
uint8_t errTPS=0;
uint8_t iTPS=0;
uint8_t jTPS=0;
uint8_t flagTPS1 = 0;

//uint32_t APPS1;				// controlli APPS
//uint32_t APPS2;
uint32_t timeAPPS=0;
uint8_t implAPPS=0;
uint8_t errAPPS=0;
uint8_t iAPPS=0;
uint8_t jAPPS=0;

uint32_t timeTPSe=0;		// controlli TPSe
uint8_t implTPSe=0;
uint8_t errTPSe=0;
uint8_t iTPSe=0;
uint8_t jTPSe=0;
uint8_t flagTPSe = 0;

uint32_t timeMe=0;		// controlli M
uint32_t timeMr=0;
uint8_t implM=0;
uint8_t errM=0;
uint8_t iM=0;
uint8_t jM=0;


uint32_t time_send = 0;  //tempistiche dell'invio


uint16_t APPS1 = 0;
uint16_t APPS2 = 0;
//uint8_t i = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void initialise_monitor_handles(void);
extern void setupCANfilter();

HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if (hadc ->Instance == ADC1){
		APPS1 = HAL_ADC_GetValue(&hadc1);
	}

	if (hadc -> Instance == ADC2){
		APPS2 = HAL_ADC_GetValue(&hadc2);
	}
}


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

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	//initialise_monitor_handles();
	//printf("prova\n");
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	setupCANfilter();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//printf("%d %d\n",APPS1,APPS2);
		if(__HAL_CAN_GET_FLAG(&hcan,CAN_FLAG_FF0))
		{
			__HAL_CAN_FIFO_RELEASE(&hcan,CAN_FIFO0 );
			if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK) // Abilita la ricezione del CAN tramite Interrupt
			{
				/* Reception Error */
				//Error_Handler();
			}
		}

		//controlli TPS
		if((((TPS1+TPS2)>1050) || ((TPS1+TPS2)<950)) && (flagTPS1 == 1)) {
			if(implTPS==0) {
				timeTPS=HAL_GetTick();
				implTPS=1;
				jTPS=0;
			}
			if(((HAL_GetTick()-timeTPS)>150) && (iTPS<3)) {
				errTPS=1;
				TxMessage.StdId = 2;
				hcan.pTxMsg->Data[0] = 1;		//invia errore
				hcan.pTxMsg->Data[1] = 0;
				hcan.pTxMsg->Data[2] = 0;
				hcan.pTxMsg->Data[3] = 0;
				HAL_CAN_Transmit(&hcan, 10);
				iTPS++;
			}
		}

		if(((TPS1+TPS2)<1050) && ((TPS1+TPS2)>950) && (implTPS==1) && (flagTPS1 == 1)) {
			implTPS=0;
			iTPS=0;
			if ((errTPS==1) && (jTPS<3)) {
				errTPS=0;
				TxMessage.StdId = 2;
				hcan.pTxMsg->Data[0] = 0;		//cancella errore
				hcan.pTxMsg->Data[1] = 0;
				hcan.pTxMsg->Data[2] = 0;
				hcan.pTxMsg->Data[3] = 0;
				HAL_CAN_Transmit(&hcan, 10);
				jTPS++;
			}
		}


		//controlli APPS
		if(((APPS1+APPS2)>4305) || ((APPS1+APPS2)<3895)) {
			if(implAPPS==0) {
				timeAPPS=HAL_GetTick();
				implAPPS=1;
				jAPPS=0;
			}
			if(((HAL_GetTick()-timeAPPS)>150) && (iAPPS<3)) {
				errAPPS=1;
				TxMessage.StdId = 3;
				hcan.pTxMsg->Data[0] = 1;		//invia errore
				hcan.pTxMsg->Data[1] = 0;
				hcan.pTxMsg->Data[2] = 0;
				hcan.pTxMsg->Data[3] = 0;
				HAL_CAN_Transmit(&hcan, 10);
				iAPPS++;
			}
		}

		if(((APPS1+APPS2)>3895) && ((APPS1+APPS2)<4305) && (implAPPS==1)) {
			implAPPS=0;
			iAPPS=0;
			if ((errAPPS==1) && (jAPPS<3)) {
				errAPPS=0;
				TxMessage.StdId = 3;
				hcan.pTxMsg->Data[0] = 0;		//cancella errore
				hcan.pTxMsg->Data[1] = 0;
				hcan.pTxMsg->Data[2] = 0;
				hcan.pTxMsg->Data[3] = 0;
				HAL_CAN_Transmit(&hcan, 10);
				jAPPS++;
			}
		}

		//controlli TPSe
		if((((TPSe-TPS1)>50) || (((TPSe-TPS1)<(-50)))) && (flagTPSe == 1)) {
			if(implTPSe==0) {
				timeTPSe=HAL_GetTick();
				implTPSe=1;
				jTPSe=0;
			}
			if(((HAL_GetTick()-timeTPSe)>750) && (iTPSe<3)) {
				errTPSe=1;
				TxMessage.StdId = 4;
				hcan.pTxMsg->Data[0] = 1;		//invia errore
				hcan.pTxMsg->Data[1] = 0;
				hcan.pTxMsg->Data[2] = 0;
				hcan.pTxMsg->Data[3] = 0;
				HAL_CAN_Transmit(&hcan, 10);
				iTPSe++;
			}
		}

		if(((TPSe-TPS1)<50) && (((TPSe-TPS1)>(-50))) && (flagTPSe == 1)) {
			implTPSe=0;
			iTPSe=0;
			if ((errTPSe==1) && (jTPSe<3)) {
				errTPSe=0;
				TxMessage.StdId = 4;
				hcan.pTxMsg->Data[0] = 0;		//cancella errore
				hcan.pTxMsg->Data[1] = 0;
				hcan.pTxMsg->Data[2] = 0;
				hcan.pTxMsg->Data[3] = 0;
				HAL_CAN_Transmit(&hcan, 10);
				jTPSe++;
			}
		}

		//controllo M e TPS
		if((M==1) && ((TPS1<66) || (TPS1>74)) /*&& (flagTPS1 == 1)*/) {
			if(implM==0) {
				timeMe=HAL_GetTick();
				implM=1;
				jM=0;
			}
			if(((HAL_GetTick()-timeMe)>1500) && (iM<3)) {
				errM=1;
				TxMessage.StdId = 0;
				hcan.pTxMsg->Data[0] = 1;		//invia errore
				hcan.pTxMsg->Data[1] = 0;
				hcan.pTxMsg->Data[2] = 0;
				hcan.pTxMsg->Data[3] = 0;
				HAL_CAN_Transmit(&hcan, 10);
				iM++;
			}
		}

		if((M==1) && ((TPS1>66) && (TPS1<74))) { // errore, ma i valori sono tornati giusti
			if(implM==1){
				implM=0;
				timeMr=HAL_GetTick();
				iM=0;
			}
			if(((HAL_GetTick()-timeMr)>1500) && (jM<3)) {
				errM=0;
				TxMessage.StdId = 0;
				hcan.pTxMsg->Data[0] = 0;		//cancella errore
				hcan.pTxMsg->Data[1] = 0;
				hcan.pTxMsg->Data[2] = 0;
				hcan.pTxMsg->Data[3] = 0;
				HAL_CAN_Transmit(&hcan, 10);
				jM++;
				M = 0;
			}
		}

		if((HAL_GetTick() -  time_send) > 8){  //invio dei valori dell'ADC ogni 5ms
			TxMessage.StdId = 11;
			/*byte_1 = buffer[0] & 0xFF;
	  			byte_2 = buffer[0] >> 8;
	  			byte_3 = buffer[1] & 0xFF;
	  			byte_4 = buffer[1] >> 8;
	  			byte_5 = ((uint16_t) byte_2 << 8) + byte_1;
	  			byte_6 = ((uint16_t) byte_4 << 8) + byte_3;*/

			/*	hcan.pTxMsg->Data[1] = & 0xFF;
			hcan.pTxMsg->Data[0] = buffer[0] >> 8;
			hcan.pTxMsg->Data[3] = buffer[1] & 0xFF;
			hcan.pTxMsg->Data[2] = buffer[1] >> 8;*/
			hcan.pTxMsg->Data[1] = APPS1 & 0xFF;
			hcan.pTxMsg->Data[0] = APPS1 >> 8;
			hcan.pTxMsg->Data[3] = APPS2 & 0xFF;
			hcan.pTxMsg->Data[2] = APPS2 >> 8;
			HAL_CAN_Transmit(&hcan, 10);
			time_send = HAL_GetTick();
		}
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
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

/* USER CODE BEGIN 4 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->pRxMsg->StdId == 20){
		flagTPS1 = 1;
		TPS1 = (((uint16_t)hcan->pRxMsg->Data[0]) << 8) + hcan->pRxMsg->Data[1];
		TPS2 = (((uint16_t)hcan->pRxMsg->Data[2]) << 8) + hcan->pRxMsg->Data[3];
		TPSe = (((uint16_t)hcan->pRxMsg->Data[4]) << 8) + hcan->pRxMsg->Data[5];
	}
	else if (hcan->pRxMsg->StdId == 1) {
		M = (((uint8_t)hcan->pRxMsg->Data[0]));
	}

	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_EWG |
			CAN_IT_EPV |
			CAN_IT_BOF |
			CAN_IT_LEC |
			CAN_IT_ERR |
			CAN_IT_FOV0 |
			CAN_IT_FMP0);
}
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
