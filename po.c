/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
char recived;
uint8_t licz;
float temperature;
	char buffer[30];
	int messageSize;
	int sekundy=0;
	int sek=0;
	volatile int a1=0;
	volatile int a2=0;
	static uint16_t cnt = 0; // Licznik wyslanych wiadomosci
	uint8_t data[50]; // Tablica przechowujaca wysylana wiadomosc.
	uint16_t size = 0; // Rozmiar wysylanej wiadomosci
	uint8_t myTab[40]= "{temp=22 hum=55 energy=0.04}\r\n";
	volatile float nm=0.001;
	int adcVal;
	volatile int iloscPom=0;
	volatile int flaga=0;
	volatile uint16_t mrug=0;
	int summrug=0;
	int size1=0;
	int zmienna1=5;
	int zmienna2=7;
	int zmienna3=9;
	uint8_t received;
	uint8_t read=0;
	uint8_t cntU=0;
	uint8_t sReady=0;
	//odebrane
	uint8_t hi=0;
	uint8_t ok=0;
	volatile uint8_t repeat=0;
	int var1=11,var2=22,var3=33;

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart ){
	    char c = received;
	    if(c=='{'&& read==0) read=1;
	    else if(read==1){
	    	if (c!= '}'){
	    			buffer[cntU++] = c;
	    	}
	    	else if((c == '}') || (cnt == sizeof(buffer)-1)){
	    		buffer[cntU] = '\0';
	    		for(cntU;cntU<21;cntU++){
	    			buffer[cntU] = '\0';
	    		}
	    		cntU = 0;
	    		read=0;
	    		if(buffer[0]=='h'&&buffer[1]=='i'){//esp wybudzony z trybu uspienia
	    			HAL_UART_Transmit_IT(&huart2,"hi", 2);
	    			hi=1;
	    			 measurements();
	    		}
	    		else if(buffer[0]=='o'&&buffer[1]=='k'){
	    			repeat=0;
//	    			HAL_UART_Transmit_IT(&huart2, "Wyslano poprawnie", 20);
	    		}
	    		else if(buffer[0]=='r'&&buffer[1]=='e'){
	    			repeat++;
	    			if(repeat<3){
	    			sendToESP(zmienna1,zmienna2,zmienna3);
	    			}
	    			else HAL_UART_Transmit_IT(&huart2, "blad odczytu danych", 20);
	    		}
	    		else if(buffer[0]=='v'&&buffer[1]=='a'&&buffer[2]=='r'){
//	    			HAL_UART_Transmit_IT(&huart2,buffer,30);
	    			sscanf( buffer, "var1=%d var2=%d var3=%d" , &var1, &var2, &var3 );
	    			size = sprintf(data, "{q1=%d q2=%d q3=%d}", var1, var2,var3);
	    			HAL_UART_Transmit_IT(&huart2,data,size);
	    		}
	    		else if(buffer[0]=='r'&&buffer[1]=='1'&&buffer[2]=='1'){
	    			HAL_GPIO_WritePin(RL1_GPIO_Port,RL1_Pin,1);
	    		}
	    		else if(buffer[0]=='r'&&buffer[1]=='1'&&buffer[2]=='0'){
	    			HAL_GPIO_WritePin(RL1_GPIO_Port,RL1_Pin,0);
	    		}
	    		else if(buffer[0]=='r'&&buffer[1]=='2'&&buffer[2]=='1'){
	    			HAL_GPIO_WritePin(RL2_GPIO_Port,RL2_Pin,1);
	    		}
	    		else if(buffer[0]=='r'&&buffer[1]=='2'&&buffer[2]=='0'){
	    			HAL_GPIO_WritePin(RL2_GPIO_Port,RL2_Pin,0);
	    	   }
	    	}
	    }





//			 HAL_UART_Transmit_IT(&huart1, data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
//			 HAL_UART_Transmit_IT(&huart2, data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
//			 HAL_UART_Transmit_IT(&huart2, received, 10); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
//		HAL_UART_Receive_IT(&huart2, &received, 10);
		HAL_UART_Receive_IT(&huart1, &received, 1);
		HAL_UART_Receive_IT(&huart2, &received, 1);
	}
	void sendToESP(int zmienna11,int zmienna22,int zmienna33){
		size = sprintf(data, "{zmienna1=%d zmienna2=%d zmienna3=%d}", zmienna11, zmienna22,zmienna33);
		HAL_UART_Transmit_IT(&huart1,data,size);
		}
	void measurements(void){
		// pomiary i do zmiennych
		zmienna1+=5;
		if(zmienna1>20) zmienna1=1;
		zmienna2+=zmienna1;
		zmienna3++;
		sendToESP(zmienna1,zmienna2,zmienna3);
	}

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
    	RCC->APB1ENR |=  RCC_APB1ENR_TIM2EN | RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    //		//RTC
    	PWR->CR |= PWR_CR_DBP; // wl zapisow do domeny
    	RCC->CSR |= RCC_CSR_LSION; // wl LSI
    	while ( !(RCC->CSR & RCC_CSR_LSIRDY)); //czekac
    	RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSI; // LSI do RTC

    	RTC->CRL&=~ RTC_CRL_RSF; // synchronizacja domen
    	while ( RTC->CRL& RTC_CRL_RSF); //sprawdzenie
    	while ( !(RTC->CRL& RTC_CRL_RTOFF)); // czeka na skonczenie operacji zapisu
    	RTC->CRL|= RTC_CRL_CNF; // wejscie w tryb konfiguracji
    	RTC->CRH = RTC_CRH_SECIE;
    	RTC->PRLL = 40000-1;
    	RTC->PRLH = 0;
    	RTC->CNTL = 0;
    	RTC->CNTH = 0;
    	RTC->CRL&=~ RTC_CRL_CNF; //wy z trybu konfiguracji
    	while (!(RTC->CRL& RTC_CRL_RTOFF) ); //czekanie z zakonczenie zapisu
    	NVIC_ClearPendingIRQ(RTC_IRQn);
//    	NVIC_EnableIRQ(RTC_IRQn); wlaczone w hal nwm

//		HAL_UART_Receive_IT(&huart2, &received, 10);
		HAL_UART_Receive_IT(&huart1, &received, 1);
		HAL_UART_Receive_IT(&huart2, &received, 1);
		a1=5;
		a2=5;
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,1);
		// wybudzenie esp timerem badz delay
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
//		HAL_UART_Transmit_IT(&huart2,"hello",5);
//		char bufor[30]="var1=1 var2=22 var3=2";
//				sscanf(bufor, "var1=%d var2=%d var3=%d" , &var1, &var2, &var3);
//				size = sprintf(data, "{q1=%d q2=%d q3=%d}", var1, var2,var3);
//				HAL_UART_Transmit_IT(&huart2,data,size);
//				HAL_UART_Transmit_IT(&huart2,"nara",4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  HAL_Delay(1000);
//	  a1++;
//	  size = sprintf(data, "{a1=%d}", a1);
//	  HAL_UART_Transmit_IT(&huart2, data, size);



//	  size = sprintf(data, "zuzsum=%d}", licz++); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size.
//	 	 HAL_UART_Transmit_IT(&huart1, data, size);
//	 	 HAL_UART_Transmit_IT(&huart2, data, size);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
__attribute__((interrupt)) void RTC_IRQHandler(void){
if ( (RTC->CRL & RTC_CRL_SECF) ){
while ( !(RTC->CRL & RTC_CRL_RTOFF) );
RTC->CRL &=~RTC_CRL_SECF;
sekundy++;
sek++;
uint8_t size;



static uint16_t cnt = 0; // Licznik wyslanych wiadomosci
uint8_t data[50];// Tablica przechowujaca wysylana wiadomosc.


if(sekundy==10){
//	 HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
//	 size = sprintf(data, "{zmienna1=%d zmienna2=%d zmienna3=%d}", zmienna1++, zmienna2++,zmienna3++); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size.
//	 HAL_UART_Transmit_IT(&huart1, data, size);
//	 HAL_UART_Transmit_IT(&huart2, data, size);
	 measurements();
	 // wybudzenie esp
	 sekundy=0;
	 mrug=0;
}
else if(sekundy==2){
	 size = sprintf(data, "cnt=%d}", licz++); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size.
//	 HAL_UART_Transmit_IT(&huart1, data, size);

}
else if(sekundy==1){
	 size = sprintf(data, "zuzmin=%d}", licz++); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size.
//	 HAL_UART_Transmit_IT(&huart1, data, size);

}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
