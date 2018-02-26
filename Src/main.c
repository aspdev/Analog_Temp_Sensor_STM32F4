/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



#define RX_BUFFER_SIZE 1000
#define TX_BUFFER_SIZE 1000
#define RX_STR_UNTIL_LF_SIZE 1000
#define ADC_BUFFER_SIZE 1000
#define CMD_ARRAY_SIZE 1000
#define PARAM_ARRAY_SIZE 1000
uint8_t rx_Buffer[RX_BUFFER_SIZE];
uint8_t tx_Buffer[TX_BUFFER_SIZE];
char rx_str_until_lf[RX_STR_UNTIL_LF_SIZE];
uint8_t char_to_send[1];
uint32_t ADC_Buffer[ADC_BUFFER_SIZE];
int cmd_Array[CMD_ARRAY_SIZE];
uint32_t param_Array[PARAM_ARRAY_SIZE];
char tabCheckCmd[5];

char cmdOne[] = {'c', 'm', 'd', '1', '\0'};
char cmdTwo[] = {'c', 'm', 'd', '2', '\0'};
char cmdThree[] = {'c', 'm', 'd', '3', '\0'};



//wskazniki do odczytu oraz zapisu w buforze
uint16_t volatile rx_rd_index;
uint16_t volatile rx_wr_index;
uint16_t volatile tx_wr_index;
uint16_t volatile tx_rd_index;
uint16_t volatile rx_str_until_lf_index;
uint16_t volatile tx_str_until_lf_index;
uint16_t volatile temp_wr_index;
uint16_t volatile cmd_Array_wr_index;
uint16_t volatile param_Array_wr_index;
uint16_t volatile cmd_Array_rd_index;
uint16_t volatile param_Array_rd_index;


float AverageADCValue;
float Temperature;
float Vsense;
float SD; // odchylenie standardowe (Standard Deviation)

const float V25 = 0.75;
const float Avg_slope = 0.001;
const float SupplyVoltage = 3.3;
const float ADCResolution = 4096.0;




		union
		{
		uint64_t i;
		float f;
		}var1,var2,var3,var4;




// liczniki elementów w buforze
uint16_t volatile rx_counter; 
uint16_t volatile tx_counter;
uint16_t volatile cmd_counter;
		
uint32_t volatile tick_ms;
uint32_t volatile writeFlashTimeInterval;
uint32_t volatile measurementTime;
uint32_t volatile numberOfmeasurementsToReadFromFlash;
uint8_t volatile measurementInProgressFlag;
uint8_t volatile readyToDisplayFlag;
uint32_t volatile tx_Uart_tickstart;
uint32_t volatile blink_Led_tickstart;

uint16_t volatile transferInProgress;

// wskaznik, któremu zostanie przypisany pierwszy wolny adres podczas zapisu do pamieci FLASH		
uint32_t *mem;
// adres pod którym zostanie zapisany pomiar		
uint32_t Address;

// wskazuje na przepelnienie bufora
volatile uint8_t rx_overflow;
		


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

// prototyp funkcji, która przetwarza i interpretuje lancuch znaków przeslanych do mikrokontrolera przez UART
void processIncomingCharacters(void);
// prototyp funkckcji, kióra wykonuje rozpoznane polecenia
void executeCommands(void);
// protptyp funkcji czytajacej znak z bufora rxBuffer
char RxBufferGetChar(void);
// prototyp funkcji czytajacej znak z bufora tx_Buffer
char TxBufferGetChar(void);
// zeruje Rx_Str_Until_Lf_Buffer
void Clear_Rx_Str_Until_Lf_Buffer(void);
// zeruje Tx_Str_Until_Lf_Buffer
void Clear_char_to_send(void);
// porównuje dwa lancuchy znaków
int compare_strings(char *first, char *second);
// sprawdza czy za komenda znajduja sie 4 cyfry
void checkDigits(void);
// zapisuje do pamieci Flash
void writeFlash(void);
// odczytuje z pamieci Flash
void readFlash(int numberOfMeasurementsToRead);
// zwraca pierwszy wolny adres w sektorze
uint32_t* returnPointerToFirstEmptyAddressInSector(void);
// wyswietla pomiary na konsole monitora
void sendDataOverUart(void);
// miga dioda podczas pomiarów
void blinkLed(void);
// zatrzymuje pomiary
void stopMeasurements(void);
// inicjalizuje parametry nad podstawie wykrytych komend
void initializeParams(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// obsluga przerwania od wysylania
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
			
			if (tx_counter > 0){
				
				char_to_send[0] = TxBufferGetChar();
				HAL_UART_Transmit_IT(&huart2, (uint8_t*)char_to_send, 1);
			}
			
			else if (tx_counter == 0)
			{
					transferInProgress = 0;
			}
}

// zwieksza indeks do zapisu i ponownie uruchamia przerwanie od odbioru
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
		 rx_wr_index++;
		 rx_counter++;
		 if(rx_wr_index >= RX_BUFFER_SIZE)
		{
			rx_wr_index = 0;
		}
		 
		HAL_UART_Receive_IT(&huart2,&rx_Buffer[rx_wr_index],1);
		
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	// liczy sredni pomiar i temperature
	int tmp = 0;
	float avg = 0.0;
	
	for (int i = 500 ; i < ADC_BUFFER_SIZE; i++)
	{
		tmp += ADC_Buffer[i];
	}
	
	avg = (float)tmp / 500;
	
	Vsense = (SupplyVoltage*avg)/(ADCResolution-1);
	Temperature = (Vsense - 0.5f) * 100;
	
	AverageADCValue = avg;
		
	float sum = 0.0;
	
	for (int i = 0; i < ADC_BUFFER_SIZE; i++)
	{
		sum += (ADC_Buffer[i] - avg) * (ADC_Buffer[i] - avg);
	}
	
	float Vsense_sum = (SupplyVoltage * sum) / (ADCResolution - 1);
	float sumC = (Vsense_sum - 0.5f) * 100;
	SD = (float)(sqrt(sumC / (double) 500));
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){

	// liczy sredni pomiar i temperature
	int tmp = 0;
	float avg = 0.0;
	
	for (int i = 0 ; i < 500; i++)
	{
		tmp += ADC_Buffer[i];
	}
	
	avg = (float)tmp / 500;
	
	Vsense = (SupplyVoltage*avg)/(ADCResolution-1);
	Temperature = (Vsense - 0.5f) * 100;
	
	AverageADCValue = avg;
	
	float sum = 0.0;
	
	for (int i = 0; i < 500; i++)
	{
		sum += (ADC_Buffer[i] - avg) * (ADC_Buffer[i] - avg);
	}
	
	float Vsense_sum = (SupplyVoltage * sum) / (ADCResolution - 1);
	float sumC = (Vsense_sum - 0.5f) * 100;
	SD = (float)(sqrt(sumC / (double) 500));
}


 
void HAL_SYSTICK_Callback()
{
	tick_ms++;
	if (measurementTime != 0 && tick_ms % 1000 == 0)
	{
		measurementTime--;
	}
	
}



/* USER CODE END 0 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	//__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	
	
																						
	
																						
	 
	// uruchamiamy nasluchiwanie od odbioru na kanale UART
	HAL_UART_Receive_IT(&huart2, &rx_Buffer[rx_wr_index],1);
	
	// inicjacja wartosci tików dla transmisji przez Uart oraz dla migania diody
	tx_Uart_tickstart = HAL_GetTick();
	blink_Led_tickstart = HAL_GetTick();

	
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
				
			processIncomingCharacters();  											// interpretuje i przetwarza przychodzace do mikrokontrolera znaki
				
			initializeParams();																	// inicjalizuje parametry w oparciu o rodzaj komendy
		
			blinkLed();																					// uruchamia miganie dioda podczas pomiaru
		
			writeFlash();																				// uruchamia zapis do pamieci Flash z okreslonym parametrem
				
			sendDataOverUart();																	// wysyla dane przez UART
		
			readFlash(numberOfmeasurementsToReadFromFlash);			// czyta z pamieci Flash - ilosc odczytów okresla param przeslany do funkcji
						
			stopMeasurements();																	// zatrzymuje pomiary oraz zapis do pamieci Flash
		
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// przetwarza i interpretuje przychodzace z PC do mikrokontrolera znaki oraz filtruje polecenia
void processIncomingCharacters(void)
{
		if (rx_counter > 0)
		{
			
			char tmp = 0;
			tmp = RxBufferGetChar();
			rx_str_until_lf[rx_str_until_lf_index] = tmp;
			rx_str_until_lf_index++;
			
			// jesli przyjdzie znak konca ramiki <LF>											
			if (tmp == '\n' && rx_str_until_lf_index <= RX_STR_UNTIL_LF_SIZE)
			{
				
				// sprawdzamy czy to, co przyszlo jest zdefiniowana globalnie komenda cmd1, cmd2, cmd3
				
				for (int j = 0, k = rx_str_until_lf_index - 9; j < 4; j++, k++)
				{
					tabCheckCmd[j] = rx_str_until_lf[k];
				}
				// dopisujemy zero na koncu, bedzie potrzebne do porównania lancuchow 
				tabCheckCmd[4] = '\0';
				
				 // jesli pojawia sie polecenie cmd1, cmd2 lub cmd3
				 if (compare_strings(tabCheckCmd, cmdOne) == 0 || 
						 compare_strings(tabCheckCmd, cmdTwo) == 0 || 
						 compare_strings(tabCheckCmd, cmdThree) == 0){
											 					 
					 // sprawdzamy czy za poleceniem znajduja sie 4 cyfry
						
						checkDigits();
								 
				}	
											
				// jesli nie rozpoznano komendy zerujemy indeks i czyscimy bufor Str_Until_Lf
				else 
				{
					rx_str_until_lf_index = 0;
					Clear_Rx_Str_Until_Lf_Buffer();
				}
				
						
			}
						
			// przepisanie ostatnich 8 elementów tablicy str_until_lf, gdy ostatni znak tej tablicy nie jest '\n'
			else if (rx_str_until_lf_index == RX_STR_UNTIL_LF_SIZE && tmp != '\n')
			{
					
					char tab[8];
					for (int j = rx_str_until_lf_index - 8, k = 0; k < 8; j++, k++)
					{
						tab[k] = rx_str_until_lf[j];
					}
					rx_str_until_lf_index = 0;
					
					for (int j = rx_str_until_lf_index; j < strlen(tab); j++)
					{
						rx_str_until_lf[j] = tab[j];
					  rx_str_until_lf_index++;
					}
					// pozostala czesc tablicy str_until_lf jest zerowana
					for ( int j = rx_str_until_lf_index; j < RX_STR_UNTIL_LF_SIZE; j++)
					{
						rx_str_until_lf[j] = 0;
					}
			}
		}
}

// sprawdza czy za wykrytym poleceniem znajduja sie 4 cyfry
void checkDigits(void){

		char tempTabCheckDigits[5];
		for (int j = 0, k = rx_str_until_lf_index - 5; j < 4; j++, k++)
		{
			tempTabCheckDigits[j] = rx_str_until_lf[k];
		}
		int m = 0;
		while(1)
		{
				if(tempTabCheckDigits[m] > 57 || tempTabCheckDigits[m] < 48)
				{
					break;
				}
				m++;
				/* jesli znajduja sie cztery cyfry konwertujemy tablice char na wartosc int
					wpisujemy skonwertowana wartosc liczbowa do tablicy parametrow
					cyfry moga byc tylko 4, gdyz wczesniej program "wycial" 8 znaków i potwierdzil,
					ze pierwsze 4 to komenda */
				if (m == 4)
				{
						tempTabCheckDigits[4] = '\0';
						sscanf(tempTabCheckDigits, "%d", &param_Array[param_Array_wr_index++]);
						int cmdSum = 0;
						for (int j = 0; j < 4; j++)
						{
							cmdSum += tabCheckCmd[j];
						}
						// wpisujemy rozpoznana komende do tablicy komend
						cmd_Array[cmd_Array_wr_index++] = cmdSum;
						if (cmd_Array_wr_index == CMD_ARRAY_SIZE)
						{
							cmd_Array_wr_index = 0;
						}
						// zwiekszamy licznik komend
						cmd_counter++;
								break;
					}
			}
						rx_str_until_lf_index = 0;
						Clear_Rx_Str_Until_Lf_Buffer();
}


// inicjalizuje parametry na podstawie komend
void initializeParams(void){

	if (cmd_counter > 0){
			
			if (cmd_Array[cmd_Array_rd_index] == 357){
			
					//Rozpoczyna pomiary, uruchamia przesyl konwersji z ADC do ADC_Buffer przez DMA
					// pomiar trwa przez zadany czas podany w sekundach (0001, 0002, 0003, etc.)
					measurementTime = param_Array[param_Array_rd_index];
					if (measurementTime != 0)
					{
						HAL_ADC_Start_DMA(&hadc1, ADC_Buffer, ADC_BUFFER_SIZE);
						measurementInProgressFlag = 1;
					}
					param_Array_rd_index++;
					if (param_Array_rd_index == PARAM_ARRAY_SIZE)
					{
						param_Array_rd_index = 0;
					}
					cmd_Array_rd_index++;
					if (cmd_Array_rd_index == CMD_ARRAY_SIZE)
					{
						cmd_Array_rd_index = 0;
					}
					cmd_counter--;
			
			}
			
			if (cmd_Array[cmd_Array_rd_index] == 358){
			
					//zapisuje pomiary do Flash z okreslonym interwalem czasowym podanym jako parametr wywolanej funkcji
					//interwal zapisu podany jest w milisekindach (1000, 2000, 3000, 4000, etc.)
					writeFlashTimeInterval = param_Array[param_Array_rd_index];
					param_Array_rd_index++;
					if (param_Array_rd_index == PARAM_ARRAY_SIZE)
					{
						param_Array_rd_index = 0;
					}
					cmd_Array_rd_index++;
					if (cmd_Array_rd_index == CMD_ARRAY_SIZE)
					{
						cmd_Array_rd_index = 0;
					}
					cmd_counter--;
			
			}
			
			if (cmd_Array[cmd_Array_rd_index] == 359){
			
					// odczytuje zapisane wartosci w pamieci, 
				 // ilosc zwróconych odczytów wskazuje parametr (0001, 0002, 0003, 0004, etc.)  
					numberOfmeasurementsToReadFromFlash = param_Array[param_Array_rd_index];
					param_Array_rd_index++;
					if (param_Array_rd_index == PARAM_ARRAY_SIZE)
					{
						param_Array_rd_index = 0;
					}
					cmd_Array_rd_index++;
					if (cmd_Array_rd_index == CMD_ARRAY_SIZE)
					{
						cmd_Array_rd_index = 0;
					}
					cmd_counter--;
			
			}
		}

}


// zatrzymuje pomiary

void stopMeasurements(void){
		
		if (measurementTime == 0)
		{
			HAL_ADC_Stop_DMA(&hadc1);
			measurementInProgressFlag = 0;
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			writeFlashTimeInterval = 0;
		}
}

// funkcja miga dioda gdy trwa pomiar
void blinkLed(void){

		if (measurementTime != 0)
		{
			if(HAL_GetTick() - blink_Led_tickstart >= 500)
			{
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				blink_Led_tickstart = HAL_GetTick();
			}
		}

}


// zapisuje do pamieci pomiar temperatury oraz odchylenie standardowe
void writeFlash(void){
	
	if ((measurementTime != 0) && (writeFlashTimeInterval != 0) && (tick_ms % writeFlashTimeInterval == 0)){
	
	mem = returnPointerToFirstEmptyAddressInSector();
	Address = (uint32_t)mem;
	var1.f = Temperature;
	var2.f = SD;
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);
	
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Address, var1.i );
	
	mem++;
	Address = (uint32_t)mem;
			
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Address, var2.i);	
	HAL_FLASH_Lock();
	
	}
}
// odczytuje z pamieci FLASH pomiary i zapisuje w buforze nadawczym 
void readFlash(int numberOfMeasurementsToRead){
	
	if( numberOfmeasurementsToReadFromFlash != 0 ){
	uint32_t *mem;
	mem = returnPointerToFirstEmptyAddressInSector();
	char buffer[10];
	uint32_t *firstAddressInSector = (uint32_t *)0x08060000;
	for (int i = 0; i < numberOfMeasurementsToRead; i++)
	{
			mem--; // cofamy wskaznik o 4 bajty na ostatni zapisany adres
			if (mem < firstAddressInSector)
			{
				break;
			}
			var3.i = *mem;
			// konwertujemy wartosc typu float na tablice char
			snprintf(buffer, 10, "%.2f%cC\n", var3.f, 0x00B0);
			for (int j = 0; j < strlen(buffer); j++)
			{
				// zapisujemy kolejne znaki do bufora nadawczego tx_Buffer
				tx_Buffer[tx_wr_index] = buffer[j];
				tx_counter++;
				tx_wr_index++;
				if(tx_wr_index > TX_BUFFER_SIZE)
				{
					tx_wr_index = 0;
				}
			}
	}
		numberOfmeasurementsToReadFromFlash = 0;
	}
	 
}

// funkcja wysyla dane przez USART. Pierwszy znak jest wywylany z watku glównego, kolejne w przerwaniu (Callback)
void sendDataOverUart(void)
{
			if( tx_counter > 0 && transferInProgress == 0){
				
				char_to_send[0] = TxBufferGetChar();
				HAL_UART_Transmit_IT(&huart2, (uint8_t*)char_to_send, 1);
				transferInProgress = 1;
		}
}
// funkcja pozwala porównywac lancuchy znaków
int compare_strings(char *first, char *second) {
   while (*first == *second) {
      if (*first == '\0' || *second == '\0')
         break;
 
      first++;
      second++;
   }
 
   if (*first == '\0' && *second == '\0')
      return 0;
   else
      return -1;
}

// funkcja zerujaca bufor rx_str_until_lf
void Clear_Rx_Str_Until_Lf_Buffer(void)
{
	for (int i = 0; i < RX_BUFFER_SIZE; i++)
	{
		rx_str_until_lf[i] = 0;
	}

}

// funkcja zerujaca bufor tx_str_until_lf
void Clear_char_to_send(void)
{
	for (int i = 0; i < TX_BUFFER_SIZE; i++)
	{
			char_to_send[i] = 0;
	}

}

// definicja funkcji czytajacej znak z tx_Buffer
char TxBufferGetChar(void)
{
	char tmp;
	
	// czytamy znak z bufora tx_Buffer i zapisujemy do zmiennej tmp
	tmp = tx_Buffer[tx_rd_index];
	
	// po odczycie zwiekszamy indeks do odczytu
	tx_rd_index++;
	
	// jezeli indeks wskazuje na koniec bufora to zerujemy indeks
	if(tx_rd_index >= TX_BUFFER_SIZE)
	{
		tx_rd_index = 0;
	}
	
	// zmniejszamy stan licznika tx_counter
	tx_counter--;
	
	return tmp;
}

// definicja funkcji czytajacej znak z rx_Buffer
char RxBufferGetChar(void)
{
	char tmp;
	// jezeli do bufora zostanie zapisany znak czytamy go i zapisujemy do zmiennej tmp
	tmp = rx_Buffer[rx_rd_index];
	// po odczycie zwiekszamy indeks do odczytu 
	rx_rd_index++;
	// jezeli indeks wskazuje na koniec bufora zerujemy indeks
	if (rx_rd_index >= RX_BUFFER_SIZE)
	{
		rx_rd_index = 0;
	}
	//wylaczamy na chwile przerwania od odbioru, aby zmniejszyc stan licznika rx_counter 
	__disable_irq();
	rx_counter--;
	__enable_irq();
	return tmp;
}
	// zwraca wskaznik na pierwszy niezapisany bajt w Sektorze nr 7
	// jesli sektor jest w calosci zapisany wymazuje caly sektor i ustawia na pierwszy bajt w sektorze
	uint32_t* returnPointerToFirstEmptyAddressInSector()
{
	uint32_t *mem = (uint32_t *)0x08060000;
	uint32_t *lastAddressInSector = (uint32_t *)0x0807FFFF;
		
	while(*mem != 0xFFFFFFFF)
	{
		// przesuwamy o 4 bajty
		mem++;
		
		if (mem > lastAddressInSector)
		{
			mem = (uint32_t *) 0x8060000;
			HAL_FLASH_Unlock();
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);
			FLASH_Erase_Sector(FLASH_SECTOR_7, VOLTAGE_RANGE_3);
			HAL_FLASH_Lock();
		}
	}
	
	return mem; 

}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
