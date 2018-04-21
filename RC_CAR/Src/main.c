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
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char* bufftr="mesaj bekliyor\r\n";																									// bufftr - HC05'E GONDERILECEK MESAJ
uint8_t buffrec[1];																																	// buffrec - HC05'DEN ALINACAK KARAKTER
int a=0;																																						// SINYALIZASYON KOSULU
int b=0;																																						// DORTLU KOSULU
int SOL_SERVO=1050;
int SAG_SERVO=1550;
int DIK_SERVO=1350;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);															 	  // VERI GONDERIMI AYARI
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); 																// CEVAP ALMA AYARI
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1); 																		// SERVO PWM BASLAT
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);																		// TIMER2 KANAL 2 PWM BASLAT ( MOTOR HIZ_1 KONTROL )
 	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);																		// TIMER2 KANAL 3 PWM BASLAT ( MOTOR HIZ_2 KONTROL )
	HAL_TIM_Base_Start_IT(&htim4); 																							// TIMER4 INTERNAL CLOCK BASLAT ( SINYALIZASYON SISTEMI )
	TIM2->CCR1=0; 																															// TIMER2 KANAL 1 = 0
	TIM2->CCR2=0;																															  // TIMER2 KANAL 2 = 0
	TIM2->CCR3=0;																																// TIMER2 KANAL 3 = 0
	HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_UART_Receive_IT(&huart1,buffrec,1);																											// USART ALICI KESMEYI BASLAT
		HAL_Delay(10);																																							// 10 ms  BEKLE ( CEVAP ALMA SURESI )
		if(buffrec[0] == 'F')																									  										// ILERI
		{
			TIM2->CCR1=DIK_SERVO; 																									 		  						// SERVO 90 DERECE AYARLA
			HAL_GPIO_WritePin(GPIOA,MTR_IN2_Pin|MTR_IN4_Pin,GPIO_PIN_RESET);			  									// MOTOR GERI DURDUR
			HAL_GPIO_WritePin(GPIOA,MTR_IN1_Pin|MTR_IN3_Pin,GPIO_PIN_SET);				  									// MOTOR ILERI BASLAT
			if(b==0){
				a=0;																																										// SINYALIZASYON KOSULUNU SIFIRLA
				HAL_GPIO_WritePin(GPIOB,SAG_SINYAL_Pin|SOL_SINYAL_Pin,GPIO_PIN_RESET);									// SINYALIZASYONU KAPAT
			}
		}
		else if(buffrec[0] == 'B') 																																	// GERI
		{
			TIM2->CCR1=DIK_SERVO; 																																		// SERVO 90 DERECE AYARLA
			HAL_GPIO_WritePin(GPIOA,MTR_IN1_Pin|MTR_IN3_Pin,GPIO_PIN_RESET);													// MOTOR ILERI DURDUR
			HAL_GPIO_WritePin(GPIOA,MTR_IN2_Pin|MTR_IN4_Pin,GPIO_PIN_SET);														// MOTOR GERI BASLAT
			if(b==0){
				a=0;																																										// SINYALIZASYON KOSULUNU SIFIRLA
				HAL_GPIO_WritePin(GPIOB,SAG_SINYAL_Pin|SOL_SINYAL_Pin,GPIO_PIN_RESET);									// SINYALIZASYONU KAPAT
			}
		}
		else if(buffrec[0] == 'R') 																																	// SAG
		{
			HAL_GPIO_WritePin(GPIOA,MTR_IN2_Pin|MTR_IN4_Pin,GPIO_PIN_RESET);													// MOTOR GERI DURDUR
			HAL_GPIO_WritePin(GPIOA,MTR_IN1_Pin|MTR_IN3_Pin,GPIO_PIN_RESET);													// MOTOR ILERI DURDUR
			TIM2->CCR1=SAG_SERVO;																																			// SERVO 45 DERECE AYARLA
			if(b==0){
			HAL_GPIO_WritePin(GPIOB,SOL_SINYAL_Pin,GPIO_PIN_RESET);																		// SINYALIZASYONU KAPAT
			a=1;																																											// SINYALIZASYON KOSULUNU ( SAG AYARLA)
			}
		}
		else if(buffrec[0] == 'L') 																																	// SOL
		{
			HAL_GPIO_WritePin(GPIOA,MTR_IN2_Pin|MTR_IN4_Pin,GPIO_PIN_RESET);													// MOTOR GERI DURDUR
			HAL_GPIO_WritePin(GPIOA,MTR_IN1_Pin|MTR_IN3_Pin,GPIO_PIN_RESET);													// MOTOR ILERI DURDUR
			TIM2->CCR1=SOL_SERVO;																																			// SERVO 135 DERECE AYARLA
			if(b==0){
			HAL_GPIO_WritePin(GPIOB,SAG_SINYAL_Pin,GPIO_PIN_RESET);																		// SINYALIZASYONU KAPAT
			a=2;																																											// SINYALIZASYON KOSULUNU ( SOL AYARLA)
			}
		}
		else if(buffrec[0] == 'I') 																																	// ILERI-SAG
		{
			HAL_GPIO_WritePin(GPIOA,MTR_IN2_Pin|MTR_IN4_Pin,GPIO_PIN_RESET);													// MOTOR GERI DURDUR
			HAL_GPIO_WritePin(GPIOA,MTR_IN1_Pin|MTR_IN3_Pin,GPIO_PIN_SET);														// MOTOR ILERI BASLAT
			TIM2->CCR1=SAG_SERVO;																																			// SERVO 45 DERECE AYARLA
			if(b==0){
			HAL_GPIO_WritePin(GPIOB,SOL_SINYAL_Pin,GPIO_PIN_RESET);																		// SINYALIZASYONU KAPAT
			a=1;																																											// SINYALIZASYON KOSULUNU ( SAG AYARLA)
			}
		}
		else if(buffrec[0] == 'G') 																																	// ILERI-SOL
		{
			HAL_GPIO_WritePin(GPIOA,MTR_IN2_Pin|MTR_IN4_Pin,GPIO_PIN_RESET);													// MOTOR GERI DURDUR
			HAL_GPIO_WritePin(GPIOA,MTR_IN1_Pin|MTR_IN3_Pin,GPIO_PIN_SET);														// MOTOR ILERI BASLAT
			TIM2->CCR1=SOL_SERVO;																																			// SERVO 135 DERECE AYARLA
			if(b==0){
			HAL_GPIO_WritePin(GPIOB,SAG_SINYAL_Pin,GPIO_PIN_RESET);																		// SINYALIZASYONU KAPAT
			a=2;																																											// SINYALIZASYON KOSULUNU ( SOL AYARLA)
			}
		}
		else if(buffrec[0] == 'J') 																																	// GERI-SAG
		{
			HAL_GPIO_WritePin(GPIOA,MTR_IN1_Pin|MTR_IN3_Pin,GPIO_PIN_RESET);													// MOTOR ILERI DURDUR
			HAL_GPIO_WritePin(GPIOA,MTR_IN2_Pin|MTR_IN4_Pin,GPIO_PIN_SET);														// MOTOR GERI BASLAT
			TIM2->CCR1=SAG_SERVO;																																			// SERVO 45 DERECE AYARLA
			if(b==0){
			HAL_GPIO_WritePin(GPIOB,SOL_SINYAL_Pin,GPIO_PIN_RESET);																		// SINYALIZASYONU KAPAT
			a=1;																																											// SINYALIZASYON KOSULUNU ( SAG AYARLA)
			}
		}
		else if(buffrec[0] == 'H') 																																	// GERI-SOL
		{
			HAL_GPIO_WritePin(GPIOA,MTR_IN1_Pin|MTR_IN3_Pin,GPIO_PIN_RESET);													// MOTOR ILERI DURDUR
			HAL_GPIO_WritePin(GPIOA,MTR_IN2_Pin|MTR_IN4_Pin,GPIO_PIN_SET);														// MOTOR GERI BASLAT
			TIM2->CCR1=SOL_SERVO;																																			// SERVO 135 DERECE AYARLA
			if(b==0){
			HAL_GPIO_WritePin(GPIOB,SAG_SINYAL_Pin,GPIO_PIN_RESET);																		// SINYALIZASYONU KAPAT
			a=2;																																											// SINYALIZASYON KOSULUNU ( SOL AYARLA)
			}
		}
		else if(buffrec[0] == 'X') 																																	// DORTLU YAK
		{
		a=3;																																												// SINYALIZASYON KOSULUNU ( DORTLU AYARLA)
		b=1;																																												// DORTLU KOSULU ACIK
		}
		else if(buffrec[0]=='x') 																																		// DORTLU SONDUR
		{
			HAL_GPIO_WritePin(GPIOB,SAG_SINYAL_Pin|SOL_SINYAL_Pin,GPIO_PIN_RESET);										// SINYALIZASYONU KAPAT
			a=0;																																											// SINYALIZASYON KOSULU KAPALI
			b=0;																																											// DORTLU KOSULU KAPALI
		}
		else if(buffrec[0]=='W')																																		// FAR YAK
		{
			HAL_GPIO_WritePin(GPIOA,ON_FAR_Pin,GPIO_PIN_SET);																					// GPIOA_07 PORTUNU KUR
		}
		else if(buffrec[0]=='w')																																		// FAR SONDUR
		{
			HAL_GPIO_WritePin(GPIOA,ON_FAR_Pin,GPIO_PIN_RESET);																				// GPIOA_07 PORTUNU RESETLE
		}
		else if(buffrec[0]=='V')																																		// KORNA ÇAL
		{
			HAL_GPIO_WritePin(GPIOB,KORNA_Pin,GPIO_PIN_SET);																					// GPIOB_10 PORTUNU KUR
		}
		else if(buffrec[0]=='v')																																		// KORNA SUS
		{				
			HAL_GPIO_WritePin(GPIOB,KORNA_Pin,GPIO_PIN_RESET);																				// GPIOB_10 PORTUNU RESETLE
		}
		else if(buffrec[0]=='U')																																		// STOP YAK
		{	
			HAL_GPIO_WritePin(GPIOB,ARKA_STOP_Pin,GPIO_PIN_SET);																			// GPIOB_00 PORTUNU KUR
		}
		else if(buffrec[0]=='u')																																		// STOP SONDUR
		{
			HAL_GPIO_WritePin(GPIOB,ARKA_STOP_Pin,GPIO_PIN_RESET);																		// GPIOB_00 PORTUNU RESETLE
		}
		else if(buffrec[0]=='S')																																		// DUR
		{
			HAL_GPIO_WritePin(GPIOA,MTR_IN1_Pin|MTR_IN2_Pin|MTR_IN3_Pin|MTR_IN4_Pin,GPIO_PIN_RESET);  // MOTOR ILERI-GERI DURDUR
			TIM2->CCR1=DIK_SERVO; 																																		// SERVO 90 DERECE AYARLA
			if(b==0){
				a=0;																																										// SINYALIZASYON KOSULUNU SIFIRLA
				HAL_GPIO_WritePin(GPIOB,SOL_SINYAL_Pin|SAG_SINYAL_Pin,GPIO_PIN_RESET);									// SINYALIZASYONU KAPAT
			}		
		}
		else if(buffrec[0]=='0')																																		// HIZ %0
		{
			TIM2->CCR2=0;																																							// CH2 0 AYARLA			( YUZDE-SIFIR)
			TIM2->CCR3=0;																																							// CH3 0 AYARLA     ( YUZDE-SIFIR)
		}
		else if(buffrec[0]=='1')																																		// HIZ %10
		{
			TIM2->CCR2=1999;																																					// CH2 1999 AYARLA ( YUZDE-ON)
			TIM2->CCR3=1999;																																					// CH3 1999 AYARLA ( YUZDE-ON)
		}
		else if(buffrec[0]=='2')																																		// HIZ %20
		{
			TIM2->CCR2=3999;																																					// CH2 3999 AYARLA ( YUZDE-YIRMI)
			TIM2->CCR3=3999;																																					// CH3 3999 AYARLA ( YUZDE-YIRMI)
		}
		else if(buffrec[0]=='3')																																		// HIZ %30
		{
			TIM2->CCR2=5999;																																					// CH2 5999 AYARLA ( YUZDE-OTUZ)
			TIM2->CCR3=5999;																																					// CH3 5999 AYARLA ( YUZDE-OTUZ)
		}
		else if(buffrec[0]=='4')																																		// HIZ %40
		{
			TIM2->CCR2=7999;																																					// CH2 7999 AYARLA ( YUZDE-KIRK)
			TIM2->CCR3=7999;																																					// CH3 7999 AYARLA ( YUZDE-KIRK)
		}
		else if(buffrec[0]=='5')																																		// HIZ %50
		{
			TIM2->CCR2=9999;																																					// CH2 9999 AYARLA ( YUZDE-ELLI)
			TIM2->CCR3=9999;																																					// CH3 9999 AYARLA ( YUZDE-ELLI)
		}
		else if(buffrec[0]=='6')																																		// HIZ %60
		{
			TIM2->CCR2=11999;																																					// CH2 11999 AYARLA ( YUZDE-ALTMIS)
			TIM2->CCR3=11999;																																					// CH3 11999 AYARLA ( YUZDE-ALTMIS)
		}
		else if(buffrec[0]=='7')																																		// HIZ %70
		{
			TIM2->CCR2=13999;																																					// CH2 13999 AYARLA ( YUZDE-YETMIS)
			TIM2->CCR3=13999;																																					// CH3 13999 AYARLA ( YUZDE-YETMIS)
		}
		else if(buffrec[0]=='8')																																		// HIZ %80
		{
			TIM2->CCR2=15999;																																					// CH2 15999 AYARLA ( YUZDE-SEKSEN)
			TIM2->CCR3=15999;																																					// CH3 15999 AYARLA ( YUZDE-SEKSEN)
		}
		else if(buffrec[0]=='9')																																		// HIZ %90
		{
			TIM2->CCR2=17999;																																					// CH2 17999 AYARLA ( YUZDE-DOKSAN)
			TIM2->CCR3=17999;																																					// CH3 17999 AYARLA ( YUZDE-DOKSAN)
		}
		else if(buffrec[0]=='q')																																		// HIZ %100
		{
			TIM2->CCR2=19999;																																					// CH2 19999 AYARLA ( YUZDE-YUZ)
			TIM2->CCR3=19999;																																					// CH3 19999 AYARLA ( YUZDE-YUZ)
			
		}
		else{
			HAL_UART_Transmit_IT(&huart1, (uint8_t *)bufftr,18); 																			// VERI GONDERIMI
			HAL_Delay(1000);
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  
  /* USER CODE END 3 */
	}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 35999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 800;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MTR_IN1_Pin|MTR_IN2_Pin|MTR_IN3_Pin|MTR_IN4_Pin 
                          |ON_FAR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARKA_STOP_Pin|SAG_SINYAL_Pin|KORNA_Pin|SOL_SINYAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MTR_IN1_Pin MTR_IN2_Pin MTR_IN3_Pin MTR_IN4_Pin 
                           ON_FAR_Pin */
  GPIO_InitStruct.Pin = MTR_IN1_Pin|MTR_IN2_Pin|MTR_IN3_Pin|MTR_IN4_Pin 
                          |ON_FAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARKA_STOP_Pin SAG_SINYAL_Pin KORNA_Pin SOL_SINYAL_Pin */
  GPIO_InitStruct.Pin = ARKA_STOP_Pin|SAG_SINYAL_Pin|KORNA_Pin|SOL_SINYAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		if(a==1){
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
		}
		else if(a==2){
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_11);	
		}
		else if(a==3){
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1|GPIO_PIN_11);
		}
	}
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
