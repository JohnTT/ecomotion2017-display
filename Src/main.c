/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
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
#include "stm32f3xx_hal.h"
/* USER CODE BEGIN Includes */
#include "display.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int file_size = 15016;
int header = 16;
uint8_t rxData[4];
uint8_t pkt_size = 0xF0;
uint8_t tx = 0x00;
uint8_t img_buf[15016];
uint8_t GetSystemInfo[4] = {0x31, 0x01, 0x01, 0x00};
uint8_t DisplayUpdate[3] = {0x24, 0x01, 0x00};
uint8_t UploadImageData[3] = {0x20, 0x01, 0x00};
uint8_t ResetDataPointer[3] = {0x20, 0x0D, 0x00};
int value = 0;
int realspeed = 0; // Real Wheel speed
int flag = 0;
char strConvert[100];
int batteryLife;
//New DMA variables
int doReDraw = 1;
int isDMAFinished = 1;
int pkt_section;
DMA_sections loop_section;
//int copy_img_buf[15016];
int MAX_SECTION = 63; //something is off here, Indexing errors??
int hasDisplayed = 0;
int hasResetDataPointer;
int sentAllData = 0;
uint8_t rxResponse[2];
uint8_t txResponse[2] = {0x00, 0x00};
//End of New DMA variables
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
int main(void)
{

	/* USER CODE BEGIN 1 */
	//int counter = 89;
	batteryLife = 90;
	img_buf[0] = 0x33;
	img_buf[1] = 0x01;
	img_buf[2] = 0x90;
	img_buf[3] = 0x01;
	img_buf[4] = 0x2C;
	img_buf[5] = 0x01;
	for (int i=6; i<16; i++)
		img_buf[i] = 0x00;
	hasResetDataPointer = 0;
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	//MX_ADC1_Init();
	//MX_CAN_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();

	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1); //start the base for update interrupts
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	flag = 0;
	updateBufferDMA();
	while (1)
	{
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */

		HAL_StatusTypeDef status;
		hcan.pTxMsg->IDE = CAN_ID_STD;
		hcan.pTxMsg->RTR = CAN_RTR_DATA;
		hcan.pTxMsg->StdId = ecoMotion_Display;
		//hcan1.pTxMsg->ExtId = ((uint32_t)CAN_PACKET_SET_DUTY << 8) | controller_id;
		hcan.pTxMsg->DLC = 0;
		status = HAL_CAN_Transmit_IT(&hcan);
		if (status != HAL_OK) {
			printf("CAN Transmit Error");
			printf("\n\r");
			Error_Handler();
		}
		//flag = 0;
		//testDraw(counter);
		//		counter += 11;
		//		counter %= 100;
		//      must find a way to make waitTCBusy without polling, somehow read a GPIO port with interrupts
		//      check the waitTCBusy in the interrupt callback, if it's ready, send the next package of data
		//      using some function 'partitioning' if not, continue back to communicating to the LCD display
		//      and try again in a bit.
		//      Need to add a timer with interrupt call checks
		//

		//HAL_Delay(10);
	}
	/* USER CODE END 3 */

}
/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
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
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1
			|RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
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

	ADC_MultiModeTypeDef multimode;
	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

}
/* CAN init function */
static void MX_CAN_Init(void)
{
	__HAL_RCC_CAN1_CLK_ENABLE();
	hcan.Instance = CAN;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	setCANbitRate(1000, 32, &hcan);
	hcan.Init.TTCM = DISABLE;
	hcan.Init.ABOM = DISABLE;
	hcan.Init.AWUM = DISABLE;
	hcan.Init.NART = DISABLE;
	hcan.Init.RFLM = DISABLE;
	hcan.Init.TXFP = DISABLE;
	static CanTxMsgTypeDef TxMessage;
	static CanRxMsgTypeDef RxMessage;
	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	CAN_FilterConfTypeDef canFilterConfig;
	canFilterConfig.FilterNumber = 0;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterFIFOAssignment = 0;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.BankNumber = 14;
	if(HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK) {
		Error_Handler();
	}
}
/* SPI1 init function */
static void MX_SPI1_Init(void)
{

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}

}
/* TIM1 init function */
static void MX_TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 6400;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 10;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

}
/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}

}
/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_CSn_Pin */
	GPIO_InitStruct.Pin = SPI1_CSn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_CSn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TC_Busyn_Pin */
	GPIO_InitStruct.Pin = TC_Busyn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TC_Busyn_GPIO_Port, &GPIO_InitStruct);

}
/* USER CODE BEGIN 4 */
static void setCANbitRate(uint16_t bitRate, uint16_t periphClock, CAN_HandleTypeDef* theHcan) {
	uint8_t prescaleFactor = 0;
	switch (periphClock) {
	case 32:
		theHcan->Init.BS1 = CAN_BS1_13TQ;
		theHcan->Init.BS2 = CAN_BS2_2TQ;
		prescaleFactor = 2;
		break;
	case 45:
		theHcan->Init.BS1 = CAN_BS1_12TQ;
		theHcan->Init.BS2 = CAN_BS2_2TQ;
		prescaleFactor = 3;
		break;
	case 48:
		theHcan->Init.BS1 = CAN_BS1_11TQ;
		theHcan->Init.BS2 = CAN_BS2_4TQ;
		prescaleFactor = 3;
		break;
	}
	theHcan->Init.SJW = CAN_SJW_1TQ;
	switch (bitRate) {
	case 1000:
		theHcan->Init.Prescaler = prescaleFactor * 1;
		break;
	case 500:
		theHcan->Init.Prescaler = prescaleFactor * 2;
		break;
	case 250:
		theHcan->Init.Prescaler = prescaleFactor * 4;
		break;
	case 125:
		theHcan->Init.Prescaler = prescaleFactor * 8;
		break;
	case 100:
		theHcan->Init.Prescaler = prescaleFactor * 10;
		break;
	case 83:
		theHcan->Init.Prescaler = prescaleFactor * 12;
		break;
	case 50:
		theHcan->Init.Prescaler = prescaleFactor * 20;
		break;
	case 20:
		theHcan->Init.Prescaler = prescaleFactor * 50;
		break;
	case 10:
		theHcan->Init.Prescaler = prescaleFactor * 100;
		break;
	}
}
int numPlaces (int n)
{
	if (n < 10) return 1;
	if (n < 100) return 2;
	if (n < 1000) return 3;
	if (n < 10000) return 4;
	if (n < 100000) return 5;
	if (n < 1000000) return 6;
	if (n < 10000000) return 7;
	if (n < 100000000) return 8;
	if (n < 1000000000) return 9;
	/*      2147483647 is 2^31-1 - add more ifs as needed
       and adjust this final return as well. */
	return 10;
}
char *itoa (int value, char *result, int base)
{
	// check that the base if valid
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}
	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;
	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while ( value );
	// Apply negative sign
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}
void setAllBlack()
{
	for (int i=header; i<file_size; i++)
		img_buf[i] = 0xFF;
}
void setAllWhite()
{
	for (int i=header; i<file_size; i++)
		img_buf[i] = 0x00;
}
void drawRectangle(dispColour colour, int X1, int Y1, int X2, int Y2)
{
	for (int y=Y1; y<=Y2; y++)
		for (int x=X1; x<=X2; x++)
			setXY(colour, x, y);
}
void clearRectangle(int X1, int Y1, int X2, int Y2)
{
	for (int y=Y1; y<=Y2; y++)
		for (int x=X1; x<=X2; x++)
			setXY(DISP_WHITE, x,y);
}
void drawDiagonal(dispColour colour, int X1, int Y1, int X2, int Y2, int _dx, int _dy)
{
	int dx = (X2 >= X1) ? _dx : -_dx;
	int dy = (Y2 >= Y1) ? _dy : -_dy;
	int x = X1;
	int y = Y1;
	while (x != X2 && y != Y2) {
		setXY(colour, x,y);
		if (x != X2)
			x += dx;
		if (y != Y2)
			y += dy;
	}
	setXY(colour, x,y);
}
void setXY(dispColour colour, int x, int y)
{
	int loc = (x-1) + (400*(y-1));
	int bit = loc%8;
	int ptr = loc/8;
	ptr += header;
	uint8_t op = 0x80;
	op >>= bit;
	if (colour == DISP_BLACK)
		img_buf[ptr] = img_buf[ptr] | op;
	else if (colour == DISP_WHITE)
		img_buf[ptr] = img_buf[ptr] & (~op);
	else if (colour == DISP_INVERT)
		img_buf[ptr] = img_buf[ptr] ^ op;
}
int drawCharacter(dispColour colour, int x, int y, int pt, char c)
{
	// pt = font thickness
	// Print character C, at top left: x,y
	int px = pt-1; // thickness
	int l = 6*px; // length
	int w = 12*px; // width
	int dx, dy;
	switch (c) {
	case 'A':
	case 'B':
	case 'C':
		dx = 1;
		dy = 1;
		// vert lines
		drawRectangle(colour, x,y,x+px,y+w);
		// horiz lines
		drawRectangle(colour, x+px+1,y,x+l-px-1,y+px);
		drawRectangle(colour, x+px+1,y+w-px,x+l-px-1,y+w);
		//		for (int t=0; t<pt; t++)
		//			drawDiagonal(colour, x+t,y+w,x+l,y,dx,dy);
		break;
	case 'D':
	case 'E':
		drawRectangle(colour, x,y,x+px,y+w);
		drawRectangle(colour, x+px+1,y,x+l-px-1,y+px);
		drawRectangle(colour, x+px+1,y+w-px,x+l-px-1,y+w);
		drawRectangle(colour, x+px+1,y+w/2-px,x+l-px-1,y+w/2);
		break;
	case 'F':
	case 'G':
	case 'H':
		drawRectangle(colour, x,y,x+px,y+w);
		drawRectangle(colour, x,y+w/2,x+l,y+w/2+px);
		drawRectangle(colour, x+l-px,y,x+l,y+w);
		break;
	case 'I':
		drawRectangle(colour, x,y,x+l,y+px);
		drawRectangle(colour, x+(l/2)-(px/2),y,x+(l/2)+(px/2),y+w);
		drawRectangle(colour, x,y+w-px,x+l,y+w);
		break;
	case 'J':
	case 'K':
		drawRectangle(colour, x,y,x+px,y+w);
		dx = 1;
		dy = 1;
		for (int t=0; t<pt; t++) {
			drawDiagonal(colour, x,y+w/2+t,x+l,y+t,dx,dy);
			drawDiagonal(colour, x,y+w/2-t,x+l,y+w-t,dx,dy);
		}
		break;
	case 'L':
	case 'M':
		drawRectangle(colour, x,y,x+px,y+w);
		dx = 1;
		dy = 1;
		for (int t=0; t<px; t++) {
			drawDiagonal(colour, x+t,y,x+l/2+t,y+w/2,dx,dy);
			drawDiagonal(colour, x+l-t,y,x+l/2-t,y+w/2,dx,dy);
			drawDiagonal(colour, x,y+t,x+l/2,y+w/2+t,dx,dy);
			drawDiagonal(colour, x+l,y+t,x+l-l/2,y+w/2+t,dx,dy);
		}
		drawRectangle(colour, x+l-px,y,x+l,y+w);
		break;
	case 'N':
		drawRectangle(colour, x,y,x+px,y+w);
		dx = 1;
		dy = 2;
		for (int t=0; t<pt; t++) {
			drawDiagonal(colour, x+t,y,x+l-t,y+w,dx,dy);
			drawDiagonal(colour, x+t,y+1,x+l-t,y+w,dx,dy);
		}
		drawRectangle(colour, x+l-px,y,x+l,y+w);
		break;
	case 'O':
		dx = 1;
		dy = 1;
		// vert lines
		drawRectangle(colour, x,y,x+px,y+w);
		drawRectangle(colour, x+l-px,y,x+l,y+w);
		// horiz lines
		drawRectangle(colour, x+px+1,y,x+l-px-1,y+px);
		drawRectangle(colour, x+px+1,y+w-px,x+l-px-1,y+w);
		//		for (int t=0; t<pt; t++)
		//			drawDiagonal(colour, x+t,y+w,x+l,y,dx,dy);
		break;
	case 'P':
		drawRectangle(colour, x,y,x+l,y+px);
		drawRectangle(colour, x,y,x+px,y+w);
		drawRectangle(colour, x,y+w/2,x+l,y+w/2+px);
		drawRectangle(colour, x+l-px,y,x+l,y+w/2);
		break;
	case 'Q':
	case 'R':
	case 'S':
		drawRectangle(colour, x,y,x+l,y+px);
		drawRectangle(colour, x,y,x+px,y+w/2);
		drawRectangle(colour, x,y+w/2,x+l,y+w/2+px);
		drawRectangle(colour, x+l-px,y+w/2,x+l,y+w);
		drawRectangle(colour, x,y+w-px,x+l,y+w);
		// makes it different from 5
		drawRectangle(colour, x+l-px,y,x+l,y+w/4);
		drawRectangle(colour, x,y+w-w/4,x+px,y+w);
		break;
	case 'T':
		drawRectangle(colour, x,y,x+l,y+px);
		drawRectangle(colour, x+(l/2)-(px/2),y,x+(l/2)+(px/2),y+w);
		break;
	case 'U':
		drawRectangle(colour, x,y,x+px,y+w);
		drawRectangle(colour, x+l-px,y,x+l,y+w);
		drawRectangle(colour, x+px+1,y,x+l-px-1,y+px);
		break;
	case 'V':
	case 'W':
	case 'X':
	case 'Y':
	case 'Z':
	case 'c':
		drawRectangle(colour, x,y+w/2-px/2,x+px,y+w);
		drawRectangle(colour, x+px+1,y+w/2-(px/2),x+l,y+w/2+(px/2));
		drawRectangle(colour, x+px+1,y+w-px,x+l,y+w);
		break;
	case 'e':
		//vert lines
		drawRectangle(colour, x,y+w/2-px/2,x+px,y+w);
		drawRectangle(colour, x+l-px,y+w/2-px/2,x+l,y+3*w/4+px/2);
		//horz lines
		drawRectangle(colour, x+px+1,y+w/2-(px/2),x+l-px-1,y+w/2+(px/2));
		drawRectangle(colour, x+px+1,y+3*w/4-(px/2),x+l-px-1,y+3*w/4+(px/2));
		drawRectangle(colour, x+px+1,y+w-px,x+l,y+w);
		break;
	case 'i':
		drawRectangle(colour, x+l/2,y+w/2,x+l/2+px,y+w);
		drawRectangle(colour, x+l/2,y+w/2-6,x+l/2+px,y+w/2-3);
		break;
	case 'm':
		drawRectangle(colour, x,y+w/4,x+px,y+w);
		dx = 1;
		dy = 1;
		for (int t=0; t<px; t++) {
			drawDiagonal(colour, x+t,y+w/4,x+l/2+t,y+w/2,dx,dy);
			drawDiagonal(colour, x+l-t,y+w/4,x+l/2-t,y+w/2,dx,dy);
			drawDiagonal(colour, x,y+w/4+t,x+l/2,y+w/2+t,dx,dy);
			drawDiagonal(colour, x+l,y+w/4+t,x+l-l/2,y+w/2+t,dx,dy);
		}
		drawRectangle(colour, x+l-px,y+w/4,x+l,y+w);
		break;
	case 'n':
		drawRectangle(colour, x,y+w/2,x+px,y+w);
		drawRectangle(colour, x+l-px,y+w/2,x+l,y+w);
		drawRectangle(colour, x+px,y+w/2,x+l-px,y+w/2+px);
		break;
	case 'o':
		//vert lines
		drawRectangle(colour, x,y+w/2-px/2,x+px,y+w);
		drawRectangle(colour, x+l-px,y+w/2,x+l,y+w);
		//hori lines
		drawRectangle(colour, x+px+1,y+w/2-(px/2),x+l,y+w/2+(px/2));
		drawRectangle(colour, x+px+1,y+w-px,x+l,y+w);
		break;
	case 't':
		drawRectangle(colour, x+l/2-px/2,y+w/4,x+l/2+px/2,y+w);
		drawRectangle(colour, x,y+w/2,x+l,y+w/2+px);
		break;
	case '0':
		dx = 1;
		dy = 1;
		// vert lines
		drawRectangle(colour, x,y,x+px,y+w);
		drawRectangle(colour, x+l-px,y,x+l,y+w);
		// horiz lines
		drawRectangle(colour, x+px+1,y,x+l-px-1,y+px);
		drawRectangle(colour, x+px+1,y+w-px,x+l-px-1,y+w);
		//		for (int t=0; t<pt; t++)
		//			drawDiagonal(colour, x+t,y+w,x+l,y,dx,dy);
		break;
	case '1':
		dx = 1;
		dy = 1;
		drawRectangle(colour, x,y,x+(l/2)-(px)+1,y+px); //vert
		drawRectangle(colour, x+(l/2)-(px/2),y,x+(l/2)+(px/2),y+w-px-1); //vert
		drawRectangle(colour, x,y+w-px,x+l,y+w); //horz
		break;
	case '2':
		drawRectangle(colour, x,y,x+l,y+px); //horz 1
		drawRectangle(colour, x,y+w/2-(px/2),x+l,y+w/2+(px/2)); //horz 2
		drawRectangle(colour, x,y+w-px,x+l,y+w); //horz 3
		drawRectangle(colour, x+l-px,y+px+1,x+l,y+w/2-(px/2)-1); //vert 1
		drawRectangle(colour, x,y+w/2+(px/2)+1,x+px,y+w-px-1); //vert 2
		break;
	case '3':
		drawRectangle(colour, x,y,x+l,y+px); //horz
		drawRectangle(colour, x+l-px,y+px+1,x+l,y+w/2-(px/2)-1); //vert
		drawRectangle(colour, x,y+w/2-(px/2),x+l,y+w/2+(px/2)); //horz
		drawRectangle(colour, x+l-px,y+w/2+(px/2)+1,x+l,y+w-px-1); //vert
		drawRectangle(colour, x,y+w-px,x+l,y+w); //horz
		break;
	case '4':
		drawRectangle(colour, x,y,x+px,y+w/2-(px/2)-1); //vert
		drawRectangle(colour, x,y+w/2-(px/2),x+l-px-1,y+w/2+(px/2)); //horz
		drawRectangle(colour, x+l-px,y,x+l,y+w); //vert
		break;
	case '5':
		drawRectangle(colour, x,y,x+l,y+px); //horz
		drawRectangle(colour, x,y+px+1,x+px,y+w/2+(px/2)); //vert
		drawRectangle(colour, x+px+1,y+w/2-(px/2),x+l-px-1,y+w/2+(px/2)); //horz
		drawRectangle(colour, x+l-px,y+w/2-(px/2),x+l,y+w-px-1); //vert
		drawRectangle(colour, x,y+w-px,x+l,y+w); //horz
		break;
	case '6':
		drawRectangle(colour, x+px+1,y,x+l,y+px); //horz
		drawRectangle(colour, x,y,x+px,y+w); //vert big
		drawRectangle(colour, x+px+1,y+w/2-(px/2),x+l-px-1,y+w/2+(px/2)); //horz
		drawRectangle(colour, x+l-px,y+w/2-(px/2),x+l,y+w-px-1); //vert
		drawRectangle(colour, x+px+1,y+w-px,x+l,y+w); //horz
		break;
	case '7':
		drawRectangle(colour, x,y,x+l-px-1,y+px);
		drawRectangle(colour, x+l-px,y,x+l,y+w);
		break;
	case '8':
		drawRectangle(colour, x+px+1,y,x+l-px-1,y+px); //horz
		drawRectangle(colour, x,y,x+px,y+w); //vert
		drawRectangle(colour, x+px+1,y+w/2-(px/2),x+l-px-1,y+w/2+(px/2)); //horz
		drawRectangle(colour, x+l-px,y,x+l,y+w); //vert
		drawRectangle(colour, x+px+1,y+w-px,x+l-px-1,y+w); //horz
		break;
	case '9':
		drawRectangle(colour, x+px+1,y,x+l-px-1,y+px); //horz
		drawRectangle(colour, x,y,x+px,y+w/2-(px/2)-1); //vert
		drawRectangle(colour, x,y+w/2-(px/2),x+l-px-1,y+w/2+(px/2));
		drawRectangle(colour, x+l-px,y,x+l,y+w);
		break;
	case '/':
		dx = 1;
		dy = 2;
		for (int t=0; t<pt; t++)
			drawDiagonal(colour, x,y+w-px+t,x+l,y+t,dx,dy);
		break;
	case ':':
		drawRectangle(colour, x,y+w/4,x+px,y+w/4+px);
		drawRectangle(colour, x,y+w-w/4,x+px,y+w-w/4+px);
		break;
	case '%':
		dx = 1;
		dy = 2;
		for (int t=0; t<pt; t++)
			drawDiagonal(colour, x,y+w-px+t,x+l,y+t,dx,dy);
		drawRectangle(colour, x, y, x+(l/4), y+(px/2));
		drawRectangle(colour, x, y+(w/4), x+(l/4), y+(px/2)+(w/4));
		drawRectangle(colour, x+3*(l/4), y+(w-(px/2)), x+l, y+w);
		drawRectangle(colour, x+3*(l/4), y+(3*(w/4)), x+l, y+(w*3/4)+(px/2));
		drawRectangle(colour, x, y, x+(px/4), y+(w/4)+(px/2));
		drawRectangle(colour, x+(l/4)-(px/4), y, x+(l/4), y+(w/4)+(px/2));
		drawRectangle(colour, x+3*(l/4), y+(3*(w/4)), x+3*(l/4)+(px/4), y+w);
		drawRectangle(colour, x+l-(px/4), y+(3*(w/4)), x+l, y+w);
		break;
	default:
		break;
	}
	return x+l;
}
int drawString(dispColour colour, int x, int y, int pt, int sp, char s[], int s_len)
{
	// sp = spacing
	int st = x;
	for (int i=0; i<s_len; i++) {
		if (s[i] == '\0')
			break;
		st = drawCharacter(colour, st, y, pt, s[i]);
		st += sp + 1;
	}
	return st;
}

//Functions related to uploading and checking the E-paper display using DMA-Interrupt mode
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){
	//printf("SPI has successfully sent and received");
	isDMAFinished = 1; //dma is done
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	//printf("SPI has successfully sent");
	//tell everyone else that DMA is finished and doing nothing
	isDMAFinished = 1;
	if (sentAllData == 1){
		HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_SET);
		sentAllData = 0;
	}
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
	printf("SPI has encountered an error");
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){//for counter update event (wrap back to 0)
	//printf("Timer Update");
	//printf("\n\r");
	if (isDMAFinished && checkDMA_TCBusy()){
		dmaImageBufferSection();
		//printf("Here?");
		//printf("\n\r");
	}
}
void resetDataPointerDMA(){
	HAL_SPI_Transmit_DMA(&hspi1, ResetDataPointer, 3); //reset the data pointer
}
void uploadImageDataDMA(){
	HAL_SPI_Transmit_DMA(&hspi1, UploadImageData, 3); //tell the display we are about to upload
}
void findDataSizeDMA(){
	HAL_SPI_Transmit_DMA(&hspi1, &pkt_size, 1); //say how large the coming packet is
}
void sendPacketDMA(){
	HAL_SPI_Transmit_DMA(&hspi1, &img_buf[pkt_section*pkt_size], pkt_size); //send the packet
}
void getResponseDMA(){
	HAL_SPI_TransmitReceive_DMA(&hspi1, &tx, rxResponse, 2); //check the response
}
void readResponseDMA(){
	if (hasDisplayed == 1){
		if (doReDraw){
			loop_section = UPDATE_BUFFER;
		}
		else {
			loop_section = RESET_DATA_POINTER; //restart everything
		}
		hasDisplayed = 0; //until next display
	}
	else if (hasResetDataPointer == 1){
		loop_section = UPLOAD_IMAGE_DATA; //continue with package sending
		hasResetDataPointer = 0; //until next data pointer reset
	}
	else if ((rxResponse[0] != 0x90) && (rxResponse[1] != 0x00)) {
		printf("Errors, data send problems\n\r");
		loop_section = UPLOAD_IMAGE_DATA; //do the loop again
		rxResponse[0] = 0xFF;
		rxResponse[1] = 0xFF;
	}
	else if (pkt_section < MAX_SECTION){
		//printf("We good, data sent correctly\n\r");
		//		printf(itoa(pkt_section, strConvert, 10));
		//		printf("\n\r");
		pkt_section++;
		loop_section = UPLOAD_IMAGE_DATA; //do the loop again
	}
	else
		loop_section = DISPLAY_IMAGE;
	isDMAFinished = 1; //since the DMA didn't do anything
}
void displayImageDMA(){
	HAL_SPI_Transmit_DMA(&hspi1, DisplayUpdate, 3); //display the new image
}
void updateBufferDMA(){
	printf("Now updating the buffer with DMA\n\r");
	char c[10];
	setAllWhite();
	//	itoa(realspeed,c,10);
	//	drawString(DISP_BLACK, 100,100,5,10,c,10);
	//	c[0] = 'K';
	//	c[1] = 'M';
	//	c[2] = '/';
	//	c[3] = 'H';
	//	c[4] = '\0';
	//	drawString(DISP_BLACK, 150,122,3,5,c,4);
	//	int percentLoc = batteryImage(15, 225, 60, 4, 2, batteryLife);
	//	batteryLife += 1;
	//	if (batteryLife > 100)
	//	   batteryLife = 0;
	screenSaverImage();
}
void screenSaverImage(){
	char f[10];
	f[0] = 'e';
	f[1] = 'c';
	f[2] = 'o';
	f[3] = 'm';
	f[4] = 'o';
	f[5] = 't';
	f[6] = 'i';
	f[7] = 'o';
	f[8] = 'n';
	f[9] = '\0';
	drawString(DISP_BLACK, 50,100,5,10,f,10);
}
void dmaImageBufferSection(){
	//	printf("in buffer section");
	//	printf("\n\r");
	isDMAFinished = 0;
	switch (loop_section){
	case RESET_DATA_POINTER:
		HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);
		printf("in reset data pointer");
		printf("\n\r");
		resetDataPointerDMA();
		hasResetDataPointer = 1;
		pkt_section = 0; //getting ready to send data packet
		loop_section = GET_RESPONSE; //read the response
		printf("out of reset data pointer");
		printf("\n\r");
		break;
	case UPLOAD_IMAGE_DATA:
		//printf("Upload image\n\r");
		HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);
		uploadImageDataDMA();
		loop_section = DATA_SIZE;
		break;
	case DATA_SIZE:
		//printf("now sending all data\n\r");
		findDataSizeDMA();
		loop_section = SEND_PACKET;
		break;
	case SEND_PACKET:
		sendPacketDMA();
		loop_section = GET_RESPONSE;
		sentAllData = 1; //for proper wait tc busy
		break;
	case GET_RESPONSE:
		HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);
		getResponseDMA();
		loop_section = READ_RESPONSE;
		break;
	case READ_RESPONSE:
		HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_SET);
		readResponseDMA();
		break;
	case DISPLAY_IMAGE:
		printf("Display update");
		printf("\n\r");
		HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);
		displayImageDMA();
		hasDisplayed = 1; //we displayed
		loop_section = GET_RESPONSE; //check the response
		break;
	case UPDATE_BUFFER:
		printf("Now updating the buffer\n\r");
		updateBufferDMA();
		isDMAFinished = 1; //since the DMA didn't do anything
		//doReDraw = 0; //we updated the buffer
		loop_section = RESET_DATA_POINTER;
		break;
	}
	//	printf("out buffer section");
	//	printf("\n\r");
}
uint8_t checkDMA_TCBusy(){
	return (uint8_t)HAL_GPIO_ReadPin(TC_Busyn_GPIO_Port, TC_Busyn_Pin);
}
//End of DMA SPI functions

int batteryImage(int x, int y, int size, int fontSize, int fontSpacing, int percent){
	char c[10];

	int numDigs = 2;
	if (percent >= 10) {
		numDigs++;
	}
	if (percent >= 100) {
		numDigs++;
	}
	int textLeft = size - (4 * fontSize * numDigs + fontSpacing * (numDigs - 1)) / 2;
	int textTop = (size / 2) - (4 * fontSize);
	float batDrawPercent = (((2*size)+(size/5)) * ((float)percent) / 100);
	drawRectangle(DISP_BLACK, x-2, y-2, x+(2*size)+2, y+(size)+2);
	drawRectangle(DISP_BLACK, x+(2*size)-2, y+(3*size/10)-2, x+(2*size)+(size/5)+2, y+(7*size/10)+2);
	if (batDrawPercent <= 2*size) {
		drawRectangle(DISP_WHITE, x+batDrawPercent, y, x+(2*size), y+(size));
		drawRectangle(DISP_WHITE, x+(2*size), y+(3*size/10), x+(2*size)+(size/5), y+(7*size/10));
	}
	else if (batDrawPercent < ((2*size)+(size/5) - 1))
		drawRectangle(DISP_WHITE, x+batDrawPercent+2, y+(3*size/10), x+(2*size)+(size/5), y+(7*size/10));

	char m[4];
	itoa(percent, m, 10);
	if (percent < 10)
		m[1] = '%';
	else if (percent < 100)
		m[2] = '%';
	else
		m[3] = '%';

	drawString(DISP_INVERT, x+textLeft, y+textTop, fontSize, fontSpacing, m, 4);
	return (2*size)+(size/5)+2;
}
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* theHcan) {
	printf("Message Transmitted and Acknowledged");
	printf("\n\r");
}
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* theHcan) {
	long int ID;
	printf("Message Received and Acknowledged from ID ");
	ID = theHcan->pRxMsg->ExtId;
	if (ID == 0)
		ID = theHcan->pRxMsg->StdId;
	printf(itoa(ID,strConvert,10));
	printf("\n\r");
	if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK) {
		Error_Handler();
	}
	//modifications to buffer will happen before we do the reDraw.
	doReDraw = 1;
	//Additional information gathering and modification here
}
void __io_putchar(uint8_t ch) {
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}
/*
static void MX_CAN_Init(void)
{
	__HAL_RCC_CAN1_CLK_ENABLE();
	hcan.Instance = CAN;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	setCANbitRate(1000, 32, &hcan);
	hcan.Init.TTCM = DISABLE;
	hcan.Init.ABOM = DISABLE;
	hcan.Init.AWUM = DISABLE;
	hcan.Init.NART = DISABLE;
	hcan.Init.RFLM = DISABLE;
	hcan.Init.TXFP = DISABLE;
	static CanTxMsgTypeDef TxMessage;
	static CanRxMsgTypeDef RxMessage;
	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	CAN_FilterConfTypeDef canFilterConfig;
	canFilterConfig.FilterNumber = 0;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterFIFOAssignment = 0;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.BankNumber = 14;
	if(HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK) {
		Error_Handler();
	}
}
 */
/* USER CODE END 4 */
/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	printf("Error Handler");
	printf("\n\r");
	HAL_StatusTypeDef status;
	do {
		hcan.pTxMsg->IDE = CAN_ID_STD;
		hcan.pTxMsg->RTR = CAN_RTR_DATA;
		hcan.pTxMsg->StdId = ecoMotion_Error - ecoMotion_Display;
		hcan.pTxMsg->DLC = 0;
		status = HAL_CAN_Transmit_IT(&hcan);
	} while (status != HAL_OK);
	/* USER CODE END Error_Handler */
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
