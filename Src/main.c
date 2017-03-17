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

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void setCANbitRate(uint16_t bitRate, uint16_t periphClock, CAN_HandleTypeDef* theHcan);
void init();
void waitTCBusy();
int readResponse(int Le);
void readStrResponse();
void clearImageBuffer();
void drawRectangle();
void setAllBlack();
void setAllWhite();
void setBlackXY(int x, int y);
void setWhiteXY(int x, int y);
void drawRectangle(int X1, int Y1, int X2, int Y2);
void clearRectangle(int X1, int Y1, int X2, int Y2);
void drawDiagonal(int X1, int Y1, int X2, int Y2, int dx, int dy);
int drawCharacter(int x, int y, int pt, char c);
int drawString(int x, int y, int pt, int sp, char s[], int size);
void batteryImage(int x, int y, int pt, int percent);
void testDraw();
void delay(int x);
// Functions defined for the TCM 441-230
void uploadImageBuffer();
void resetDataPointer();
void displayUpdate();
void getDeviceInfo();
void getDeviceID();
void getSystemInfo();
void getSystemVersionCode();
void readSensorData();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	int counter = 1;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		printf("Test");
		printf("\n\r");
//		flag = 0;
//		testDraw(counter);
//		counter *= 2;




		HAL_Delay(1000);
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
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
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
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
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

void delay(int x)
{
	int m;
	for (m = 0; m < x*10000; m++);
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
void waitTCBusy()
{
	uint8_t rx;
	HAL_Delay(10);

	rx = HAL_GPIO_ReadPin(TC_Busyn_GPIO_Port, TC_Busyn_Pin);
	while (!rx) {
		// printf("\n\rTC Busy\n\r");
		rx = HAL_GPIO_ReadPin(TC_Busyn_GPIO_Port, TC_Busyn_Pin);
		HAL_Delay(10);
	}
}
int readResponse(int Le)
{
	int flag = 0;
	int rx;
	waitTCBusy();
	HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);
	for (int i=0; i<Le+2; i++) {
		HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
		//        printf(" **%x ", (uint8_t)rxData);
		if (i == 0 && (uint8_t)rx != 0x90)
			flag = 1;
		if (i == 1 && (uint8_t)rx != 0x00)
			flag = 1;
	}
	HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_SET);
	//    printf("\n\rEnd of Response %d\n\r", flag);
	return flag;
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
void drawRectangle(int X1, int Y1, int X2, int Y2)
{
	for (int y=Y1; y<=Y2; y++)
		for (int x=X1; x<=X2; x++)
			setBlackXY(x,y);
}
void drawWhiteRectangle(int X1, int Y1, int X2, int Y2)
{
	for (int y=Y1; y<=Y2; y++)
		for (int x=X1; x<=X2; x++)
			setWhiteXY(x,y);
}
void clearRectangle(int X1, int Y1, int X2, int Y2)
{
	for (int y=Y1; y<=Y2; y++)
		for (int x=X1; x<=X2; x++)
			setWhiteXY(x,y);
}
void drawDiagonal(int X1, int Y1, int X2, int Y2, int _dx, int _dy)
{
	int dx = (X2 >= X1) ? _dx : -_dx;
	int dy = (Y2 >= Y1) ? _dy : -_dy;
	int x = X1;
	int y = Y1;
	while (x != X2 && y != Y2) {
		setBlackXY(x,y);
		if (x != X2)
			x += dx;
		if (y != Y2)
			y += dy;
	}
	setBlackXY(x,y);
}
void drawWhiteDiagonal(int X1, int Y1, int X2, int Y2, int _dx, int _dy)
{
	int dx = (X2 >= X1) ? _dx : -_dx;
	int dy = (Y2 >= Y1) ? _dy : -_dy;
	int x = X1;
	int y = Y1;
	while (x != X2 && y != Y2) {
		setWhiteXY(x,y);
		if (x != X2)
			x += dx;
		if (y != Y2)
			y += dy;
	}
	setWhiteXY(x,y);
}
void setBlackXY(int x, int y)
{
	int loc = (x-1) + (400*(y-1));
	int bit = loc%8;
	int ptr = loc/8;
	ptr += header;
	uint8_t op = 0b10000000;
	op = op >> bit;
	img_buf[ptr] = img_buf[ptr] | op;
}
void setWhiteXY(int x, int y)
{
	int loc = (x-1) + (400*(y-1));
	int bit = loc%8;
	int ptr = loc/8;
	ptr += header;
	int op = 0b10000000;
	op = op >> bit;
	img_buf[ptr] = img_buf[ptr] & (~op);
}
int drawCharacter(int x, int y, int pt, char c)
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
	case 'D':
	case 'E':
	case 'F':
	case 'G':
	case 'H':
		drawRectangle(x,y,x+px,y+w);
		drawRectangle(x,y+w/2,x+l,y+w/2+px);
		drawRectangle(x+l-px,y,x+l,y+w);
		break;
	case 'I':
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x+(l/2)-(px/2),y,x+(l/2)+(px/2),y+w);
		drawRectangle(x,y+w-px,x+l,y+w);
		break;
	case 'J':
	case 'K':
		drawRectangle(x,y,x+px,y+w);
		dx = 1;
		dy = 1;
		for (int t=0; t<pt; t++) {
			drawDiagonal(x,y+w/2+t,x+l,y+t,dx,dy);
			drawDiagonal(x,y+w/2-t,x+l,y+w-t,dx,dy);
		}
		break;
	case 'L':
	case 'M':
		drawRectangle(x,y,x+px,y+w);
		dx = 1;
		dy = 1;
		for (int t=0; t<px; t++) {
			drawDiagonal(x+t,y,x+l/2+t,y+w/2,dx,dy);
			drawDiagonal(x+l-t,y,x+l/2-t,y+w/2,dx,dy);
			drawDiagonal(x,y+t,x+l/2,y+w/2+t,dx,dy);
			drawDiagonal(x+l,y+t,x+l-l/2,y+w/2+t,dx,dy);
		}
		drawRectangle(x+l-px,y,x+l,y+w);
		break;
	case 'N':
	case 'O':
	case 'P':
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x,y,x+px,y+w);
		drawRectangle(x,y+w/2,x+l,y+w/2+px);
		drawRectangle(x+l-px,y,x+l,y+w/2);
		break;
	case 'Q':
	case 'R':
	case 'S':
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x,y,x+px,y+w/2);
		drawRectangle(x,y+w/2,x+l,y+w/2+px);
		drawRectangle(x+l-px,y+w/2,x+l,y+w);
		drawRectangle(x,y+w-px,x+l,y+w);
		// makes it different from 5
		drawRectangle(x+l-px,y,x+l,y+w/4);
		drawRectangle(x,y+w-w/4,x+px,y+w);
		break;
	case 'T':
	case 'U':
	case 'V':
	case 'W':
	case 'X':
	case 'Y':
	case 'Z':
	case '0':
		dx = 1;
		dy = 1;
		// vert lines
		drawRectangle(x,y,x+px,y+w);
		drawRectangle(x+l-px,y,x+l,y+w);
		// horiz lines
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x,y+w-px,x+l,y+w);
		for (int t=0; t<pt; t++)
			drawDiagonal(x+t,y+w,x+l,y,dx,dy);
		break;
	case '1':
		dx = 1;
		dy = 1;
		for (int t=0; t<pt; t++)
			drawDiagonal(x,y+w/4-t,x+l/2,y,dx,dy);
		drawRectangle(x+(l/2)-(px/2),y,x+(l/2)+(px/2),y+w);
		drawRectangle(x,y+w-px,x+l,y+w);
		break;
	case '2':
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x,y+w-px,x+l,y+w);
		drawRectangle(x+l-px,y,x+l,y+w/2);
		dx = 1;
		dy = 1;
		for (int t=0; t<pt; t++)
			drawDiagonal(x,y+w-t,x+l,y+w/2,dx,dy);
		break;
	case '3':
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x+l-px,y,x+l,y+w/2);
		drawRectangle(x,y+w/2,x+l,y+w/2+px);
		drawRectangle(x+l-px,y+w/2,x+l,y+w);
		drawRectangle(x,y+w-px,x+l,y+w);
		break;
	case '4':
		drawRectangle(x,y,x+px,y+w/2);
		drawRectangle(x,y+w/2,x+l,y+w/2+px);
		drawRectangle(x+l-px,y,x+l,y+w);
		break;
	case '5':
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x,y,x+px,y+w/2);
		drawRectangle(x,y+w/2,x+l,y+w/2+px);
		drawRectangle(x+l-px,y+w/2,x+l,y+w);
		drawRectangle(x,y+w-px,x+l,y+w);
		break;
	case '6':
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x,y,x+px,y+w);
		drawRectangle(x,y+w/2,x+l,y+w/2+px);
		drawRectangle(x+l-px,y+w/2,x+l,y+w);
		drawRectangle(x,y+w-px,x+l,y+w);
		break;
	case '7':
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x+l-px,y,x+l,y+w);
		break;
	case '8':
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x,y,x+px,y+w);
		drawRectangle(x,y+w/2,x+l,y+w/2+px);
		drawRectangle(x+l-px,y,x+l,y+w);
		drawRectangle(x,y+w-px,x+l,y+w);
		break;
	case '9':
		drawRectangle(x,y,x+l,y+px);
		drawRectangle(x,y,x+px,y+w/2);
		drawRectangle(x,y+w/2,x+l,y+w/2+px);
		drawRectangle(x+l-px,y,x+l,y+w);
		break;
	case '/':
		dx = 1;
		dy = 1;
		for (int t=0; t<pt; t++)
			drawDiagonal(x,y+w/2+t+2*px,x+l,y+t+2*px,dx,dy);
		break;
	case ':':
		drawRectangle(x,y+w/4,x+px,y+w/4+px);
		drawRectangle(x,y+w-w/4,x+px,y+w-w/4+px);
		break;
	default:
		break;
	}
	return x+l;
}
int drawWhiteCharacter(int x, int y, int pt, char c)
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
	case 'D':
	case 'E':
	case 'F':
	case 'G':
	case 'H':
		drawWhiteRectangle(x,y,x+px,y+w);
		drawWhiteRectangle(x,y+w/2,x+l,y+w/2+px);
		drawWhiteRectangle(x+l-px,y,x+l,y+w);
		break;
	case 'I':
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x+(l/2)-(px/2),y,x+(l/2)+(px/2),y+w);
		drawWhiteRectangle(x,y+w-px,x+l,y+w);
		break;
	case 'J':
	case 'K':
		drawWhiteRectangle(x,y,x+px,y+w);
		dx = 1;
		dy = 1;
		for (int t=0; t<pt; t++) {
			drawWhiteDiagonal(x,y+w/2+t,x+l,y+t,dx,dy);
			drawWhiteDiagonal(x,y+w/2-t,x+l,y+w-t,dx,dy);
		}
		break;
	case 'L':
	case 'M':
		drawWhiteRectangle(x,y,x+px,y+w);
		dx = 1;
		dy = 1;
		for (int t=0; t<px; t++) {
			drawWhiteDiagonal(x+t,y,x+l/2+t,y+w/2,dx,dy);
			drawWhiteDiagonal(x+l-t,y,x+l/2-t,y+w/2,dx,dy);
			drawWhiteDiagonal(x,y+t,x+l/2,y+w/2+t,dx,dy);
			drawWhiteDiagonal(x+l,y+t,x+l-l/2,y+w/2+t,dx,dy);
		}
		drawWhiteRectangle(x+l-px,y,x+l,y+w);
		break;
	case 'N':
	case 'O':
	case 'P':
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x,y,x+px,y+w);
		drawWhiteRectangle(x,y+w/2,x+l,y+w/2+px);
		drawWhiteRectangle(x+l-px,y,x+l,y+w/2);
		break;
	case 'Q':
	case 'R':
	case 'S':
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x,y,x+px,y+w/2);
		drawWhiteRectangle(x,y+w/2,x+l,y+w/2+px);
		drawWhiteRectangle(x+l-px,y+w/2,x+l,y+w);
		drawWhiteRectangle(x,y+w-px,x+l,y+w);
		// makes it different from 5
		drawWhiteRectangle(x+l-px,y,x+l,y+w/4);
		drawWhiteRectangle(x,y+w-w/4,x+px,y+w);
		break;
	case 'T':
	case 'U':
	case 'V':
	case 'W':
	case 'X':
	case 'Y':
	case 'Z':
	case '0':
		dx = 1;
		dy = 1;
		// vert lines
		drawWhiteRectangle(x,y,x+px,y+w);
		drawWhiteRectangle(x+l-px,y,x+l,y+w);
		// horiz lines
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x,y+w-px,x+l,y+w);
		for (int t=0; t<pt; t++)
			drawWhiteDiagonal(x+t,y+w,x+l,y,dx,dy);
		break;
	case '1':
		dx = 1;
		dy = 1;
		for (int t=0; t<pt; t++)
			drawWhiteDiagonal(x,y+w/4-t,x+l/2,y,dx,dy);
		drawWhiteRectangle(x+(l/2)-(px/2),y,x+(l/2)+(px/2),y+w);
		drawWhiteRectangle(x,y+w-px,x+l,y+w);
		break;
	case '2':
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x,y+w-px,x+l,y+w);
		drawWhiteRectangle(x+l-px,y,x+l,y+w/2);
		dx = 1;
		dy = 1;
		for (int t=0; t<pt; t++)
			drawWhiteDiagonal(x,y+w-t,x+l,y+w/2,dx,dy);
		break;
	case '3':
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x+l-px,y,x+l,y+w/2);
		drawWhiteRectangle(x,y+w/2,x+l,y+w/2+px);
		drawWhiteRectangle(x+l-px,y+w/2,x+l,y+w);
		drawWhiteRectangle(x,y+w-px,x+l,y+w);
		break;
	case '4':
		drawWhiteRectangle(x,y,x+px,y+w/2);
		drawWhiteRectangle(x,y+w/2,x+l,y+w/2+px);
		drawWhiteRectangle(x+l-px,y,x+l,y+w);
		break;
	case '5':
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x,y,x+px,y+w/2);
		drawWhiteRectangle(x,y+w/2,x+l,y+w/2+px);
		drawWhiteRectangle(x+l-px,y+w/2,x+l,y+w);
		drawWhiteRectangle(x,y+w-px,x+l,y+w);
		break;
	case '6':
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x,y,x+px,y+w);
		drawWhiteRectangle(x,y+w/2,x+l,y+w/2+px);
		drawWhiteRectangle(x+l-px,y+w/2,x+l,y+w);
		drawWhiteRectangle(x,y+w-px,x+l,y+w);
		break;
	case '7':
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x+l-px,y,x+l,y+w);
		break;
	case '8':
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x,y,x+px,y+w);
		drawWhiteRectangle(x,y+w/2,x+l,y+w/2+px);
		drawWhiteRectangle(x+l-px,y,x+l,y+w);
		drawWhiteRectangle(x,y+w-px,x+l,y+w);
		break;
	case '9':
		drawWhiteRectangle(x,y,x+l,y+px);
		drawWhiteRectangle(x,y,x+px,y+w/2);
		drawWhiteRectangle(x,y+w/2,x+l,y+w/2+px);
		drawWhiteRectangle(x+l-px,y,x+l,y+w);
		break;
	case '/':
		dx = 1;
		dy = 1;
		for (int t=0; t<pt; t++)
			drawWhiteDiagonal(x,y+w/2+t+2*px,x+l,y+t+2*px,dx,dy);
		break;
	case ':':
		drawWhiteRectangle(x,y+w/4,x+px,y+w/4+px);
		drawWhiteRectangle(x,y+w-w/4,x+px,y+w-w/4+px);
		break;
	default:
		break;
	}
	return x+l;
}
int drawString(int x, int y, int pt, int sp, char s[], int s_len)
{
	// sp = spacing
	int st = x;
	for (int i=0; i<s_len; i++) {
		st = drawCharacter(st, y, pt, s[i]);
		st += sp + 1;
	}
	return st;
}
int drawWhiteString(int x, int y, int pt, int sp, char s[], int s_len)
{
	// sp = spacing
	int st = x;
	for (int i=0; i<s_len; i++) {
		st = drawWhiteCharacter(st, y, pt, s[i]);
		st += sp + 1;
	}
	return st;
}
void uploadImageBuffer()
{
	int Le = 0;
	resetDataPointer();
	for (int i = 0; i*pkt_size < file_size; i++) {
		do {
			waitTCBusy();
			// Send Image Data
			HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);
			// CMD
			HAL_SPI_TransmitReceive(&hspi1, UploadImageData, rxData, 3, 1000);
			// Image Data
			HAL_SPI_TransmitReceive(&hspi1, &pkt_size, rxData, 1, 1000);
			for (int j = 0; j < pkt_size; j++)
				HAL_SPI_TransmitReceive(&hspi1, &img_buf[i*pkt_size+j], rxData, 1, 1000);
			HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_SET);
			//            printf("\n\rSend Image Data (0x200100) Sent\n\r");
		} while (readResponse(Le) == 1);
	}
}
void resetDataPointer()
{
	int Le = 0;
	waitTCBusy();
	HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, ResetDataPointer, rxData, 3, 1000);
	HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_SET);
	//    printf("\n\rReset Data Pointer (0x200D00) Sent\n\r");
	readResponse(Le);
}
void displayUpdate()
{
	int Le = 0;
	waitTCBusy();
	HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, DisplayUpdate, rxData, 3, 1000);
	HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_SET);
	//    printf("\n\rDisplay Update (0x240100) Sent\n\r");
	readResponse(Le);
}
void testDraw(int i)
{
	int loc;
	char c[10];
	setAllWhite();
	drawString(270, 10, 5, 10, itoa(i, c, 10), 4);
	itoa(realspeed,c,10);
	c[4] = 'K';
	c[5] = 'M';
	c[6] = '/';
	c[7] = 'H';
	drawString(100,100,5,10,c,10);
	batteryImage(100, 200, 10, 97);
	uploadImageBuffer();
	displayUpdate();
}
void batteryImage(int x, int y, int pt, int percent){
	drawRectangle(x, y, x+(10*pt), y+(5*pt));
	drawRectangle(x+(10*pt), y+(5*pt/2)-pt, x+(pt)+(10*pt), y+(5*pt/2)+pt);
	char c[10];
	drawWhiteString(x+(pt), y-pt+(4*pt/2), 4, pt, itoa(percent, c, 10), 3);
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
	while(1)
	{
	}
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
