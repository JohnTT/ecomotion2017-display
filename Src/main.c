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
#include <string.h>
#include "display.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
AllCell_Bat_RTC displayRTC;
displayBMSTypeDef displayBMS;

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

//Adafruit 7-seg variables
uint8_t i2c_addr_7seg = 0x70 << 1;
uint16_t displaybuffer_7seg[8];
uint8_t position_7seg = 0;
static const uint8_t numbertable[] = {
		0x3F, /* 0 */
		0x06, /* 1 */
		0x5B, /* 2 */
		0x4F, /* 3 */
		0x66, /* 4 */
		0x6D, /* 5 */
		0x7D, /* 6 */
		0x07, /* 7 */
		0x7F, /* 8 */
		0x6F, /* 9 */
		0x77, /* a */
		0x7C, /* b */
		0x39, /* C */
		0x5E, /* d */
		0x79, /* E */
		0x71, /* F */
};

const uint16_t i2c_timeout = 1000;

//cmds
uint8_t i2c_cmd_oscOn = 0x21;

int counter = 0;
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
static void MX_I2C1_Init(void);

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


	/* USER CODE BEGIN 2 */
#ifdef _DEBUG_ON
	MX_GPIO_Init();
	//MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
//	MX_CAN_Init();
	//MX_SPI1_Init();
	//MX_TIM1_Init();
	MX_I2C1_Init();
#else
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_CAN_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();

	HAL_TIM_Base_Start_IT(&htim1); //start the base for update interrupts
	flag = 0;
	updateBufferDMA();
#endif

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

//		flag = 0;
		//		counter += 11;
		//		counter %= 100;
		//      must find a way to make waitTCBusy without polling, somehow read a GPIO port with interrupts
		//      check the waitTCBusy in the interrupt callback, if it's ready, send the next package of data
		//      using some function 'partitioning' if not, continue back to communicating to the LCD display
		//      and try again in a bit.
		//      Need to add a timer with interrupt call checks
		//
		printf("MAIN LOOP DISPLAY\n\r");
//		printUART2();
//		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
//		for (int i = 0; i < 1000; i++) {}
//		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);



		counter++;
		test_7seg();
		HAL_Delay(100);
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
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

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
			|RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
//static void MX_CAN_Init(void)
//{
//
//  hcan.Instance = CAN;
//  hcan.Init.Prescaler = 2;
//  hcan.Init.Mode = CAN_MODE_NORMAL;
//  hcan.Init.SJW = CAN_SJW_1TQ;
//  hcan.Init.BS1 = CAN_BS1_13TQ;
//  hcan.Init.BS2 = CAN_BS2_2TQ;
//  hcan.Init.TTCM = DISABLE;
//  hcan.Init.ABOM = DISABLE;
//  hcan.Init.AWUM = DISABLE;
//  hcan.Init.NART = DISABLE;
//  hcan.Init.RFLM = DISABLE;
//  hcan.Init.TXFP = DISABLE;
//  if (HAL_CAN_Init(&hcan) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2000090E;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
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
//static void MX_TIM1_Init(void)
//{
//
//  TIM_ClockConfigTypeDef sClockSourceConfig;
//  TIM_MasterConfigTypeDef sMasterConfig;
//
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 6400;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 10000;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
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

	//	char c[10];
	//	const static uint8_t batteryX, batteryY;
	//	const static uint8_t rearSpeedX, rearSpeedY;
	//	const static uint8_t bmsCurrentX, bmsCurrentY;
	//	const static uint8_t bmsVoltageX, bmsVoltageY;
	//	const static uint8_t bmsTemperatureX, bmsTemperatureY;
	//
	//	setAllWhite();
	//
	//	// Rear Wheel Speed
	//	itoa(realspeed,c,10);
	//	drawString(DISP_BLACK, 100,100,5,10,c,10);
	//	c[0] = 'K';
	//	c[1] = 'M';
	//	c[2] = '/';
	//	c[3] = 'H';
	//	c[4] = '\0';
	//	drawString(DISP_BLACK, 150,122,3,5,c,4);
	//
	//	// BMS Current Draw
	//
	//	// BMS Voltage
	//
	//	// BMS Temperature - TO DO Warning Indicator
	//
	//	// Battery Percentage/State of Charge
	//	int percentLoc = batteryImage(15, 225, 60, 4, 2, uint8_t(displayBMS.bat_percentage));
	//	batteryLife += 1;
	//	if (batteryLife > 100)
	//		batteryLife = 0;
	//
	//	// Real Time Clock - Current Time
	//
	//

	// Screensaver Image
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

//Functions related to controlling the Adafruit 1.2" Seven Segment
void begin_7seg() {
	if (HAL_I2C_Master_Transmit(&hi2c1, i2c_addr_7seg, &i2c_cmd_oscOn, 1, i2c_timeout) == HAL_OK)
		printf("We good\n\r");
	else
		printf("We no good\n\r");
	blinkRate_7seg(HT16K33_BLINK_OFF);

	setBrightness_7seg(5); // max brightness is 15
}
void blinkRate_7seg(uint8_t b) {
	if (b > 3)
		b = 0; // turn off if not sure

	uint8_t txMsg = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1);
	HAL_I2C_Master_Transmit(&hi2c1,i2c_addr_7seg,&txMsg, 1, i2c_timeout);
}
void setBrightness_7seg(uint8_t b) {
	if (b > 15)
		b = 15;
	uint8_t txMsg = HT16K33_CMD_BRIGHTNESS | b;
	HAL_I2C_Master_Transmit(&hi2c1,i2c_addr_7seg,&txMsg, 1, i2c_timeout);
}

// print_7seg
void print_7seg(double n)
{
	int digits = 2;
	printFloat_7seg(n, digits, DEC);
}
void printNumber_7seg(long n, uint8_t base)
{
	printFloat_7seg(n, 0, base);
}
void printFloat_7seg(double n, uint8_t fracDigits, uint8_t base)
{
	uint8_t numericDigits = 4;   // available digits on display
	uint8_t isNegative = false;  // true if the number is negative

	// is the number negative?
	if(n < 0) {
		isNegative = true;  // need to draw sign later
		--numericDigits;    // the sign will take up one digit
		n *= -1;            // pretend the number is positive
	}

	// calculate the factor required to shift all fractional digits
	// into the integer part of the number
	double toIntFactor = 1.0;
	for(int i = 0; i < fracDigits; ++i) toIntFactor *= base;

	// create integer containing digits to display by applying
	// shifting factor and rounding adjustment
	uint32_t displayNumber = n * toIntFactor + 0.5;

	// calculate upper bound on displayNumber given
	// available digits on display
	uint32_t tooBig = 1;
	for(int i = 0; i < numericDigits; ++i) tooBig *= base;

	// if displayNumber is too large, try fewer fractional digits
	while(displayNumber >= tooBig) {
		--fracDigits;
		toIntFactor /= base;
		displayNumber = n * toIntFactor + 0.5;
	}

	// did toIntFactor shift the decimal off the display?
	if (toIntFactor < 1) {
		printError_7seg();
	} else {
		// otherwise, display the number
		int8_t displayPos = 4;

		if (displayNumber)  //if displayNumber is not 0
		{
			for(uint8_t i = 0; displayNumber || i <= fracDigits; ++i) {
				uint8_t displayDecimal = (fracDigits != 0 && i == fracDigits);
				writeDigitNum_7seg(displayPos--, displayNumber % base, displayDecimal);
				if(displayPos == 2) writeDigitRaw_7seg(displayPos--, 0x00);
				displayNumber /= base;
			}
		}
		else {
			writeDigitNum_7seg(displayPos--, 0, false);
		}

		// display negative sign if negative
		if(isNegative) writeDigitRaw_7seg(displayPos--, 0x40);

		// clear remaining display positions
		while(displayPos >= 0) writeDigitRaw_7seg(displayPos--, 0x00);
	}
}
void writeDigitRaw_7seg(uint8_t d, uint8_t bitmask) {
	if (d > 4) return;
	displaybuffer_7seg[d] = bitmask;
}
void writeDigitNum_7seg(uint8_t d, uint8_t num, uint8_t dot) {
	if (d > 4)
		return;

	writeDigitRaw_7seg(d, numbertable[num] | (dot << 7));
}
size_t write_7seg(uint8_t c) {

	uint8_t r = 0;

	if (c == '\n') position_7seg = 0;
	if (c == '\r') position_7seg = 0;

	if ((c >= '0') && (c <= '9')) {
		writeDigitNum_7seg(position_7seg, c-'0', false);
		r = 1;
	}

	position_7seg++;
	if (position_7seg == 2) position_7seg++;

	return r;
}
void writeDisplay_7seg(void) {

	// start at address $00
	uint8_t txMsg[17];
	txMsg[0] = 0x00;


	for (uint8_t i=0; i<8; i++) {
		txMsg[1+2*i] = displaybuffer_7seg[i] & 0xFF;
		txMsg[1+2*i+1] = displaybuffer_7seg[i] >> 8;
	}
	HAL_I2C_Master_Transmit(&hi2c1, i2c_addr_7seg, txMsg, 17, i2c_timeout);
}
void printError_7seg(void) {
	for(uint8_t i = 0; i < SEVENSEG_DIGITS; ++i) {
		writeDigitRaw_7seg(i, (i == 2 ? 0x00 : 0x40));
	}
}
void test_7seg() {
	begin_7seg();
	print_7seg(2.0);
	writeDisplay_7seg();
	return;
}

// Other stuff
int getTim1Prescaler(){
	return HAL_RCC_GetPCLK2Freq() / 5000; //since it is multiplied by 2
}
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

	HAL_GPIO_WritePin(LEDx_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
#ifdef _CAN_PRINTF
	uint32_t ID;
	if (theHcan->pRxMsg->IDE == CAN_ID_EXT)
		ID = theHcan->pRxMsg->ExtId;
	else
		ID = theHcan->pRxMsg->StdId;
	printf("Message Received and Acknowledged from ID %x\n\r", ID);
#endif


	if (theHcan->pRxMsg->IDE == CAN_ID_STD) {
		parseCANMessage(theHcan->pRxMsg);
	}


	HAL_GPIO_WritePin(LEDx_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
	if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK) {
		Error_Handler();
	}

}
void parseCANMessage(CanRxMsgTypeDef *pRxMsg) {
	static const float _Current_Factor = 0.05;
	static const float _Voltage_Factor = 0.05;
	static const float _Impedance_Factor = 0.01;
	static const float _CellVolt_Factor = 0.01;
	static const float _Day_Factor = 0.25;
	static const float _Second_Factor = 0.25;
	static const float _SOC_Factor = 0.5;
	static const float _Capacity_Factor = 0.01;


	static const uint16_t _Current_Offset = 1600;
	static const uint16_t _Temp_Offset = 40;
	static const uint32_t _PwAvailable_Offset = 2000000000;
	static const float _Year_Offset = 1985;

	masterCAN1_BMSTypeDef bmsBuf;

	switch (pRxMsg->StdId) {
	case ecoMotion_MasterBMS:
		memcpy(&bmsBuf, pRxMsg->Data, sizeof(bmsBuf));
		displayBMS.current = bmsBuf.current *_Current_Factor - _Current_Offset;
		displayBMS.voltage = bmsBuf.voltage *_Voltage_Factor;
		displayBMS.temperature = bmsBuf.temperature - _Temp_Offset;
		displayBMS.bat_percentage = bmsBuf.bat_percentage * _SOC_Factor;
		break;
	case ecoMotion_MasterRTC:
		memcpy(&displayRTC, pRxMsg->Data, sizeof(displayRTC));
		displayRTC.Day *= _Day_Factor;
		displayRTC.Second *= _Second_Factor;
		break;
	default:
		break;
	}

	// modifications to buffer will happen before we do the reDraw.
	doReDraw = 1;
}

#ifdef _DEBUG_ON
void __io_putchar(uint8_t ch) {
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}
#endif
void printUART2() {
	// Bat State - Need to be started
	printf("Current: %f [Amps]\n\r", displayBMS.current);
	printf("Voltage: %f [Volts]\n\r", displayBMS.voltage);
	printf("Temperature: %u [deg C]\n\r", displayBMS.temperature);
	printf("Battery: %f [%%]\n\r", displayBMS.bat_percentage);


	printf("RTC MESSAGE ---------------\n\r");
	printf("Year: %u\n\r", displayRTC.Year+1985);
	printf("Month: %u\n\r", displayRTC.Month);
	printf("Day: %u\n\r", displayRTC.Day);
	printf("Hour: %u\n\r", displayRTC.Hour);
	printf("Minute: %u\n\r", displayRTC.Minute);
	printf("Second: %u\n\r", displayRTC.Second);
	printf("\n\r");

}
static void MX_TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = getTim1Prescaler();
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 10000;
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
static void MX_CAN_Init(void)
{
	__HAL_RCC_CAN1_CLK_ENABLE();
	hcan.Instance = CAN;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	setCANbitRate(500, 32, &hcan);
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

static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

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


	//Error Handler LEDs
	GPIO_InitStruct.Pin = LED0_Pin | LED1_Pin | LED2_Pin | LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LEDx_GPIO_Port, &GPIO_InitStruct);
}

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
	printf("Error Handler\n\r");
	HAL_StatusTypeDef status;
	do {
		hcan.pTxMsg->IDE = CAN_ID_EXT;
		hcan.pTxMsg->RTR = CAN_RTR_DATA;
		hcan.pTxMsg->StdId = ecoMotion_Error_Display;
		hcan.pTxMsg->DLC = 0;

		printf("Error Handler - Display - CAN1 ID: %lx\n\r", hcan.pTxMsg->ExtId);

#ifdef _ERRORHANDLER_CAN1TRANSMIT
		status = HAL_CAN_Transmit_IT(&hcan);
#endif
		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		for (int i = 0; i < 1000; i++) {}
		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

		HAL_Delay(100);
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
