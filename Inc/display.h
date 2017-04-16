/*
 * display.h
 *
 *  Created on: Mar 19, 2017
 *      Author: Constellations
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

// #define _CAR 1

#ifdef _CAR

#define _ERRORHANDLER_CAN1TRANSMIT
#define _REBROADCAST_ALLCELL

#endif

#ifndef _CAR

#define _DEBUG_ON 1

#ifdef _DEBUG_ON
#define _CAN_PRINTF 1
#endif

#endif

typedef enum dispColour_t {
	DISP_BLACK,
	DISP_WHITE,
	DISP_INVERT
} dispColour;
typedef enum DMA_sections_t {
	RESET_DATA_POINTER,
	UPLOAD_IMAGE_DATA,
	DATA_SIZE,
	SEND_PACKET,
	GET_RESPONSE,
	READ_RESPONSE,
	DISPLAY_IMAGE,
	UPDATE_BUFFER
} DMA_sections;

typedef enum {
	ecoMotion_MotorControl = 0x01,
	ecoMotion_Speed = 0x02,
	ecoMotion_FrontWheels = 0x03,
	ecoMotion_Master_BMS = 0x04,
	ecoMotion_Humidity = 0x05,
	ecoMotion_Temperature = 0x06,
	ecoMotion_Throttle = 0x20,
	ecoMotion_Master = 0x30,
	ecoMotion_MasterBMS = 0x31,
	ecoMotion_MasterRTC = 0x3A,
	ecoMotion_Display = 0x40,
	ecoMotion_Error_Throttle = 0xFFF,
	ecoMotion_Error_Master = 0x0FEF,
	ecoMotion_Error_Display = 0xFDF,
} CAN_DEVICE_ID;

typedef struct {
	uint16_t current;
	uint16_t voltage;
	uint8_t temperature;
	uint8_t bat_percentage;
} masterCAN1_BMSTypeDef;

typedef struct {
	double current;
	double voltage;
	uint8_t temperature;
	double bat_percentage;
} displayBMSTypeDef;

typedef struct {
	uint8_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
} AllCell_Bat_RTC;

static void setCANbitRate(uint16_t bitRate, uint16_t periphClock, CAN_HandleTypeDef* theHcan);
void parseCANMessage(CanRxMsgTypeDef *pRxMsg);
void init();
void waitTCBusy();
int readResponse(int Le);
void readStrResponse();
void clearImageBuffer();
void setAllBlack();
void setAllWhite();
void setXY(dispColour colour, int x, int y);
void drawRectangle(dispColour colour, int X1, int Y1, int X2, int Y2);
void clearRectangle(int X1, int Y1, int X2, int Y2);
void drawDiagonal(dispColour colour, int X1, int Y1, int X2, int Y2, int dx, int dy);
int drawCharacter(dispColour colour, int x, int y, int pt, char c);
int drawString(dispColour colour, int x, int y, int pt, int sp, char s[], int size);
int batteryImage(int x, int y, int size, int fontSize, int fontSpacing, int percent);
void testDraw();
void delay(int x);
void screenSaverImage();
// Functions defined for the TCM 441-230 using Polling
void uploadImageBuffer();
void resetDataPointer();
void displayUpdate();
void getDeviceInfo();
void getDeviceID();
void getSystemInfo();
void getSystemVersionCode();
void readSensorData();

//Functions defined for the TCM 441-230 using DMA
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void dmaImageBufferSection();
uint8_t checkDMA_TCBusy();
int readDMAResponse();
void updateBufferDMA();
#endif /* DISPLAY_H_ */
