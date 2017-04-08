/*
 * display.h
 *
 *  Created on: Mar 19, 2017
 *      Author: Constellations
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

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
	ecoMotion_Throttle = 0x00,
	ecoMotion_Master = 0x10,
	ecoMotion_Display = 0x20,
	ecoMotion_Error = 0xFF
} CAN_DEVICE_ID;

static void setCANbitRate(uint16_t bitRate, uint16_t periphClock, CAN_HandleTypeDef* theHcan);
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
