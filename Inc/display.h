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
void batteryImage(int x, int y, int size, int fontSize, int fontSpacing, int percent);
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

#endif /* DISPLAY_H_ */
