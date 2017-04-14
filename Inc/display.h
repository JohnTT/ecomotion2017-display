/*
 * display.h
 *
 *  Created on: Mar 19, 2017
 *      Author: Constellations
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

// AllCell BMS CAN Bus

/*
 * Use this function to parse CAN byte stream into structures.
 * memcpy(&parsedData, data, sizeof(data));
 */
typedef enum {
	AllCell_Bat_State_ID = 0x0CFF2020,
	AllCell_Bat_Info_ID = 0x0CFF2120,
	AllCell_Bat_Current_ID = 0x18FF8020,
	AllCell_Bat_Voltage_ID =  0x18FF8120,
	AllCell_Bat_Temperature_ID = 0x18FF8220,
	AllCell_Bat_Status_ID =  0x18FF8320,
	AllCell_Bat_PwAvailable_ID = 0x18FF8420,
	AllCell_Bat_RTC_ID = 0x18FFD020
} AllCell_BMS_CAN_ID;

typedef struct {
	uint8_t State;
	uint16_t Fault_Code;
	uint16_t Timer;
	uint16_t Reset;
	uint8_t Balance;
} AllCell_Bat_State;

typedef struct {
	uint16_t Current;
	uint16_t Voltage;
	uint8_t Temp;
	uint16_t Impedance;
} AllCell_Bat_Info;

typedef struct {
	uint16_t Current;
	uint16_t Charge_Limit;
	uint16_t Discharge_Limit;
} AllCell_Bat_Current;

typedef struct {
	uint16_t Voltage;
	uint16_t Min_Cell_Voltage;
	uint8_t Nb_Min_Voltage;
	uint16_t Max_Cell_Voltage;
	uint8_t Nb_Max_Voltage;
} AllCell_Bat_Voltage;

typedef struct {
	uint8_t Temp_BMS;
	uint8_t Avg_Cell_Temp;
	uint8_t Min_Cell_Temp;
	uint8_t Nb_Min_Temp;
	uint8_t Max_Cell_Temp;
	uint8_t Nb_Max_Temp;
	// uint8_t Temp_Ambient; // Unused for this BMS version
} AllCell_Bat_Temperature;

typedef struct {
	uint8_t SOC;
	uint16_t Capacity;
	// uint8_t SOH; // Unused for this BMS version
} AllCell_Bat_Status;

typedef struct {
	uint32_t PwAvailable_Charge;
	uint32_t PwAvailable_Disharge;
} AllCell_Bat_PwAvailable;

typedef struct {
	uint8_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
} AllCell_Bat_RTC;


//-----------------------------




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
    ecoMotion_Display = 0x40,
	ecoMotion_Error_Throttle = 0xFFF,
	ecoMotion_Error_Master = 0x0FEF,
	ecoMotion_Error_Display = 0xFDF,s
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
void batteryImage(int x, int y, int size, int fontSize, int fontSpacing, int percent);
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
