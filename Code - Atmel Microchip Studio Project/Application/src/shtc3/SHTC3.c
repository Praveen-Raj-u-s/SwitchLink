#include "shtc3/SHTC3.h"

/* I2C Config */
/************************/

uint8_t dataIn[6];
static uint8_t dataWake[2]= {0x35, 0x17};
static uint8_t dataSleep[2]= {0xB0, 0x98};
static uint8_t dataRead[2]= {0x58, 0xE0};
	
I2C_Data tempData;

/* Function Definition */
/************************/

static int32_t SHTC3_WakeWrite(uint8_t *setData, uint16_t length)
{	
	tempData.address = SHTC3_ADDR;
	tempData.lenOut = length;
	tempData.msgOut = setData;
	tempData.msgIn = NULL;
	tempData.lenIn = 0;
	return I2cWriteDataWait(&tempData, 10);
	 
}

static int32_t SHTC3_DataRead(uint8_t *setData, uint16_t length)
{
	tempData.address = SHTC3_ADDR;
	tempData.lenOut = length;
	tempData.msgOut = setData;
	tempData.msgIn = dataIn;
	tempData.lenIn = 6;
	return I2cReadDataWait(&tempData, 210, 10);
	
}

/**
 * @fn		int SHTC3_Measure(uint16_t *temperature, uint16_t *humidity)
 * @brief	Function to read the temperature and humidity from the SHTC3 sensor
 * @param[out]	temperature Pointer to a variable that will hold the temperature data
 * @param[out]	humidity Pointer to a variable that will hold the humidity data
 * @return		Returns SHT3_OK if the measurement was successful, SHT3_COMM_ERROR otherwise
 * @note
 */

int SHTC3_Measure(uint16_t *temperature)
{
	
	SHTC3_WakeWrite(dataWake, 2);
	SHTC3_DataRead(dataRead, 2);	
	SHTC3_DataRead(dataSleep, 2);
		
	temp = (dataIn[3] << 8) | dataIn[4] ;
	
	return temperature[1] = -45 +175 * temp/65536;

}