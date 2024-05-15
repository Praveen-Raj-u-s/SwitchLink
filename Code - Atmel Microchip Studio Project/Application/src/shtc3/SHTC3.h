#include "asf.h"
#include "main.h"
#include "SerialConsole/SerialConsole.h"
#include "I2cDriver/I2cDriver.h"

/* Variables */
/************************/
uint16_t temp;

/* Macros */
/************************/

#define SHTC3_ADDR 0x70
#define SHT3_OK 0  ///< Returns that the sensor had no error

/* Function Prototypes                                                  
/************************/

static int32_t SHTC3_WakeWrite(uint8_t *setData, uint16_t length);
static int32_t SHTC3_DataRead(uint8_t *setData, uint16_t length);
int SHTC3_Measure(uint16_t *temperature);
