/*
 * proximity.h
 *
 * Created: 27-04-2024 20:46:15
 *  Author: Praveen Raj U S
 */ 

/*
#ifndef PROXIMITY_H_
#define PROXIMITY_H_

#include "asf.h"

// Define any constants specific to the VL6180X that are used globally
#define VL6180X_IDENTIFICATION_MODEL_ID  0x000 // Update this with the correct register address
#define VL6180X_SYSRANGE_START           0x018
#define VL6180X_RESULT_RANGE_VAL         0x062
#define VL6180X_SYSTEM_INTERRUPT_CLEAR   0x015

// I2C data structure used for communication with the VL6180X sensors
typedef struct {
	uint8_t address;        // Address of the I2C device
	const uint8_t *msgOut;  // Pointer to data to be written
	uint8_t *msgIn;         // Pointer to data buffer for read
	uint16_t lenIn;         // Length of data to read
	uint16_t lenOut;        // Length of data to write
	uint32_t scl_pin;       // SCL pin number
	uint32_t sda_pin;       // SDA pin number
	uint8_t sensor_id;      // Sensor identifier
} I2C_Data;

// Function declarations
int32_t vl6180x_init(I2C_Data *i2c_data, uint8_t sensor_id);
int32_t vl6180x_configure_default(I2C_Data *i2c_data);
int32_t vl6180x_measure_distance(I2C_Data *i2c_data, uint8_t *distance);



#endif /* PROXIMITY_H_ */


//----------------------------------------------------------------------------------------------------------
//Attempt-2

#ifndef PROXIMITY_H_
#define PROXIMITY_H_

#include "I2cDriver.h"  // Your custom I2C driver
#include "delay.h"      // ASF delay header

#define VL6180X_ADDR 0x29
#define VL6180X_I2C_ADDRESS 0x29

// Function prototypes

//void vl6180x_init(void);
//uint8_t vl6180x_read_range(void);
//void vl6180x_default_settings(void);
//----
void vl6180x_init(void);
uint8_t vl6180x_read_range_single(void);
uint16_t vl6180x_read_ambient_light_single(void);

#endif // PROXIMITY_H_


