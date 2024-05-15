/*
 * proximity.c
 *
 * Created: 27-04-2024 22:28:04
 *  Author: Praveen Raj U S
 */ 



/*

#include "asf.h"
#include "proximity.h"
#include "I2cDriver/I2cDriver.h"



int32_t vl6180x_init(I2C_Data *i2c_data, uint8_t sensor_id) {
	uint8_t id;
	// Setup I2C pin configuration based on sensor ID
	switch (sensor_id) {
		case 1: // Sensor 1
		i2c_data-> scl_pin = PIN_PA17;
		i2c_data-> sda_pin = PIN_PA22;
		break;
		case 2: // Sensor 2 (assuming different pins or same pins with different address)
		i2c_data-> scl_pin = PIN_PA09; // Change as necessary
		i2c_data-> sda_pin = PIN_PA08; // Change as necessary
		break;
		default:
		return -1; // Invalid sensor ID
	}

	i2c_data->address = VL6180X_IDENTIFICATION_MODEL_ID; // Assuming both sensors have the same I2C address configuration
	i2c_data->msgOut = NULL;
	i2c_data->msgIn = &id;
	i2c_data->lenIn = 1;
	i2c_data->lenOut = 0;

	if (I2cReadData(i2c_data) != 0) return -1; // Error in I2C read
	if (id != 0xB4) return -2; // Model ID does not match expected value

	return vl6180x_configure_default(i2c_data);
}


int32_t vl6180x_configure_default(I2C_Data *i2c_data) {
	// Write configuration registers
	uint8_t data[2];

	// Mandatory: Private registers setup
	data[0] = 0x0207;
	data[1] = 0x01;
	i2c_data->address = data[0];
	i2c_data->msgOut = &data[1];
	i2c_data->lenIn = 0;
	i2c_data->lenOut = 1;
	if (I2cWriteData(i2c_data) != 0) return -1;

	// Additional mandatory registers...
	// Configure public registers for typical application settings

	return 0; // Configuration successful
}

int32_t vl6180x_measure_distance(I2C_Data *i2c_data, uint8_t *distance) {
	uint8_t data = 0x01; // Command to start measurement

	// Start range measurement
	i2c_data->address = VL6180X_SYSRANGE_START;
	i2c_data->msgOut = &data;
	i2c_data->lenIn = 0;
	i2c_data->lenOut = 1;
	if (I2cWriteData(i2c_data) != 0) return -1;

	// Read range value
	i2c_data->address = VL6180X_RESULT_RANGE_VAL;
	i2c_data->msgOut = NULL;
	i2c_data->msgIn = distance;
	i2c_data->lenIn = 1;
	i2c_data->lenOut = 0;
	if (I2cReadData(i2c_data) != 0) return -2;

	// Clear interrupts by reading interrupt status
	uint8_t clear_it = 0x07;
	i2c_data->address = VL6180X_SYSTEM_INTERRUPT_CLEAR;
	i2c_data->msgOut = &clear_it;
	i2c_data->lenIn = 0;
	i2c_data->lenOut = 1;
	if (I2cWriteData(i2c_data) != 0) return -3;

	return 0; // Measurement successful
}
*/



//------------------------------------------------------------------------------------------------------------------------------------
//Attempt-2


/*
#include "proximity/proximity.h"

void vl6180x_init() {
	// Perform a system fresh out of reset
	uint8_t fresh_out_of_reset;
	I2cReadData(VL6180X_ADDR, 0x016, &fresh_out_of_reset, 1);
	if (fresh_out_of_reset == 1) {
		vl6180x_default_settings();  // Load default settings if the device is fresh out of reset
		uint8_t zero = 0;
		I2cWriteData(VL6180X_ADDR, 0x016, &zero, 1);  // Clear the fresh out of reset flag
	}

	// Mandatory : private registers
	I2cWriteData(VL6180X_ADDR, 0x0207, (uint8_t[]){0x01}, 1);
	I2cWriteData(VL6180X_ADDR, 0x0208, (uint8_t[]){0x01}, 1);
	I2cWriteData(VL6180X_ADDR, 0x0096, (uint8_t[]){0x00}, 1);
	I2cWriteData(VL6180X_ADDR, 0x0097, (uint8_t[]){0xfd}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00e3, (uint8_t[]){0x00}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00e4, (uint8_t[]){0x04}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00e5, (uint8_t[]){0x02}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00e6, (uint8_t[]){0x01}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00e7, (uint8_t[]){0x03}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00f5, (uint8_t[]){0x02}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00d9, (uint8_t[]){0x05}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00db, (uint8_t[]){0xce}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00dc, (uint8_t[]){0x03}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00dd, (uint8_t[]){0xf8}, 1);
	I2cWriteData(VL6180X_ADDR, 0x009f, (uint8_t[]){0x00}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00a3, (uint8_t[]){0x3c}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00b7, (uint8_t[]){0x00}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00bb, (uint8_t[]){0x3c}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00b2, (uint8_t[]){0x09}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00ca, (uint8_t[]){0x09}, 1);
	I2cWriteData(VL6180X_ADDR, 0x0198, (uint8_t[]){0x01}, 1);
	I2cWriteData(VL6180X_ADDR, 0x01b0, (uint8_t[]){0x17}, 1);
	I2cWriteData(VL6180X_ADDR, 0x01ad, (uint8_t[]){0x00}, 1);
	I2cWriteData(VL6180X_ADDR, 0x00ff, (uint8_t[]){0x05}, 1);
	I2cWriteData(VL6180X_ADDR, 0x0100, (uint8_t[]){0x05}, 1);
	I2cWriteData(VL6180X_ADDR, 0x0199, (uint8_t[]){0x05}, 1);
	I2cWriteData(VL6180X_ADDR, 0x01a6, (uint8_t[]){0x1b}, 1);
	I2cWriteData(VL6180X_ADDR, 0x01ac, (uint8_t[]){0x3e}, 1);
	I2cWriteData(VL6180X_ADDR, 0x01a7, (uint8_t[]){0x1f}, 1);
	delay_ms(2);  // Device stabilization delay
}

void vl6180x_default_settings() {
	// Public registers - See datasheet for more detail
	I2cWriteData(VL6180X_ADDR, 0x0011, (uint8_t[]){0x10}, 1);  // Enables polling for 'New Sample ready'
	// when measurement completes
	I2cWriteData(VL6180X_ADDR, 0x010a, (uint8_t[]){0x30}, 1);  // Set the averaging sample period
	I2cWriteData(VL6180X_ADDR, 0x003f, (uint8_t[]){0x46}, 1);  // Sets the light and dark gain (upper
	// nibble). The lower nibble sets the integration time
	I2cWriteData(VL6180X_ADDR, 0x0031, (uint8_t[]){0xFF}, 1);  // sets the # of range measurements after
	// which auto calibration of system is performed
	I2cWriteData(VL6180X_ADDR, 0x0040, (uint8_t[]){0x63}, 1);  // Set ALS integration time to 100ms
	I2cWriteData(VL6180X_ADDR, 0x002e, (uint8_t[]){0x01}, 1);  // perform a single temperature calibration
	// of the ranging sensor
}

uint8_t vl6180x_read_range() {
	uint8_t range;
	I2cReadData(VL6180X_ADDR, 0x062, &range, 1);  // Assuming register 0x062 holds the range value
	return range;
}

*/

//----------------------------------------------------------------------------------------------------------------

//Attempt-3

#include "proximity/proximity.h"
#include "I2cDriver/I2cDriver.h"
#include "SerialConsole/SerialConsole.h"
#include "SerialConsole/circular_buffer.h"

// Define the register addresses (based on the datasheet)
#define VL6180X_REG_IDENTIFICATION_MODEL_ID    0x000
#define VL6180X_REG_SYSTEM_INTERRUPT_CLEAR     0x015
#define VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET  0x016
#define VL6180X_REG_SYSRANGE_START             0x018
#define VL6180X_REG_SYSALS_START               0x038
#define VL6180X_REG_RESULT_RANGE_VAL           0x062
#define VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO 0x04f
#define VL6180X_REG_RESULT_ALS_VAL             0x050
#define VL6180X_REG_FIRMWARE_RESULT_SCALER     0x0120
#define VL6180X_RESULT_RANGE_STATUS 0x4d

static const uint8_t init_sequence[] = {
	// Mandatory: Private registers - See VL6180X datasheet for details
	0x0207, 0x01,
	0x0208, 0x01,
	0x0096, 0x00,
	0x0097, 0xfd,
	0x00e3, 0x00,
	0x00e4, 0x04,
	0x00e5, 0x02,
	0x00e6, 0x01,
	0x00e7, 0x03,
	0x00f5, 0x02,
	0x00d9, 0x05,
	0x00db, 0xce,
	0x00dc, 0x03,
	0x00dd, 0xf8,
	0x009f, 0x00,
	0x00a3, 0x3c,
	0x00b7, 0x00,
	0x00bb, 0x3c,
	0x00b2, 0x09,
	0x00ca, 0x09,
	0x0198, 0x01,
	0x01b0, 0x17,
	0x01ad, 0x00,
	0x00ff, 0x05,
	0x0100, 0x05,
	0x0199, 0x05,
	0x01a6, 0x1b,
	0x01ac, 0x3e,
	0x01a7, 0x1f,
	0x0030, 0x00,
	// ...
	0x010A, 0x30, // Set the averaging sample period (compromise between lower noise and increased execution time)
	0x003F, 0x46, // Sets the light and dark gain (upper nibble). The lower nibble sets the integration time
	0x0131, 0xFF, // sets the # of range measurements after which auto calibration of system is performed
	0x0039, 0x07, // Set ALS integration time to 100ms
	0x028A, 0x00, // Disable interleaved mode for example

	// Recommended : Public registers - See datasheet for more detail
	0x0016, 0x00, // Reset value
	// ...
	0x001B, 0x09, // Set the number of averaged samples
	0x003E, 0x31, // Set default ranging inter-measurement period to 100ms
	0x0014, 0x24, // Configures interrupt on 'New Sample Ready threshold event'
	// ...
};


#define WAIT_I2C_LINE_MS 1000

void vl6180x_init() {
	SerialConsoleWriteString("Staring Init \r\n");
	uint8_t fresh_out_of_reset;
	// Check if the device is fresh out of reset
	I2C_Data i2c_data = {
		.address = VL6180X_I2C_ADDRESS,
		.msgOut = (const uint8_t[]){VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET},
		.lenOut = 1,
		.msgIn = &fresh_out_of_reset,
		.lenIn = 1
	};
	if (I2cReadDataWait(&i2c_data, WAIT_I2C_LINE_MS, portMAX_DELAY) < 0) {
		// Handle error
		SerialConsoleWriteString("Error: Failed to read fresh-out-of-reset status \r\n");
		return;
	}

	if (fresh_out_of_reset == 1) {
		// Apply the initialization sequence
		for (size_t i = 0; i < sizeof(init_sequence); i += 2) {
			uint8_t data_to_write[] = {init_sequence[i], init_sequence[i + 1]};
			i2c_data.msgOut = data_to_write;
			i2c_data.lenOut = 2;
			i2c_data.msgIn = NULL;
			i2c_data.lenIn = 0;
			if (I2cWriteDataWait(&i2c_data, portMAX_DELAY) < 0) {
				// Handle error
				SerialConsoleWriteString("Error: Failed to write initialization sequence \r\n");
				
				return;
			}
		}
		// Clear the 'fresh out of reset' bit
		uint8_t zero = 0;
		i2c_data.msgOut = &zero;
		i2c_data.lenOut = 1;
		if (I2cWriteDataWait(&i2c_data, portMAX_DELAY) < 0) {
			// Handle error
			SerialConsoleWriteString("Error: Failed to clear fresh-out-of-reset bit \r\n");
			return;
		}
	}

	// Additional setup as required by the datasheet
	// ...

  SerialConsoleWriteString("Message: Sensor initialized successfully \r \n");
  
}






// Read a single range measurement
uint8_t vl6180x_read_range_single() {
	SerialConsoleWriteString("Reading Range \r\n");
	uint8_t range_val = 0;
	uint8_t status;
	
	// Wait for device ready
	do {
		// Read range status register
		uint8_t read_range_status_data = VL6180X_RESULT_RANGE_STATUS;
		I2C_Data status_i2c_data = {
			.address = VL6180X_I2C_ADDRESS,
			.msgOut = &read_range_status_data,
			.lenOut = 1,
			.msgIn = &status,
			.lenIn = 1
		};
		if (I2cReadDataWait(&status_i2c_data, WAIT_I2C_LINE_MS, portMAX_DELAY) < 0) {
			// Handle error
			SerialConsoleWriteString("Error: Failed to read range status register \r\n");
			return 255; // Error value
		}
		
		SerialConsoleWriteString("Range Status Register Value: \r\n");
		SerialConsoleWriteString(status);
		SerialConsoleWriteString(" \r\n");
		
	} while ((status & (1 << 0)) != 0); // Check if bit 0 indicates device is not ready

	
	
	// Start the range measurement
	uint8_t start_measurement_data[] = {VL6180X_REG_SYSRANGE_START, 0x01};
	I2C_Data i2c_data = {
		.address = VL6180X_I2C_ADDRESS,
		.msgOut = start_measurement_data,
		.lenOut = 2,
		.msgIn = NULL,
		.lenIn = 0
	};
	if (I2cWriteDataWait(&i2c_data, portMAX_DELAY) < 0) {
		// Handle error
		SerialConsoleWriteString("Error: Failed to start range measurement \r\n");
		return 255; // Error value
	}

	delay_ms(100);
	// Wait for device ready and read the range value
	uint8_t read_range_data = VL6180X_REG_RESULT_RANGE_VAL;
	i2c_data.msgOut = &read_range_data;
	i2c_data.lenOut = 1;
	i2c_data.msgIn = &range_val;
	i2c_data.lenIn = 1;
	if (I2cReadDataWait(&i2c_data, WAIT_I2C_LINE_MS, portMAX_DELAY) < 0) {
		// Handle error
		SerialConsoleWriteString("Error: Failed to read range value \r\n");
		return 255; // Error value
	}
	delay_ms(100);
	SerialConsoleWriteString(range_val);
	// Clear the interrupt
	uint8_t clear_interrupt_data[] = {VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07};
	i2c_data.msgOut = clear_interrupt_data;
	i2c_data.lenOut = 2;
	i2c_data.msgIn = NULL;
	i2c_data.lenIn = 0;
	if (I2cWriteDataWait(&i2c_data, portMAX_DELAY) < 0) {
		// Handle error
		SerialConsoleWriteString("Error: Failed to clear interrupt \r\n");
		return 255; // Error value
	}
	
	//SerialConsoleWriteString("Range Value is: \r\n");	
	char data[50];
	sprintf(data,"Range Value is : %d \r\n",range_val);
	SerialConsoleWriteString(data);
	
	return range_val;
}


// Read a single ambient light measurement
uint16_t vl6180x_read_ambient_light_single() {
	uint16_t als_val = 0;
	uint8_t status;
	// Start the ambient light measurement
	uint8_t start_ambient_light_measurement_data[] = {VL6180X_REG_SYSALS_START, 0x01};
	I2C_Data i2c_data = {
		.address = VL6180X_I2C_ADDRESS,
		.msgOut = start_ambient_light_measurement_data,
		.lenOut = 2,
		.msgIn = NULL,
		.lenIn = 0
	};
	if (I2cWriteDataWait(&i2c_data, portMAX_DELAY) < 0) {
		// Handle error
		return 0xFFFF; // Error value
	}

	// Wait for device ready and read the ALS value
	uint8_t read_als_data = VL6180X_REG_RESULT_ALS_VAL;
	i2c_data.msgOut = &read_als_data;
	i2c_data.lenOut = 1;
	i2c_data.msgIn = (uint8_t *)&als_val;
	i2c_data.lenIn = 2;
	if (I2cReadDataWait(&i2c_data, WAIT_I2C_LINE_MS, portMAX_DELAY) < 0) {
		// Handle error
		return 0xFFFF; // Error value
	}

	// Clear the interrupt
	uint8_t clear_interrupt_data[] = {VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07};
	i2c_data.msgOut = clear_interrupt_data;
	i2c_data.lenOut = 2;
	i2c_data.msgIn = NULL;
	i2c_data.lenIn = 0;
	if (I2cWriteDataWait(&i2c_data, portMAX_DELAY) < 0) {
		// Handle error
		return 0xFFFF; // Error value
	}

	return als_val;
}
