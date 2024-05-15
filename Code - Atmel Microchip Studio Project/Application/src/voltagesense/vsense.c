/*
 * vsense.c
 *
 * Created: 27-04-2024 22:28:04
 *  Author: Praveen Raj U S
 */ 




#include "asf.h"
#include "vsense.h"
#include "SerialConsole/SerialConsole.h"

#include "adc.h"
#include "conf_board.h"
#include "conf_clocks.h"
#include "buzzer/buzzer.h"


#define STATUS_PIN PIN_PB22

struct adc_module adc_instance;
bool isCharging;


void configure_adc(void) {
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);
	config_adc.gain_factor = ADC_GAIN_FACTOR_DIV2;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
	config_adc.reference = ADC_REFERENCE_INTVCC1; // Internal VCC1 (1/1.48 VCCANA)
	config_adc.positive_input = ADC_POSITIVE_INPUT_PIN0; // PA02 is AIN0 on this board
	config_adc.resolution = ADC_RESOLUTION_12BIT;

	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
}




void configure_gpio(void) {
	
	struct port_config config_port_pin;//Define structure needed to configure a pin
	port_get_config_defaults(&config_port_pin); //Initialize structure with default configurations.
	config_port_pin.direction = PORT_PIN_DIR_INPUT; //Set pin as INPUT
	port_pin_set_config(STATUS_PIN, &config_port_pin); //We assign the pin configuration to the PIN_PB02


	//port_pin_set_output_level(PIN_PA10, true);
	
	while (true)
	{
		isCharging = port_pin_get_input_level(STATUS_PIN);
		delay_ms(1000);
		if (isCharging == true)
		{
			SerialConsoleWriteString("Charging.. \r\n");
			delay_ms(100);
		}
		else
		{
			SerialConsoleWriteString("Not Charging.. \r\n");
			delay_ms(100);
			play_simple_tune();
			delay_ms(100);
		}
		delay_ms(1000);

	}
	//SerialConsoleWriteString(isCharging);
		
	
}



void read_adc(void) {
	uint16_t result;
	adc_start_conversion(&adc_instance);
	adc_read(&adc_instance, &result);

	// Convert ADC result to voltage
	// Assuming VCCANA is 3.3V and using 12-bit resolution
	float voltage = (float)result * (3.3f / 4096.0f);

	// Print the voltage to the console (use printf or similar in your environment)
	char vdata[50];
	
	sprintf(vdata,"Voltage Value is : %f \r\n",voltage);
	SerialConsoleWriteString(vdata);
	//SerialConsoleWriteString(voltage);
	


	// Determine the charging status based on the voltage
	if (voltage >= 4.5f && voltage <= 4.6f) {
		
		SerialConsoleWriteString("Charging.. \r\n");
		//printf("Charging\n");
		} else if (voltage >= 4.8f && voltage <= 4.9f) {
			
		SerialConsoleWriteString("Not Charging.. \r\n");	
		//printf("Not Charging\n");
		} else {
		SerialConsoleWriteString("Unknown State");
		//printf("Unknown state\n");
	}
}



void volt(void) {
	
	configure_adc();

	while (true) {
		read_adc();
		delay_ms(1000); // Delay for a second
	}

	//return 0;
}


