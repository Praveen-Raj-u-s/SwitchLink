/*
 * relay.c
 *
 * Created: 27-04-2024 22:28:04
 *  Author: Praveen Raj U S
 */ 
#include "asf.h"


int relay_open(void);


/**************************************************************************//**
 * @fn        int relay_open(void)
 * @brief     Function to Turn the Relay On
 * @return    None
 * @note      This function configures PIN_PB02 as output, sets the output level to
 *            high for 2000 milliseconds, and then sets it to low.
 *****************************************************************************/

int relay_open(void){
	struct port_config config_port_pin;//Define structure needed to configure a pin
	port_get_config_defaults(&config_port_pin); //Initialize structure with default configurations.
	
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT; //Set pin as OUTPUT
	port_pin_set_config(PIN_PB03, &config_port_pin); //We assign the pin configuration to the PIN_PB02



	port_pin_set_output_level(PIN_PB03, true);
	delay_ms(2000);
	port_pin_set_output_level(PIN_PB03, false);
	delay_ms(1000);
	//return 0;

}