/*
 * buzzer.c
 *
 * Created: 27-04-2024 22:28:04
 *  Author: Praveen Raj U S
 */ 
#include "asf.h"

int buzzer_blip(void);
int buzzer_open(void);

//Pin is PA10
//Pin was PB02


/**************************************************************************//**
 * @fn         int buzzer_blip(void)
 * @brief      Function to generate a short blip sound using a buzzer
 * @return     None
 * @note       This function configures PIN_PB02 as output, sets the output level to
 *             high for 300 milliseconds, and then sets it to low for 100 milliseconds.
 *****************************************************************************/

int buzzer_blip(void){
	struct port_config config_port_pin;//Define structure needed to configure a pin
	port_get_config_defaults(&config_port_pin); //Initialize structure with default configurations.
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT; //Set pin as OUTPUT
	port_pin_set_config(PIN_PA10, &config_port_pin); //We assign the pin configuration to the PIN_PB02


	port_pin_set_output_level(PIN_PA10, true);
	delay_ms(10000);
	port_pin_set_output_level(PIN_PA10, false);
	delay_ms(1000);

}


/**************************************************************************//**
 * @fn        int buzzer_open(void)
 * @brief     Function to generate a longer sound using a buzzer
 * @return    None
 * @note      This function configures PIN_PB02 as output, sets the output level to
 *            high for 2000 milliseconds, and then sets it to low.
 *****************************************************************************/

int buzzer_open(void){
	struct port_config config_port_pin;//Define structure needed to configure a pin
	port_get_config_defaults(&config_port_pin); //Initialize structure with default configurations.
	
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT; //Set pin as OUTPUT
	port_pin_set_config(PIN_PA10, &config_port_pin); //We assign the pin configuration to the PIN_PB02



	port_pin_set_output_level(PIN_PA10, true);
	delay_ms(20000);
	port_pin_set_output_level(PIN_PA10, false);
	delay_ms(500);

}



/**************************************************************************//**
 * @fn        void play_simple_tune(void)
 * @brief     Function to generate a tone using a buzzer
 * @return    None
 * @note      This function configures PIN_PB02 as output, sets the output level to
 *            high for 2000 milliseconds, and then sets it to low.
 *****************************************************************************/


void play_simple_tune(void) {
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA10, &config_port_pin);

	// Play a simple tune: multiple frequencies by changing delay
	for (int i = 0; i < 10; i++) {
		// High frequency tone
		port_pin_set_output_level(PIN_PA10, true);
		delay_ms(100);  // Adjust delay for tone pitch
		port_pin_set_output_level(PIN_PA10, false);
		delay_ms(100);  // Adjust delay for tone pitch

		// Low frequency tone
		port_pin_set_output_level(PIN_PA10, true);
		delay_ms(200);  // Adjust delay for tone pitch
		port_pin_set_output_level(PIN_PA10, false);
		delay_ms(200);  // Adjust delay for tone pitch
	}
}
