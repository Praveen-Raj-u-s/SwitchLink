/**************************************************************************/ /**
 * @file      ButtonTask.c
 * @brief     Button Task to check the SW0 pin config
 * @author    Team 21
 * @date      2024-03-29

 ******************************************************************************/
#include "ButtonTask.h"
#include "asf.h"

void InitButton(void) {
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	pin_conf.direction = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(BUTTON_0_PIN, &pin_conf);
}

bool ButtonPressed(void) {
	// Active Low configuration
	return port_pin_get_input_level(BUTTON_0_PIN) == false;
}
