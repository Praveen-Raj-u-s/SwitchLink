/**************************************************************************/ /**
 * @file      ButtonTask.h
 * @brief     Button Task Header File
 * @author    Team 21
 * @date      2024-03-29

 ******************************************************************************/
#ifndef BUTTON_H_INCLUDED
#define BUTTON_H_INCLUDED

#include <stdbool.h>

typedef enum buttonControlStates {
    BUTTON_RELEASED,    ///< BUTTON IS RELEASED
    BUTTON_PRESSED,     ///< BUTTON IS PRESSED
    BUTTON_MAX_STATES   /// Max number of states.
} buttonControlStates;

void InitButton(void);
bool ButtonPressed(void);

#endif
