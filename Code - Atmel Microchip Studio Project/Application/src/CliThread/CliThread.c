/**
* @file      CliThread.c
* @brief     File for the CLI Thread handler. Uses FREERTOS + CLI
* @author    Eduardo Garcia
* @date      2020-02-15

**************************/

/**************************
 * Includes
 **************************/
#include "CliThread.h"
#include "string.h"
#include "imu/lis2dh12_reg.h"
#include "shtc3/SHTC3.h"
#include "I2cDriver/I2cDriver.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "buzzer/buzzer.h"
#include "relay/relay.h"
#include "proximity/proximity.h"
#include "voltagesense/vsense.h"

/**************************
 * Defines
 **************************/
#define version "v1.0"
#define IMU_ADDRESS 0x68

/**************************
 * Variables
 **************************/
static const char pcWelcomeMessage[] = "FreeRTOS CLI.\r\nType Help to view a list of registered commands.\r\n";
extern char latestRx;

static const CLI_Command_Definition_t xOTAUCommand = {"fw", "fw: Download a file and perform an FW update\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_OTAU, 0};
static const CLI_Command_Definition_t xResetCommand = {"reset", "reset: Resets the device\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_ResetDevice, 0};
static const CLI_Command_Definition_t xI2cScan = {"i2c", "i2c: Scans I2C bus\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_i2cScan, 0};
const CLI_Command_Definition_t xClearScreen = {CLI_COMMAND_CLEAR_SCREEN, CLI_HELP_CLEAR_SCREEN, CLI_CALLBACK_CLEAR_SCREEN, CLI_PARAMS_CLEAR_SCREEN};
static const CLI_Command_Definition_t xVerisonCommand = {"version","version: Latest fw version\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_Version, 0};
static const CLI_Command_Definition_t xTicksCommand = {"ticks","ticks: Number of ticks elapsed \r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_Ticks, 0};
static const CLI_Command_Definition_t xTemp = {"temp", "temp: Temperature value from SHTC3\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_temp, 0};
static const CLI_Command_Definition_t xImu = {"imu", "imu: Acceleration Values\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_imu, 0};
static const CLI_Command_Definition_t xBuzzerCommand = {"buzzer", "buzzer: Sets the buzzer active\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_BuzzerCommand, 0};
static const CLI_Command_Definition_t xRelayCommand = {"relay", "relay: Sets the relay active\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_RelayCommand, 0};
static const CLI_Command_Definition_t xProximityCommand = {"proximity", "proximity: Sets the proximity active\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_ProximityCommand, 0};
static const CLI_Command_Definition_t xVoltageCommand = {"volt", "volt: Reads the voltage\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_VoltageCommand, 0};
	
	
SemaphoreHandle_t cliCharReadySemaphore;  ///< Semaphore to indicate that a character has been received
SemaphoreHandle_t readchar_mutex;
char ticks[5];

/**************************
 * Forward Declarations
 **************************/
static void FreeRTOS_read(char *character);

/**************************
 * Callback Functions
 **************************/

/**************************
 * CLI Thread
 **************************/

void vCommandConsoleTask(void *pvParameters)
{
    // REGISTER COMMANDS HERE
    FreeRTOS_CLIRegisterCommand(&xOTAUCommand);
    FreeRTOS_CLIRegisterCommand(&xClearScreen);
    FreeRTOS_CLIRegisterCommand(&xResetCommand);
    FreeRTOS_CLIRegisterCommand(&xI2cScan);
	FreeRTOS_CLIRegisterCommand(&xVerisonCommand);
    FreeRTOS_CLIRegisterCommand(&xTicksCommand);
	FreeRTOS_CLIRegisterCommand(&xTemp);
	FreeRTOS_CLIRegisterCommand(&xImu);
	FreeRTOS_CLIRegisterCommand(&xBuzzerCommand);
	FreeRTOS_CLIRegisterCommand(&xRelayCommand);
	FreeRTOS_CLIRegisterCommand(&xProximityCommand);
	FreeRTOS_CLIRegisterCommand(&xVoltageCommand);
	
    char cRxedChar[2];
    unsigned char cInputIndex = 0;
    BaseType_t xMoreDataToFollow;
    /* The input and output buffers are declared static to keep them off the stack. */
    static char pcOutputString[MAX_OUTPUT_LENGTH_CLI], pcInputString[MAX_INPUT_LENGTH_CLI];
    static char pcLastCommand[MAX_INPUT_LENGTH_CLI];
    static bool isEscapeCode = false;
    static char pcEscapeCodes[4];
    static uint8_t pcEscapeCodePos = 0;

    /* This code assumes the peripheral being used as the console has already
    been opened and configured, and is passed into the task as the task
    parameter.  Cast the task parameter to the correct type. */

    /* Send a welcome message to the user knows they are connected. */
    SerialConsoleWriteString((char *)pcWelcomeMessage);

    // Any semaphores/mutexes/etc you needed to be initialized, you can do them here
    cliCharReadySemaphore = xSemaphoreCreateBinary();
    if (cliCharReadySemaphore == NULL) {
        LogMessage(LOG_ERROR_LVL, "Could not allocate semaphore\r\n");
        vTaskSuspend(NULL);
    }

    for (;;) {
        FreeRTOS_read(&cRxedChar[0]);

        if (cRxedChar[0] == '\n' || cRxedChar[0] == '\r') {
            /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
            SerialConsoleWriteString((char *)"\r\n");
            // Copy for last command
            isEscapeCode = false;
            pcEscapeCodePos = 0;
            strncpy(pcLastCommand, pcInputString, MAX_INPUT_LENGTH_CLI - 1);
            pcLastCommand[MAX_INPUT_LENGTH_CLI - 1] = 0;  // Ensure null termination

            /* The command interpreter is called repeatedly until it returns
            pdFALSE.  See the "Implementing a command" documentation for an
            explanation of why this is. */
            do {
                /* Send the command string to the command interpreter.  Any
                output generated by the command interpreter will be placed in the
                pcOutputString buffer. */
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand(pcInputString,        /* The command string.*/
                                                               pcOutputString,       /* The output buffer. */
                                                               MAX_OUTPUT_LENGTH_CLI /* The size of the output buffer. */
                );

                /* Write the output generated by the command interpreter to the
                console. */
                // Ensure it is null terminated
                pcOutputString[MAX_OUTPUT_LENGTH_CLI - 1] = 0;
                SerialConsoleWriteString(pcOutputString);

            } while (xMoreDataToFollow != pdFALSE);

            /* All the strings generated by the input command have been sent.
            Processing of the command is complete.  Clear the input string ready
            to receive the next command. */
            cInputIndex = 0;
            memset(pcInputString, 0x00, MAX_INPUT_LENGTH_CLI);
            memset(pcOutputString, 0, MAX_OUTPUT_LENGTH_CLI);
        } else {
            /* The if() clause performs the processing after a newline character
is received.  This else clause performs the processing if any other
character is received. */

            if (true == isEscapeCode) {
                if (pcEscapeCodePos < CLI_PC_ESCAPE_CODE_SIZE) {
                    pcEscapeCodes[pcEscapeCodePos++] = cRxedChar[0];
                } else {
                    isEscapeCode = false;
                    pcEscapeCodePos = 0;
                }

                if (pcEscapeCodePos >= CLI_PC_MIN_ESCAPE_CODE_SIZE) {
                    // UP ARROW SHOW LAST COMMAND
                    if (strcasecmp(pcEscapeCodes, "oa")) {
                        /// Delete current line and add prompt (">")
                        sprintf(pcInputString, "%c[2K\r>", 27);
                        SerialConsoleWriteString((char *)pcInputString);
                        /// Clear input buffer
                        cInputIndex = 0;
                        memset(pcInputString, 0x00, MAX_INPUT_LENGTH_CLI);
                        /// Send last command
                        strncpy(pcInputString, pcLastCommand, MAX_INPUT_LENGTH_CLI - 1);
                        cInputIndex = (strlen(pcInputString) < MAX_INPUT_LENGTH_CLI - 1) ? strlen(pcLastCommand) : MAX_INPUT_LENGTH_CLI - 1;
                        SerialConsoleWriteString(pcInputString);
                    }

                    isEscapeCode = false;
                    pcEscapeCodePos = 0;
                }
            }
            /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */

            else if (cRxedChar[0] == '\r') {
                /* Ignore carriage returns. */
            } else if (cRxedChar[0] == ASCII_BACKSPACE || cRxedChar[0] == ASCII_DELETE) {
                char erase[4] = {0x08, 0x20, 0x08, 0x00};
                SerialConsoleWriteString(erase);
                /* Backspace was pressed.  Erase the last character in the input
                buffer - if there are any. */
                if (cInputIndex > 0) {
                    cInputIndex--;
                    pcInputString[cInputIndex] = 0;
                }
            }
            // ESC
            else if (cRxedChar[0] == ASCII_ESC) {
                isEscapeCode = true;  // Next characters will be code arguments
                pcEscapeCodePos = 0;
            } else {
                /* A character was entered.  It was not a new line, backspace
                or carriage return, so it is accepted as part of the input and
                placed into the input buffer.  When a n is entered the complete
                string will be passed to the command interpreter. */
                if (cInputIndex < MAX_INPUT_LENGTH_CLI) {
                    pcInputString[cInputIndex] = cRxedChar[0];
                    cInputIndex++;
                }

                // Order Echo
                cRxedChar[1] = 0;
                SerialConsoleWriteString(&cRxedChar[0]);
            }
        }
    }
}

/**
* @fn			void FreeRTOS_read(char* character)
* @brief		STUDENTS TO COMPLETE. This function block the thread unless we received a character. How can we do this?
                                There are multiple solutions! Check all the inter-thread communications available! See https://www.freertos.org/a00113.html
* @details		STUDENTS TO COMPLETE.
* @note
***************************/
static void FreeRTOS_read(char *character)
{
	while(1)
	{
		if(xSemaphoreTake(cliCharReadySemaphore ,(TickType_t)10) == pdTRUE)
		{
			*character = latestRx;
			break;
		}
	}
	
}

/**
 * @fn			void CliCharReadySemaphoreGiveFromISR(void)
 * @brief		Give cliCharReadySemaphore binary semaphore from an ISR
 * @details
 * @note
 */
void CliCharReadySemaphoreGiveFromISR(void)
{
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(cliCharReadySemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**************************
 * CLI Functions - Define here
 **************************/

// THIS COMMAND USES vt100 TERMINAL COMMANDS TO CLEAR THE SCREEN ON A TERMINAL PROGRAM LIKE TERA TERM
// SEE http://www.csie.ntu.edu.tw/~r92094/c++/VT100.html for more info
// CLI SPECIFIC COMMANDS
static char bufCli[CLI_MSG_LEN];
BaseType_t xCliClearTerminalScreen(char *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    char clearScreen = ASCII_ESC;
    snprintf(bufCli, CLI_MSG_LEN - 1, "%c[2J", clearScreen);
    snprintf(pcWriteBuffer, xWriteBufferLen, bufCli);
    return pdFALSE;
}

// Example CLI Command. Reads from the IMU and returns data.
BaseType_t CLI_OTAU(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	char bootloader_flag[] = "0:boot_flag.txt";
	f_unlink(bootloader_flag);
	
	FIL file_object;
	
	bootloader_flag[0] = LUN_ID_SD_MMC_0_MEM + '0';
	FRESULT res = f_open(&file_object, (char const *)bootloader_flag, FA_CREATE_ALWAYS | FA_WRITE);

	if (res != FR_OK) {
		LogMessage(LOG_INFO_LVL, "[FAIL] res %d\r\n", res);
		} else {
		SerialConsoleWriteString("boot_flag.txt added!\r\n");
	}
    WifiHandlerSetState(WIFI_DOWNLOAD_INIT);

    return pdFALSE;
}

// Example CLI Command. Resets system.
BaseType_t CLI_ResetDevice(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    system_reset();
    return pdFALSE;
}

/**
 * @brief    Scans fot connected i2c devices
 * @param    p_cli
 * @param    argc
 * @param    argv
 **************************/
BaseType_t CLI_i2cScan(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    I2C_Data i2cDevice;
    uint8_t address;
    // Send 0 command byte
    uint8_t dataOut[2] = {0, 0};
    uint8_t dataIn[2];
    dataOut[0] = 0;
    dataOut[1] = 0;
    i2cDevice.address = 0;
    i2cDevice.msgIn = (uint8_t *)&dataIn[0];
    i2cDevice.lenOut = 1;
    i2cDevice.msgOut = (const uint8_t *)&dataOut[0];
    i2cDevice.lenIn = 1;

    SerialConsoleWriteString("0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        snprintf(bufCli, CLI_MSG_LEN - 1, "%02x: ", i);
        SerialConsoleWriteString(bufCli);

        for (int j = 0; j < 16; j++) {
            i2cDevice.address = (i + j) << 1;

            int32_t ret = I2cWriteDataWait(&i2cDevice, 100);
            if (ret == 0) {
                snprintf(bufCli, CLI_MSG_LEN - 1, "%02x: ", i2cDevice.address);
                SerialConsoleWriteString(bufCli);
            } else {
                snprintf(bufCli, CLI_MSG_LEN - 1, "X ");
                SerialConsoleWriteString(bufCli);
            }
        }
        SerialConsoleWriteString("\r\n");
    }
    SerialConsoleWriteString("\r\n");
    return pdFALSE;
}

//CLI Command. Prints the firmware version.
BaseType_t CLI_Version( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	SerialConsoleWriteString(version);
	return pdFALSE;
}

//CLI Command. Prints the ticks elapsed since start of the scheduler.
BaseType_t CLI_Ticks( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	char buffer[30];
	sprintf(buffer,"%d\n\r", xTaskGetTickCount());
	SerialConsoleWriteString(buffer);
	return pdFALSE;
}

BaseType_t CLI_temp(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	uint16_t temp[2];
	SHTC3_Measure(temp);
	char celsius[50];
	sprintf(celsius, "\r\nTemp = %d\r\n", temp[1]);
	SerialConsoleWriteString(celsius);
	return pdFALSE;

}

BaseType_t CLI_imu( int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString )
{
	static int16_t  data_raw_acceleration[3];
	static int16_t  data_raw_angular_rate;
	static float acceleration_mg[3];
	uint8_t reg;
	stmdev_ctx_t *dev_ctx = GetImuStruct();


	/* Read output only if new xl value is available */
	lis2dh12_xl_data_ready_get(dev_ctx, &reg);

	if(reg){
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lis2dh12_acceleration_raw_get(dev_ctx, data_raw_acceleration);
		acceleration_mg[0] =
		lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] =
		lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] =
		lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration[2]);

		snprintf(pcWriteBuffer,xWriteBufferLen, "Acceleration [mg]:X %d\tY %d\tZ %d\r\n",
		(int)acceleration_mg[0], (int)acceleration_mg[1], (int)acceleration_mg[2]);
	}else
	{
		snprintf(pcWriteBuffer,xWriteBufferLen, "No data ready! \r\n");
	}
	return pdFALSE;
}


BaseType_t CLI_BuzzerCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	SerialConsoleWriteString("Buzzer activated \r\n");
	buzzer_open();
	//buzzer_blip();
}

BaseType_t CLI_RelayCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	SerialConsoleWriteString("Relay activated \r\n");
	relay_open();
	//buzzer_blip();
}


BaseType_t CLI_ProximityCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	SerialConsoleWriteString("Proximity activated \r\n");
	SerialConsoleWriteString("  \r\n");	
	vl6180x_init();
	SerialConsoleWriteString("Initialization Successful \r\n");
	
	
	uint8_t range_value = vl6180x_read_range_single();
	SerialConsoleWriteString(range_value);
	SerialConsoleWriteString("  \r\n");
	
	
	uint16_t ambient_light_value = vl6180x_read_ambient_light_single();
	//SerialConsoleWriteString(ambient_light_value);
	//SerialConsoleWriteString(" \r\n");
	
	//buzzer_blip();
}


BaseType_t CLI_VoltageCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	SerialConsoleWriteString("Reading Voltage.. \r\n");
	delay_ms(1000);
	
	configure_gpio();
	delay_ms(1000);
}
