/**
 * @file      main.c
 * @brief     Main application entry point
 * @author    Eduardo Garcia
 * @author    Nick M-G
 * @date      2022-04-14
 ******************************************************************************/

/****
 * Includes
 ******************************************************************************/
#include <errno.h>

#include "CliThread/CliThread.h"
#include "FreeRTOS.h"
#include "I2cDriver\I2cDriver.h"
#include "SerialConsole.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "asf.h"
#include "driver/include/m2m_wifi.h"
#include "main.h"
#include "stdio_serial.h"
#include "imu/lis2dh12_reg.h"
#include "ButtonTask/ButtonTask.h"
#include "proximity/proximity.h"
#include "voltagesense/vsense.h"
#include "buzzer/buzzer.h"
#include "relay/relay.h"

extern BaseType_t CLI_imu(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
// Defines for task priorities and stack sizes
#define BUTTON_TASK_PRIORITY (5)
#define BUTTON_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)

#define PROXIMITY_TASK_PRIORITY (7)
#define PROXIMITY_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)


//#define CHARGER_TASK_PRIORITY (5)
//#define CHARGER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)

//Prototype for the button monitoring task
static void ButtonMonitorTask(void *pvParameters);
//static void ChargerMonitorTask(void *pvParameters);
static struct mqtt_module mqtt_inst;

/****
 * Defines and Types
 ******************************************************************************/
#define APP_TASK_ID 0 /**< @brief ID for the application task */
#define CLI_TASK_ID 1 /**< @brief ID for the command line interface task */
static const CLI_Command_Definition_t xImu ={"imu","imu: Returns a value from the IMU\r\n",CLI_imu,0};

/****
 * Local Function Declaration
 ******************************************************************************/
void vApplicationIdleHook(void);
//!< Initial task used to initialize HW before other tasks are initialized
static void StartTasks(void);
void vApplicationDaemonTaskStartupHook(void);

void vApplicationStackOverflowHook(void);
void vApplicationMallocFailedHook(void);
void vApplicationTickHook(void);

/****
 * Variables
 ******************************************************************************/
static TaskHandle_t cliTaskHandle = NULL;      //!< CLI task handle
static TaskHandle_t daemonTaskHandle = NULL;   //!< Daemon task handle
static TaskHandle_t wifiTaskHandle = NULL;     //!< Wifi task handle
static TaskHandle_t uiTaskHandle = NULL;       //!< UI task handle
static TaskHandle_t controlTaskHandle = NULL;  //!< Control task handle
static buttonControlStates buttonState;

char bufferPrint[64];  ///< Buffer for daemon task


/*

static void ProximitySensorTask(void *pvParameters) {
	I2C_Data sensor_data = { .address = 0x29, .sensor_id = 1 };  // Setup for sensor 1
	uint8_t distance;

	// Initialize and configure the proximity sensor
	if (vl6180x_init(&sensor_data, sensor_data.sensor_id) != 0) {
		SerialConsoleWriteString("Error initializing Proximity Sensor 1!\r\n");
		} else {
		vl6180x_configure_default(&sensor_data);
		SerialConsoleWriteString("Proximity Sensor 1 initialized and configured.\r\n");
	}

	// Task loop
	while (1) {
		if (vl6180x_measure_distance(&sensor_data, &distance) == 0) {
			char buffer[64];
			snprintf(buffer, sizeof(buffer), "Distance: %u mm\r\n", distance);
			SerialConsoleWriteString(buffer);
		}
		vTaskDelay(pdMS_TO_TICKS(100));  // Delay between measurements
	}
}

static void StartProximitySensorTask(void) {
	if (xTaskCreate(ProximitySensorTask, "Proximity Sensor", PROXIMITY_TASK_STACK_SIZE, NULL, PROXIMITY_TASK_PRIORITY, NULL) != pdPASS) {
		SerialConsoleWriteString("Error: Proximity Sensor task could not be initialized!\r\n");
	}
}
*/







/**
 * @brief Main application function.
 * Application entry point.
 * @return int
 */
int main(void)
{
    /* Initialize the board. */
    system_init();

    /* Initialize the UART console. */
    InitializeSerialConsole();

    // Initialize trace capabilities
    vTraceEnable(TRC_START);
    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;  // Will not get here
}

/**
 * function          vApplicationDaemonTaskStartupHook
 * @brief            Initialization code for all subsystems that require FreeRToS
 * @details			This function is called from the FreeRToS timer task. Any code
 *					here will be called before other tasks are initilized.
 * @param[in]        None
 * @return           None
 */

volatile uint8_t data[256];
void vApplicationDaemonTaskStartupHook(void)
{
    SerialConsoleWriteString("\r\n\r\n-----ESE516 Main Program-----\r\n");

    // Initialize HW that needs FreeRTOS Initialization
    SerialConsoleWriteString("\r\n\r\nInitialize HW...\r\n");
    if (I2cInitializeDriver() != STATUS_OK) {
        SerialConsoleWriteString("Error initializing I2C Driver!\r\n");
    } else {
        SerialConsoleWriteString("Initialized I2C Driver!\r\n");
    }
	
	// Initialize button
	SerialConsoleWriteString("Initializing Button...\r\n");
	InitButton();
	SerialConsoleWriteString("Button Initialized.\r\n");
	
	
	
	uint8_t whoamI = 0;
	(lis2dh12_device_id_get(GetImuStruct(), &whoamI));
	
	if (whoamI != LIS2DH12_ID){
		SerialConsoleWriteString("Cannot find IMU!\r\n");
	}
	else
	{
		SerialConsoleWriteString("IMU found!\r\n");
		if(InitImu() == 0)
		{
			SerialConsoleWriteString("IMU initialized!\r\n");
		}
		else
		{
			SerialConsoleWriteString("Could not initialize IMU\r\n");
		}
	}

	//StartProximitySensorTask();  // Initialize the proximity sensor task
	
    StartTasks();

    vTaskSuspend(daemonTaskHandle);
}

/**
 * function          StartTasks
 * @brief            Initialize application tasks
 * @details
 * @param[in]        None
 * @return           None
 */
static void StartTasks(void)
{
    snprintf(bufferPrint, 64, "Testing Heap before starting tasks: %d\r\n", xPortGetFreeHeapSize());
    SerialConsoleWriteString(bufferPrint);

    // Initialize Tasks here

    if (xTaskCreate(vCommandConsoleTask, "CLI_TASK", CLI_TASK_SIZE, NULL, CLI_PRIORITY, &cliTaskHandle) != pdPASS) {
        SerialConsoleWriteString("ERR: CLI task could not be initialized!\r\n");
    }

    snprintf(bufferPrint, 64, "Heap after starting CLI: %d\r\n", xPortGetFreeHeapSize());
    SerialConsoleWriteString(bufferPrint);

    if (xTaskCreate(vWifiTask, "WIFI_TASK", WIFI_TASK_SIZE, NULL, WIFI_PRIORITY, &wifiTaskHandle) != pdPASS) {
        SerialConsoleWriteString("ERR: WIFI task could not be initialized!\r\n");
    }
    snprintf(bufferPrint, 64, "Heap after starting WIFI: %d\r\n", xPortGetFreeHeapSize());
    SerialConsoleWriteString(bufferPrint);

	// Start Button Monitor task
	if (xTaskCreate(ButtonMonitorTask, "Button Monitor", BUTTON_TASK_STACK_SIZE, NULL, BUTTON_TASK_PRIORITY, NULL) != pdPASS) {
		SerialConsoleWriteString("ERR: Button Monitor task could not be initialized!\r\n");
	}
	
	// Start Button Monitor task
	/*if (xTaskCreate(ChargerMonitorTask, "Charger Monitor", CHARGER_TASK_STACK_SIZE, NULL, CHARGER_TASK_PRIORITY, NULL) != pdPASS) {
		SerialConsoleWriteString("ERR: Charger Monitor task could not be initialized!\r\n");
	}*/
}

void vApplicationMallocFailedHook(void)
{
    SerialConsoleWriteString("Error on memory allocation on FREERTOS!\r\n");
    while (1)
        ;
}

void vApplicationStackOverflowHook(void)
{
    SerialConsoleWriteString("Error on stack overflow on FREERTOS!\r\n");
    while (1)
        ;
}

#include "MCHP_ATWx.h"
void vApplicationTickHook(void)
{
    SysTick_Handler_MQTT();
}

static void ButtonMonitorTask(void *pvParameters) {
	bool button_held = false;
	const TickType_t xDelay = 50 / portTICK_PERIOD_MS; // Delay for debouncing
	char pcWriteBuffer[128];
	bool print = true;
	bool buttonpressed = true;
	for (;;) {
		if(button_held == false && print == true){
			SerialConsoleWriteString("Button Released\r\n");
			port_pin_set_output_level(LED_0_PIN,pdTRUE);
			print = false;
		}
		if (ButtonPressed()) {
			if (button_held == true)
			{
				memset(pcWriteBuffer, 0, sizeof(pcWriteBuffer)); // Clear the buffer
				CLI_imu(pcWriteBuffer, sizeof(pcWriteBuffer), NULL); // Call the IMU data fetch and format function
				SerialConsoleWriteString(pcWriteBuffer);
				port_pin_set_output_level(LED_0_PIN,pdFALSE);
				print = true;
			}
			else
			{
				button_held=true;
				SerialConsoleWriteString("Button Pressed - IMU Data\r\n");
				
				print = true;
			}
			
			vTaskDelay(xDelay); // Debounce delay
		}
		else{
			button_held = false;
		}
		vTaskDelay(100 / portTICK_PERIOD_MS); // Short delay to yield time to other tasks
	}
}


static void ChargerMonitorTask(void *pvParameters){
	for (;;)
	{
		SerialConsoleWriteString("Charger Task");
		configure_gpio();
	}
}
//--------------------------------------------------------------------------------------------------



