# SwitchLink

SwitchLink is a Smart Switch Control Unit that goes beyond conventional remote switch control. It features a Smart USB charging system utilizing an OP Amp comparator to detect the full-charge status of connected devices. This feature is especially useful for devices and appliances lacking built-in Charge Protection Circuitry. When a device reaches full charge, SwitchLink notifies the user with defined acoustic notifications generated by a Piezo-buzzer via PWM.

## Key components and functionalities include:

* Smart USB Charging System: Monitors and indicates full-charge status using an OP Amp comparator.
* Acoustic Notifications: Utilizes a Piezo-buzzer controlled through PWM to notify users when charging is complete.
* Relay Module: Enables control of electrical switches.
* Proximity Sensors: Provides a non-contact user interface for switch control.
* Remote Control: Allows switching control over the Internet using MQTT.
* SwitchLink enhances the usability and safety of home appliances and devices, providing both manual and remote control options.


## Critical Components:

Microcontroller (MCU):
SAMW25 Xplained Pro: This microcontroller handles all processing tasks, including sensor data acquisition, control logic, and communication. It is integrated with FreeRTOS for real-time task management and efficient operation.

![image-removebg-preview](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/be7518bf-673f-40b1-af91-a3addea1cad7)

Sensors:

* VL6180X Proximity Sensors: Two proximity sensors are used for non-contact switch control. These sensors detect the presence of a hand or object near the switch, allowing for gesture-based control of the connected devices.

![image-removebg-preview (1)](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/3cc1c76a-1670-48ac-99c9-2f5dde4c7727)


* OP Amp Comparator: Used in the Smart USB charging system to detect the full-charge status of connected devices. This comparator monitors the voltage level and triggers notifications when the device is fully charged.

![image-removebg-preview (2)](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/92fff826-ceda-4f69-b764-4a4bd96e856c)


Actuators:

* Relay Module: Controls the switching of electrical devices. The relay module is activated based on inputs from the proximity sensors or remote commands received over the Internet.

* Piezo-buzzer: Generates acoustic notifications when a device connected to the USB charging system is fully charged. The buzzer is controlled using PWM signals to create different sound patterns.

Communication Module:

 * MQTT Protocol: Enables remote control and monitoring of the switches over the Internet. The device connects to an MQTT broker, allowing users to send commands and receive status updates from anywhere with Internet access.


Power Management:

* LDO: MCP1700T converts the input of 3.3V to 4.2V to 2.8V and supplies to the VL6180X sensor whose ideal working voltage is 2.8V.

![image-removebg-preview (3)](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/042b3e7d-523f-44be-9395-02d7f2b7e3a4)


* Buck Converter: TPS62082DSGR supplies to most of the compoents a stable 3.3V from the 3.3V to 4.2V input.

![image-removebg-preview (4)](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/093e6745-98b5-4a6d-9cec-6dc62b5534f6)

  
## Block Diagram:

![image](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/c8816c0a-f5d6-4f1b-b5fa-2764fc3267ef)


## Functional Workflow:

* Power Up: The device initializes, and the PMIC ensures stable power distribution to all components.
* Sensor Monitoring: The proximity sensors continuously monitor for user gestures to control the switches.
* Charge Detection: The OP Amp comparator monitors the voltage level of connected devices.
* User Interaction: When a proximity sensor detects a hand gesture, it sends a signal to the MCU, which activates the corresponding relay to turn the device on or off.
* Remote Commands: Users can send commands via the Internet using the MQTT protocol, allowing remote control of the switches.
* Notifications: When a device reaches full charge, the OP Amp comparator triggers the Piezo-buzzer to notify the user acoustically.


## PCB Designing:

Altium Designer was leveraged for Footprint Creation, Schematic Capture, layout, Workmanship, Gerber File generation etc.

Altium Project Link: https://upenn-eselabs.365.altium.com/designs/0D535367-1C3A-4320-998F-776AB24CDA05


## Schematics:

* Main Sheet:

![Switchlink Schematics-01](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/bcf5ba8b-e43f-4cb7-9aed-f86b0244c826)

* 
![Switchlink Schematics-02](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/7399f2b9-9b40-4a14-bfcb-3d1940047235)


![Switchlink Schematics-03](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/1a79a15c-ed98-44dc-aca4-44ec9ed4b5fc)


![Switchlink Schematics-04](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/d8d221ff-b83c-4f61-98e9-1a9999b1437a)


![Switchlink Schematics-05](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/9940557d-a484-41e3-8d47-081e26c06199)


![Switchlink Schematics-06](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/8d996f85-a7ac-4376-bde6-4b34112619f7)


![Switchlink Schematics-07](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/be58b38a-cdf2-429b-8225-414e3e691ce0)


![Switchlink Schematics-08](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/4c64e0d1-cc46-41be-9a11-3afdbac27f4b)


![Switchlink Schematics-09](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/6ed32399-0cc6-4e11-8f7d-774d0ad69ed0)


![Switchlink Schematics-10](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/a2cd26d7-ed41-4fc6-8eef-9bd963059d20)


![Switchlink Schematics-11](https://github.com/Praveen-Raj-u-s/SwitchLink/assets/114270637/4134a43d-57e8-428c-b7f9-c8e66e54cece)





