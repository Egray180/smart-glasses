BLE_updateOLED:
	Combines OLED firmware with BLE firmware for final implementation

ChangeUARTto9600:
	Changes the UART rate of the BLE module to 9600baud/s, which is what subsequent code uses and is more stable

BLE_WorkingDebug:
	Firmware used to get BLE working in the first place. Was used in conjunction with MECH421 Serial Communicator.exe for testing. 

Bluetooth:
	Slightly more developed BLE firmware than the debug version which automatically sets desired parameters on start up. 

OLED Setup:
	Extremely primitive version of OLED display setup firmware, not sure if it is a working version. See original-screen-code branch for a proper working version that lets you write text to the screen from a PC side Windows Forms App.
	
