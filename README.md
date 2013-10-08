#Introduction

The LiteBlu USB Host is a Microchip PIC microcontroller board which contains a built-in full-speed USB host controller. This allows the board to connect to and control any USB device.

#Features
	No USB Programming required
	UART and I2C interface
	Software upgradable via UART using built in bootloader (DS30Loader)
	Available firmware
	USB Flash Drive
	USB Keyboard (coming soon)
	USB Mouse (coming soon)
	USB Joystick (coming soon)
	Built-in Real Time Clock
	5V and 3.3V compatible
	On-board 3.3V regulator with output (up to 250mA)

#Host software
##Flash Drive

The Flash Drive Host Software allows you to read/write from/to a USB Flash Drive. You can connect to it using terminal software (RS232 to serial TTL convertion required) or directly to your 5V or 3.3V microcontroller.

###Features
	Supports FAT16 and FAT32 formats
	Simple serial (UART) interface
	Silent mode or terminal mode

###Required Connections
	5V Power [INPUT]
	GND
	TX [OUTPUT]
	RX [INPUT]
	SS [OUTPUT] Optional. Goes high when USB Flash Drive connected, low if removed.

###Supported Commands




#Programming
This section describes how to reprogram the LiteBlu USB Host using the built in bootloader.

The built in bootloader is the free DS30Loader.
DS30Loader Setup
Browse to the downloaded hex file
Select the correct device. PIC24FJ 64GB0002
Select the Baud rate as 115200
Select the correct serial port that the USB Host is connected to
Tick "Write Flash"
Select "Xon/Xoff" Flow Control

Then put the USB Host into bootloader mode by resetting it. This can be done by toggling the power, or by shorting the reset pin to ground. The reset pin is the square pad on the 5-way (ICSP) connector on the side of the USB Host board and GND is the center pin on the 5-way (ICSP) connector. The green LED will light up for the entire duration (3 seconds) the USB Host board is in the bootloader.

After resetting the USB Host board, immediately(within 3 seconds) click "Write" in the top left corner of the DS30Loader GUI.
