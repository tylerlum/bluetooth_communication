# bluetooth_communication

The purpose of this repository is to store Arduino code and instructions for Bluetooth communication between the HC-05 and HC-06 Bluetooth modules.

## What does this code do?

This Arduino software demonstrates for basic communication between a Slave and Master. The Slave reads a potentiometer's value, sends it to the Master through Bluetooth, and then the Master lights up the LED with a brightness proportional to that value.

## Instructions

1. For each Bluetooth module, bring them into AT mode (its LED will be blinking at ~2Hz) and then send commands to setup the modules accordingly. For the Slave (HC-06), the device is in AT mode by default unless connected to another device. 

To be continued.

## Important details

* From an Arduino forum: "Although most modules have on-board current limiting circuit which allows the TX/RX pins to work with 5V signal, a 5V to 3.3V circuit (a simple voltage divider) at the Arduino TX pin is recommended". "On a 5v "Arduino" 3.3v will trigger the Rx port (even tho it's a 5v port) , if you transmit to a 3.3v device use a voltage divider to convert 5v down to 3.3v." In summary, this means that the Bluetooth Tx pin can be directly connected to the Arduino's Rx pin, but the Arduino's Tx pin's output must be put through a voltage divider before connecting to the Bluetooth's Rx pin.

* The exact details about the Bluetooth module initial setup may vary between suppliers and even devices. When setting up the device, you may need to try different baud rates (both in the Serial setup code and the Serial monitor bottom right) and include/exclude line endings or returns (NL and CR) (changed in bottom right of Serial monitor). As well, AT commands may be in the form "AT+NAME=NEW_NAME" or "AT+NAMENEW_NAME". Many resources said that the HC-06 would require no line endings or returns and would use the AT command form "AT+NAMENEW_NAME", but the actual device required NL and CR and AT command form "AT+NAME=NEW_NAME". Many resources said that the HC-05 would default at an initial 9600 baud rate, but actually would be at 38400. For your specific device, be sure to experiment with these different parameters. It is recommended to change all UART baud rates to 9600 for simplicity.

## Future work

* A primary goal of the project is to minimize the use of Arduinos. In particular, it would be best to find a way to use the Slave without an Arduino. Research must be done to see if the Slave Arduino can be replaced with a smaller, simpler device for serial communication to the HC-06. The TOF sensor that will be used by the Slave also communicates with I2C, but it needs to be tested if they can be connected directly or not.

## References

* Primary reference for HC-05 and HC-06 setup and communication: https://www.youtube.com/watch?v=Y2uwyUNt9ZM&t=5s

* Demonstration for proper AT commands: https://www.youtube.com/watch?v=2fISwoemKIw

* Longer demonstration of AT commands: https://www.youtube.com/watch?v=VveS1MhYYMk

* Example code for AT mode setup: https://forum.arduino.cc/index.php?topic=484135.0

* General HC-06 information: https://www.youtube.com/watch?v=Iyd4gX0AR54

* AT commands in detail: http://www.mediafire.com/file/xr05sdzxp0p48bb/BLUTOOTH_AT_Command.pdf/file

* Basic demonstration of HC-05: https://www.youtube.com/watch?v=x3KAXjnP06o
