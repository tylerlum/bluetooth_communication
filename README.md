# bluetooth_communication

The purpose of this repository is to store Arduino code and instructions for Bluetooth communication between the HC-05 and HC-06 Bluetooth modules.

NEW INFORMATION TO WORK WITH TOF SENSOR. You must copy the `vl53l0x-arduino-master` code into `C:\Program Files (x86)\Arduino\libraries` or wherever the `Arduino\libraries` folder is. This is necessary to compile.

## What does this code do?

This Arduino software demonstrates for basic communication between a Slave and Master. The Slave reads a potentiometer's value, sends it to the Master through Bluetooth, and then the Master lights up the LED with a brightness proportional to that value.

![Alt Text](demo.gif)

_Figure 1: Turning the potentiometer of the Slave wirelessly changes the LED brightness of the Master using Bluetooth._

## Requirements

* 2 x Arduino Unos or similar

* HC-05 and HC-06 Bluetooth modules

* Solid core wire

* 2 x LEDs

* 8 x 1 kOhm Resistors

* 1 x Potentiometer

* 1 x Breadboard

## Instructions

Please read through all the instructions and important details before beginning, as there are many details to get right.

* For each Bluetooth module, upload AT_Mode_Setup to the connected Arduino and then bring them into AT mode using (its LED will be blinking at ~2Hz). For this code to work properly, the Bluetooth module Tx pin should connect to pin 2 and the Rx pin should connect to pin 3. Then send AT commands to setup the modules accordingly. AT commands must be all caps.

    * Slave
    
        * For the Slave (HC-06), the device is in AT mode by default unless connected to another device.
        
        * Open the Serial Monitor and send "AT". If an OK response is received, you can move forward. Otherwise, test out changing if line endings or returns are included and try again. Also try changing the baud rate in the code and the Serial monitor. 
        
        * Print the name, version, and PIN by typing "AT+NAME", "AT+VERSION", and "AT+PIN" or "AT+PSWD"
        
        * Send the following commands: Set slave role "AT+ROLE=0", Set baud rate "AT+UART=9600,0,0", Set slave mode connection type "AT+CMODE=2"
        
        * Check that the values were properly changed by sending "AT+ROLE", "AT+UART", "AT+CMODE". Then save the output of the following "AT+ADDR".
    
    * Master
    
        * For the Master (HC-05), you can hold the push button on the module while it is powered, de-power the module, and then re-power it, which should bring it to AT mode. 
        
        * Open the Serial Monitor and send "AT". If an OK response is received, you can move forward. Otherwise, test out changing if line endings or returns are included and try again. Also try changing the baud rate in the code and the Serial monitor. 
        
        * Print the name, version, and PIN by typing "AT+NAME", "AT+VERSION", and "AT+PIN" or "AT+PSWD"
        
        * Send the following commands: Set master role "AT+ROLE=1", Set baud rate "AT+UART=9600,0,0", Set connection mode to connect to only one specific device (not any) "AT+CMODE=1", Set the Slave address to look for "AT+BIND=\<Slave address\>" (must change colons to commas), Try to connect to this address, "AT+LINK=\<Slave address\>" (must change colons to commas), Try to connect to this address for limited time 20 "AT+PAIR=\<Slave address\>,20" (must change colons to commas), Check that state is initialized "AT+STATE"
        
        * Check that the values were properly changed by sending "AT+ROLE", "AT+UART", "AT+CMODE", "AT+BIND".
    
* De-power and re-power to Bluetooth modules. They should automatically connect, which is shown when the blinking frequency drops.

* Upload the Master and Slave Arduino code

* Connect the corresponding pins. Only the middle 4 pins of the HC-05 need to be connected. 5V for Vcc, GND for GND, BT Tx to Arduino Rx, BT RX to Arduino Tx with V divider.

* Run the Arduinos, turn the potentiometer, and see the LEDs change brightness.

## Important details

* From an Arduino forum: "Although most modules have on-board current limiting circuit which allows the TX/RX pins to work with 5V signal, a 5V to 3.3V circuit (a simple voltage divider) at the Arduino TX pin is recommended". "On a 5v "Arduino" 3.3v will trigger the Rx port (even tho it's a 5v port) , if you transmit to a 3.3v device use a voltage divider to convert 5v down to 3.3v." In summary, this means that the Bluetooth Tx pin can be directly connected to the Arduino's Rx pin, but the Arduino's Tx pin's output must be put through a voltage divider before connecting to the Bluetooth's Rx pin.

* The exact details about the Bluetooth module initial setup may vary between suppliers and even devices. When setting up the device, you may need to try different baud rates (both in the Serial setup code and the Serial monitor bottom right) and include/exclude line endings or returns (NL and CR) (changed in bottom right of Serial monitor). As well, AT commands may be in the form "AT+NAME=NEW_NAME" or "AT+NAMENEW_NAME". Many resources said that the HC-06 would require no line endings or returns and would use the AT command form "AT+NAMENEW_NAME", but the actual device required NL and CR and AT command form "AT+NAME=NEW_NAME". Many resources said that the HC-05 would default at an initial 9600 baud rate, but actually would be at 38400. For your specific device, be sure to experiment with these different parameters. It is recommended to change all UART baud rates to 9600 for simplicity.

* The Tx and Rx pins must be disconnected during upload, otherwise it will cause the upload to fail.

* In connected mode, the Bluetooth module LED will blink at ~0.5Hz. In AT mode, the Bluetooth module LED will blink at ~2Hz. 

* The default PIN on most devices is 0000 or 1234. The default baud rate is usually 9600 or 38400.

## Future work

* A primary goal of the project is to minimize the use of Arduinos. In particular, it would be best to find a way to use the Slave without an Arduino. Research must be done to see if the Slave Arduino can be replaced with a smaller, simpler device for serial communication to the HC-06. The TOF sensor that will be used by the Slave also communicates with I2C, but it needs to be tested if they can be connected directly or not.

## References

* Primary reference for HC-05 and HC-06 setup and communication: https://www.youtube.com/watch?v=Y2uwyUNt9ZM&t=5s

* Demonstration for proper AT commands: https://www.youtube.com/watch?v=2fISwoemKIw

* Longer demonstration of AT commands: https://www.youtube.com/watch?v=VveS1MhYYMk

* Example code for AT mode setup: https://forum.arduino.cc/index.php?topic=484135.0

* General HC-06 information: https://www.youtube.com/watch?v=Iyd4gX0AR54

* AT commands in detail: http://www.mediafire.com/file/xr05sdzxp0p48bb/BLUTOOTH_AT_Command.pdf/file

* Basic demonstration of HC-05: https://www.youtube.com/watch?v=x3KAXjnP06o.
