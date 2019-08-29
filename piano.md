the raspberry pi is wired up like this

## POWER
Do not try and power the chain of sensors with the Pi IO pins! It will not 
end well.

- RED 5V
- BLACK GND

## SENSORS
I2C bus. The connection only needs to run as far as the first sensor on the 
0x49 chain

- GREEN SDA1
- BLUE SCL1

# SERIAL CONNECTION
Use a USB to UART (NOT RS232!!) adapter for this, such as one of the many 
FT232 or CP2102 boards out there.

Signal levels are 3.3V. Baud rate can go up to 115200. 

Once the pi is booted you can access a standard linux tty over the serial 
connection. This is intended for servicing in the field, if you don't want to 
lug a HDMI screen around.

Username: pi, password: Flipside2018

- GREY GND
- YELLOW TXD -> RXD on bridge
- PURPLE RXD -> TXD on bridge