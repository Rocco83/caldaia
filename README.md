# Boiler controller
This project has born to control temperature and pressure of my home boiler.
It make use of a TE M32JM-000105-100PG pressure trasducer (which include a temperature trasducer).
This version is an i2c device. I've selected the i2c version to avoid reading over the digital input.

## Detail over the device
The very same code used in the datasheet can be found over several other similar devices.
Due to that, basic sketches has been found in Arduino playground. I've selected one, and built on top of it.

In general, regardingless of the warning of the datasheet, performing a Wire.read(4) (give me 4 bytes) is working properly.

## Wiring
### M32JM specific
M32JM-000105-100PG require a pullup (from the technical specifications, 4.7KOhm should be OK) from SCL to VCC and from SDA to VCC.

### Ethernet long cable
i2c is not meant for long distance.
M32JM blocked Arduino more than once.

2 adafruit devices has been added into the chain:
https://www.adafruit.com/product/5159 Adafruit TCA4307 Hot-Swap I2C Buffer with Stuck Bus Recovery
https://www.adafruit.com/product/4756 Adafruit LTC4311 I2C Extender / Active Terminator

https://electronics.stackexchange.com/questions/107663/sending-i2c-reliabily-over-cat5-cables
https://pinoutguide.com/Net/poe_pinout.shtml
Pin 1 (Pair 1, White/Orange): SCL+
Pin 2 (Pair 1, Orange): SCL-
Pin 3 (Pair 2, White/Green): SDA+
Pin 4 (Pair 3, Blue): +12V
Pin 5 (Pair 3, White/Blue): +12V
Pin 6 (Pair 2, Green): SDA-
Pin 7 (Pair 4, White/Brown): GND
Pin 8 (Pair 4, Brown): GND

### Caveats and workaround
When the device was reporting a "stale" read (check the datasheet) it's always stale from that moment on.
It's not a matter of reading frequency.
So based on another portion of code, when stale is detected, another i2c read with 0 bytes is performed.

The casting in the conversion is needed as otherwise (beginner hint) the variables are managed as `int8_t` (or Arduino int), so max int 128 (AFAIR).

### reference
[original code](https://forum.arduino.cc/t/ms-4525do/298848/19)
[TE datasheet](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FM3200%7FA11%7Fpdf%7FEnglish%7FENG_DS_M3200_A11.pdf%7F20010080-00)

## instuctions
The following libraries are needed (using arduino-cli, feel free to pick the library names)
* `arduino-cli lib install MQTT`
* `arduino-cli lib install Ethernet`
* `arduino-cli lib install ArduinoJson`

You need to adjust the `#define MQTTBROKER` with your FQDN to be used.

### Compile instructions
This is mainly how to compile it for the specific arduino used in this project. Feel free to adapt it.
`arduino-cli compile boiler.ino -b arduino:avr:ethernet && arduino-cli upload boiler --port=/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_USB-Serial_64935343733351E002D0-if00 -b arduino:avr:ethernet && screen -L -Logfile ~/logs/arduino-$(date +%Y%m%d-%H%M%S).log /dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_USB-Serial_64935343733351E002D0-if00 115200`
