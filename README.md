=== insttuctions ===

arduino-cli compile caldaia.ino -b arduino:avr:ethernet && arduino-cli upload caldaia --port=/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_USB-Serial_64935343733351E002D0-if00 -b arduino:avr:ethernet && screen /dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_USB-Serial_64935343733351E002D0-if00 115200
