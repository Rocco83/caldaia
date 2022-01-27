=== reference ===
original link: 
// Written: 7/9/2019
// Rev.: 1.00
// Changes: First release back to the community. Started with bits of code downloaded from the Arduino forums.
//
// The sensor specific characteristics are setup with constants in the beginning of the program.
// The device page is https://www.te.com/usa-en/product-CAT-BLPS0002.html 4
// The device data sheet can be found at https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+SheeM32B1pdEnglisENG_DS_M32O_B10.pdCAT-BLPS0002 10
//

TE datasheet
farnell and mouser link
https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FM3200%7FA11%7Fpdf%7FEnglish%7FENG_DS_M3200_A11.pdf%7F20010080-00



=== insttuctions ===
arduino-cli lib install ArduinoMqttClient
arduino-cli lib install MQTT -- decidere quale usare
arduino-cli lib install Ethernet
arduino-cli lib install "Adafruit SSD1306"
arduino-cli lib install ArduinoJson

arduino-cli compile caldaia.ino -b arduino:avr:ethernet && arduino-cli upload caldaia --port=/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_USB-Serial_64935343733351E002D0-if00 -b arduino:avr:ethernet && screen /dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_USB-Serial_64935343733351E002D0-if00 115200
