#include <Wire.h> // Arduino I2C library
#include <stdint.h> // Standard C, Allows explicit data type declaration.
#include <SPI.h>

// not tested
setup() {}

loop() {
int M3200address = 0x28;   // 0x28, 0x36 or 0x46, depending on the sensor.
float maxPressure = 100;   // pressure in PSI for this sensor, 100, 250, 500, ... 10k.

int n = Wire.requestFrom( M3200address, 4);   // request 4 bytes
if( n == 4 ) {
 uint16_t rawP;     // pressure data from sensor
 uint16_t rawT;     // temperature data from sensor
 rawP = (uint16_t) Wire.read();    // upper 8 bits
 rawP <<= 8;
 rawP |= (uint16_t) Wire.read();    // lower 8 bits
 rawT = (uint16_t)  Wire.read();    // upper 8 bits
 rawT <<= 8;
 rawT |= (uint16_t) Wire.read();   // lower 8 bits

 byte status = rawP >> 14;   // The status is 0, 1, 2 or 3
 rawP &= 0x3FFF;   // keep 14 bits, remove status bits

 rawT >>= 5;     // the lowest 5 bits are not used

 // The math could be done with integers, but I choose float for now
 float pressure = ((rawP - 1000.0) / (15000.0 - 1000.0)) * maxPressure;
 float temperature = ((rawT - 512.0) / (1075.0 - 512.0)) * 55.0;

 Serial.print( "Status = ");
 Serial.print( status);
 Serial.print( ", Pressure = ");
 Serial.print( pressure);
 Serial.print( " psi, Temperature = ");
 Serial.print( temperature);
 Serial.print( " *C");
 Serial.println();
} else {
 Serial.println( "Sensor not found");
}

}
