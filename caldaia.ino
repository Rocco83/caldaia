// File M32 Sensor MQTT client
//
// A Arduino program to send the data (pressure and temperature) from the pressure sensors TE Connectivity M32 series (I2C versions only)
// Original file for MS4525 pressure sensor
// https://forum.arduino.cc/t/ms-4525do/298848/14
// other useful link:
// https://forum.arduino.cc/t/ms-4525do/298848/5
// https://forum.arduino.cc/t/pressure-sensor-i2c-communication/853443/2
//
#include <Wire.h> // Arduino I2C library
#include <stdint.h> // Standard C, Allows explicit data type declaration.

/*
 * M32 sensor characteristics (from the 02/2021 version of the data sheet)
 */

// M32 sensor I2C address
const uint8_t M32Address = 0x28;

// M32 sensor full scale range and units
const int16_t M32FullScaleRange = 6.89476; // 100 psi in bar

// M32 sensor, see PERFORMANCE SPECIFICATION (DIGITAL). Could be to be adjusted
const int16_t M32MinScaleCounts = 1000;
const int16_t M32FullScaleCounts = 15000;
const int16_t M32Span=M32FullScaleCounts-M32MinScaleCounts;

// M32 sensor pressure style, gauge or differential. Comment out the wrong one.
// Differential
//const int16_t M32ZeroCounts=(M32MinScaleCounts+M32FullScaleCounts)/2;
// Absolute, Gauge
const int16_t M32ZeroCounts=M32MinScaleCounts;

/*
 * end of M32 sensor characteristics
 */

// functions for debugging and printing

// function to print the bits of a variable
void printBinary(int number, uint8_t Length){
  static int bits;
  if ( bits > 64 ) {
    // bits cant reach 64
    Serial.print(F("64 loops done, failed somewhere. Debug: bits:"));
    Serial.print(bits);
    Serial.print(F(" lenght: "));
    Serial.print(Length);
    Serial.print(F(" number: "));
    Serial.println(number);
    bits=0;
    number=0;
    return;
  }
  Serial.flush();
  if (number) { //The remaining bits have a value greater than zero continue
    bits++; // Count the number of bits so we know how many leading zeros to print first
    printBinary(number >> 1, Length); // Remove a bit and do a recursive call this function.
    if (bits) for (uint8_t x = (Length - bits); x; x--)Serial.write('0'); // Add the leading zeros
    bits = 0; // clear no need for this any more
    Serial.write((number & 1) ? '1' : '0'); // print the bits in reverse order as we depart the recursive function
  }
}

// function to write the float/double value to Serial (seems unneeded)
char* tempToAscii(float temp)
// convert long to type char and store in variable array ascii
{
  char ascii[20];// needs to be this big to hold a type float

  int frac;
  int rnd;

    rnd = (unsigned int)(temp*1000)%10;
    frac=(unsigned int)(temp*100)%100;  //get three numbers to the right of the deciaml point.
    if (rnd>=5) frac=frac+1;
    itoa((int)temp,ascii,10);           // I changed it to 2 decimal places
    strcat(ascii,".");

  if (frac<10)  {itoa(0,&ascii[strlen(ascii)],10); }   // if fract < 10 should print .0 fract ie if fract=6 print .06

    itoa(frac,&ascii[strlen(ascii)],10); //put the frac after the deciaml
    
  return ascii;
}

// fetch_i2cdata is the core function to do the I2C read and extraction of the three data fields
byte fetch_i2cdata(double &pressure, double &temperature, byte rawdata[])
{
uint8_t buffersize;
byte _status;
byte Press_H;
byte Press_L;
byte Temp_H;
byte Temp_L;

uint16_t P_dat; // 14 bit pressure data
uint16_t T_dat; // 11 bit temperature data

// wake sensor
// if not performed, 1 reading every 2 is stale
Wire.requestFrom(0x28, 0);
// read data
// the casting is needed because Wire.requestFrom require int OR uint_8t, not a mix of such
// a previous version was asking also for stop=true, but is creating Stale issue along with the above wake sensor
buffersize = Wire.requestFrom(M32Address, static_cast<uint8_t>(4)); //Request 4 bytes, 2 pressure/status and 2 temperature
// buffersize = 4 expected, otherwise it's an error
if ( buffersize != 4 ) {
  return 4;
}
rawdata[0] = Wire.read();
rawdata[1] = Wire.read();
rawdata[2] = Wire.read();
rawdata[3] = Wire.read();

// as the variable Press_H/L, Temp_H/L were meant to be modified, have to save the rawdata into another variable
Press_H = rawdata[0];
Press_L = rawdata[1];
Temp_H = rawdata[2];
Temp_L = rawdata[3];

// get the return code, first 2 bit of the first i2c packet
// 00: OK
// 01: Reserved
// 10: Stale
// 11: Error
_status = (Press_H >> 6) & 0x03;

// keep in Press_H only the meaningful data for pressure, first byte
Press_H = Press_H & 0x3f;
// combine Press_H and Press_L to get the 14 bit precision
P_dat = (((uint16_t)Press_H) << 8 ) | Press_L; // space added after ‘8’ so code can be uploaded without emoji conversion!

// the low part of the temperature must remove the last 5 bits, as that are always 0 (unused)
Temp_L = (Temp_L >> 5);
// the correct temperature must take the high part, adding 3 bit on the end, and then the remaining 3 bit on Temp_L
T_dat = (((uint16_t)Temp_H) << 3) | Temp_L;

// Print the data on Serial for monitoring (unneeded, anyhow)

switch (_status)
{
case 0:
//Serial.println("Ok ");
break;
case 1:
Serial.println("M32 i2c Busy 01");
break;
case 2:
Serial.println("M32 i2c Slate 10");
break;
default:
Serial.println("M32 i2c Error 11");
break;
}

// print raw data
Serial.println("raw data. S: Status, P: Pressure, T: Temperature, x: unused");
Serial.println("SSPPPPPP PPPPPPPP TTTTTTTT TTTxxxxx");
printBinary(rawdata[0], 8);
Serial.print(" ");
printBinary(rawdata[1], 8);
Serial.print(" ");
printBinary(rawdata[2], 8);
Serial.print(" ");
printBinary(rawdata[3], 8);
Serial.println("\n");

// Print debug data for pressure
Serial.print("raw pressure (dec count): ");
Serial.print(P_dat);
Serial.print(", ");
Serial.print("raw pressure (binary): ");
printBinary(P_dat, 14);
Serial.println("\n");

// Print debug data for temperature 
Serial.print("raw temperature (dec count): ");
Serial.print(T_dat);
Serial.print(", ");
Serial.print("temperature (binary): ");
printBinary(T_dat, 11);
Serial.println("\n");

// make the math on pressure
// static_cast<double> is needed for each variable to avoid that Arduino decide to make use of int or such
pressure=(static_cast<double>(static_cast<double>(P_dat)-M32ZeroCounts))/static_cast<double>(M32Span)*static_cast<double>(M32FullScaleRange);

// Original formula in the datasheet
// output (decimal counts) = ( output °C + 50°C ) * 2048 / (150°C - (-50°C))
/* output °C	Digital counts (Dec)	Digital counts
   0		 512			0x200
  10		 614			0x266
  25		 767			0x2FF
  40		 921			0x399
  55		1075			0x433 */
// reversed formula from datasheet as we have count and want temperature
// output °C = ( output (decimal counts) / 2048 * 200 ) - 50
// without static_cast<double> the values are mocked (eg 798*200 = 28528, while should be 153600
temperature = ( static_cast<double>(T_dat) * static_cast<double>(200) / static_cast<double>(2048) ) - static_cast<double>(50);

// Finally print converted data

Serial.print("pressure bar: ");
Serial.println(pressure);
Serial.print("temperature °C: ");
Serial.println(temperature);

return _status;
}

// setup is the main function to setup the diagnostic serial port and the I2C port
void setup()
{

Serial.begin(115200);
Wire.begin();
// wait until serial port opens for native USB devices
// Arduino Uno/Ethernet/Mega does not need it (no CDC serial)
while (! Serial)
{
delay(1);
}

Serial.println("M32 test");

}

// main()
void loop()
{
// variabled defined here and passed as reference to fetch_i2cdata
byte _status; // A two bit field indicating the status of the I2C read
double pressure;
double temperature;
byte rawdata[4];

_status = fetch_i2cdata(pressure, temperature, rawdata);


delay(1000);

}
