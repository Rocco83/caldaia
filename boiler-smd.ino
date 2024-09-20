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
#include <SPI.h>
#include <Ethernet.h>
#include <MQTT.h>
#include <ArduinoJson.h>
// Arduino R4 watchdog
#include <WDT.h>
// variable for the local project
#include "boiler.h"

// tested up to 163 chars
// {"temperature":23.83,"temperature_unit":"°C","pressure":1.11,"pressure_unit":"bar","uptime":4294967000,"binData":["00000011","11001001","01011110","10000000"]}
// Sketch uses 32180 bytes (99%) of program storage space. Maximum is 32256 bytes.
// Global variables use 1063 bytes (51%) of dynamic memory, leaving 985 bytes for local variables. Maximum is 2048 bytes.
#define BUFFERSIZE 180

/*
 * M32 sensor characteristics (from the 02/2021 version of the data sheet)
 */

// datasheet 1.6.1:
// "Users  that  use  “status  bit”  polling  should  select  a  frequency  slower  than  20%  more  than  the  update  time."
// delay count in milliseconds
// 100 KHz (standard i2c) = 100000 read / sec, 100 read/millisecond
// theoretically, 1 read / ms would be just enough, so delay(1)
// update time: 
#define I2CPOLLWAIT 100*1.2

// M32 sensor I2C address
const uint8_t M32Address PROGMEM = 0x28;
// this is normally used, keeping it as a note
//int M32Address = 0x28;   // 0x28, 0x36 or 0x46, depending on the sensor.
//#define M32Address 0x28

// M32 sensor full scale range and units
#define M32FullScaleRange 6.89476 // 100 psi in bar

// M32 sensor, see PERFORMANCE SPECIFICATION (DIGITAL). Could be to be adjusted
//const int16_t M32MinScaleCounts PROGMEM = 1000;
//const int16_t M32FullScaleCounts PROGMEM = 15000;
//const int16_t M32Span = M32FullScaleCounts-M32MinScaleCounts;
#define M32MinScaleCounts 1000
#define M32FullScaleCounts 15000
const int16_t M32Span PROGMEM = M32FullScaleCounts-M32MinScaleCounts;

// M32 sensor pressure style, gauge or differential. Comment out the wrong one.
// Differential
//const int16_t M32ZeroCounts=(M32MinScaleCounts+M32FullScaleCounts)/2;
// Absolute, Gauge
//const int16_t M32ZeroCounts = M32MinScaleCounts;
// Absolute, but doing by #define
#define M32ZeroCounts M32MinScaleCounts

/*
 * end of M32 sensor characteristics
 */

/*
 * Network variables
 */
// trick: mac must be "unicast" + "locally administred", eg xE‑xx‑xx‑xx‑xx‑xx
//const byte mac[] = {0x6E, 0x61, 0x6C, 0x64, 0x61, 0x69};
byte mac[] = {0x6E, 0x61, 0x6C, 0x64, 0x61, 0x69};

EthernetClient net;
IPAddress ip;
uint8_t ipObtained = false;

MQTTClient mqttClient(BUFFERSIZE);

/*
 * Watchdog variables
 */
// watchdog related, set 5 sec as timeout
// UNO R4 min wdtInterval 1ms / max wdtInterval 5592ms
const long wdtInterval = 5592;

// marking the uptime
unsigned long now = millis();

/*
 * Adafruit TCA4307 - i2c recovery
 */
uint8_t tca4307_ready = 12; // purple
uint8_t tca4307_enable = 13; // gray

/*
 * Other variables
 */

// global variable used for all the functions
double pressure;
double temperature;
uint16_t P_dec;
uint16_t T_dec;
byte rawdata[4];

// time in ms expected to run the sketch
//#define RUNTIME 48

/*
 * functions for debugging and printing
 */

// function to print the bits of a variable
// TODO currently used only in debug sections
void printBinary(int number, uint8_t Length){
  static int bits;
  if ( bits > 64 ) {
    // bits can't reach 64
    Serial.print(F("64 loops done, failed. bits:"));
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
  /* number (hex) is the real data to be converted in 1 and 0
   * this is called recursively, when 0 is hitted it means that all the remaining bits are 0
   */
  if (number) {
    bits++; // Count the number of bits so we know how many leading zeros to print first
    printBinary(number >> 1, Length); // Remove a bit and do a recursive call this function.
    if (bits) for (uint8_t x = (Length - bits); x; x--) Serial.write('0'); // Add the leading zeros
    bits = 0; // clear no need for this any more
    Serial.write((number & 1) ? '1' : '0'); // print the bits in reverse order as we depart the recursive function
  } else {
    // corner case, number (hex) is filled by 0
    // print all the remaining numbers with 0 for padding
    if ( bits == 0 ) {
      for( int count = 0; count < Length; count++) {
        Serial.write('0');
      }
    }
  }
}

char * int2bin(uint8_t x) {
  static char buffer[9];
  for (int i=0; i<8; i++) buffer[7-i] = '0' + ((x & (1 << i)) > 0);
  buffer[8] ='\0';
  return buffer;
}

// declare reset function at address 0
// calling this trigger a *soft* reset
void(* resetFunc) (void) = 0;


// rounds a number to 2 decimal places
// example: round(3.14159) -> 3.14
double round2(double value) {
   return (int)(value * 100 + 0.5) / 100.0;
}

// initialize the ethernet from setup()
byte initializeEthernet() {
  Serial.println(F("DHCP init"));
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("DHCP init failed"));

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {

      Serial.println(F("Ethernet shield was not found"));

    } else if (Ethernet.linkStatus() == LinkOFF) {

      Serial.println(F("Ethernet cable not connected"));
    }

    delay(5000);
    resetFunc();
  }
  Serial.println(F("DHCP init OK"));
  // assign value to the global value that give me current status
  if ( checkDhcp() == false ) {
    Serial.println(F("DHCP config failed"));
    delay(1000);
    resetFunc();
  }
  ipObtained=true;
  // dhcp test end

  Serial.println(F("Network configuration OK"));

  // is it needed?
  byte macBuffer[6];  // create a buffer to hold the MAC address
  Ethernet.MACAddress(macBuffer); // fill the buffer
  Serial.print(F("MAC: "));
  for (byte octet = 0; octet < 6; octet++) {
    Serial.print(macBuffer[octet], HEX);
    if (octet < 5) {
      Serial.print(F(":"));
    }
  }
  Serial.println(F("\n"));

  // print your local IP address:
  Serial.print(F("IP: "));
  Serial.println(Ethernet.localIP());
  Serial.print(F("GW: "));
  Serial.println(Ethernet.gatewayIP());
  Serial.print(F("Subnet: "));
  Serial.println(Ethernet.subnetMask());
  Serial.print(F("DNS: "));
  Serial.println(Ethernet.dnsServerIP());
  Serial.println();

  return 0;
}

byte checkDhcp() {
    /*
     * 0: nothing happened
     * 1: renew failed
     * 2: renew success
     * 3: rebind fail
     * 4: rebind success
     */
    //Serial.println(F("Calling maintain"));
    ipObtained = Ethernet.maintain();
    switch( ipObtained ){
    case 0:
      return true;
      break;
    case 1:
      return false;
      break;
    case 2:
      return true;
      break;
    case 3:
      return false;
      break;
    case 4:
      return true;
      break;
    default:
      return false;
      break;
    }
}

void mqttConnect() {
  Serial.print(F("Connecting to MQTT broker: "));
  Serial.println(MQTTBROKER);

  // watchdog shall come in play and make a full reset
  while (!mqttClient.connect(MQTT_DeviceName, MQTT_Username, MQTT_Password)) {
  //while (!mqttClient.connect(MQTT_DeviceName, MQTT_Username, MQTT_Password) || counter == 60) {
    Serial.print(F("."));
    //counter++;
    delay(1000);
  }

  Serial.println(F("\r\nconnected"));

  // not needed, only send
  //mqttClient.subscribe(MQTT_topic_Message);
}

byte MQTTSend() {
  // variable for the JSON message to MQTT
  StaticJsonDocument<BUFFERSIZE-8> mqttData;
  char json_string[BUFFERSIZE-8];
  byte mqttreturn;

  // write data to mqtt variables
  mqttData["temperature"] = temperature;
  mqttData["temperature_unit"] = "C";

  mqttData["pressure"] = pressure;
  mqttData["pressure_unit"] = "bar";

  mqttData["uptime"] = now;

  mqttData["p_dec"] = (uint16_t)P_dec;
  mqttData["t_dec"] = (uint16_t)T_dec;

  // removed as the memory of arduino ethernet does not allow a very long string
  /*
  JsonArray binData = mqttData.createNestedArray("binData");
  binData.add(int2bin(rawdata[0]));
  binData.add(int2bin(rawdata[1]));
  binData.add(int2bin(rawdata[2]));
  binData.add(int2bin(rawdata[3]));
  */

  serializeJson(mqttData, json_string);

  Serial.print(F("JSON String (len: "));
  Serial.print(strlen(json_string));
  Serial.println(F("):"));
  Serial.println(json_string);

  mqttClient.loop();

  if ( ! mqttClient.connected() ) {
    mqttConnect();
  }

  mqttreturn = mqttClient.publish(MQTT_topic_Message, json_string);
  if ( !mqttreturn ) {
    Serial.println(F("MQTT send failed"));
  } else {
    Serial.println(F("MQTT send OK"));
  }

  return mqttreturn;
}

/*
 * Perform an hardware i2c reset through the tca4307 module
 * EN - This is the Enable input pin. Allows you to disconnect the in and out sides
 *      when pulled low
 * LOW: GND
 * HIGH: VCC
 */
void tca4307Reset() {
  // disable i2c
  Serial.println(F("tca4307Reset: disable i2c"));
  Serial.print(F("tca4307Reset: pin"));
  Serial.println(tca4307_enable);
  digitalWrite(tca4307_enable, LOW); // GND pulling down the pin 13
  delay(3000);
  // enable i2c
  Serial.println(F("tca4307Reset: enable i2c"));
  digitalWrite(tca4307_enable, HIGH); // 5VDC same as the pin 13
}

uint8_t resetStale() {
  byte byteread;
  // wake sensor
  // if not performed, 1 reading every 2 is stale
  Serial.println(F("resetStale: Read from i2c, try to remove the stale (beginTransmission and endTransmission)"));
  delay(I2CPOLLWAIT);
  // this is equivalent to beginTransfer and endTransfer
  //Wire.requestFrom((uint8_t)M32Address, (uint8_t)0);
  Wire.beginTransmission((uint8_t) M32Address);
  byteread = Wire.endTransmission();
  if(byteread != 0x00) {
    Serial.println(F("Missing i2c device. Sleeping 1s and resetting i2c through tca430"));
    Serial.flush();
    // TODO reset i2c bus with EN pindisable 
    tca4307Reset();
    return 1;
  } else {
    Serial.println(F("i2c device found, continue"));
    Serial.flush();
    return 0;
  }
  // this is never reached
  return 0;
}

// fetch_i2cdata is the core function to do the I2C read and extraction of the three data fields
byte fetch_i2cdata() {
  uint8_t buffersize;
  // A two bit field indicating the status of the I2C read
  byte i2c_status;
  byte Press_H;
  byte Press_L;
  byte Temp_H;
  byte Temp_L;

  //uint16_t P_dec; // 14 bit pressure data
  //uint16_t T_dec; // 11 bit temperature data
  P_dec = 0;
  T_dec = 0;

  // tca4307 readiness
  byte tca4307_status;

/*
 * Datasheet exception to i2c
 * - Sending a start-stop condition without any transitions on the SCL line (no clock pulses in between) creates a communication error for
 *   the next communication, even if the next start condition is correct and the clock pulse is applied. An additional start condition must be
 *   sent, which results in restoration of proper communication.
 * - The restart condition – a falling SDA edge during data transmission when the SCL clock line is still high – creates the same situation.
 *   The next communication fails, and an additional start condition must be sent for correct communication.
 * - A falling SDA edge is not allowed between the start condition and the first rising SCL edge. If using an I2C address with the first bit 0,
 *   SDA must be held down from the start condition through the first bit. 
 */

  // with tca-4307
  // EN - This is the Enable input pin. Allows you to disconnect the in and out sides when pulled low
  // RDY - This is the Ready output pin. It will let you know if the peripheral is buffer-connected to the controller (and is safe to attempt communication).

  tca4307_status = digitalRead(tca4307_ready);
  Serial.print(F("tca4307 readiness: "));
  Serial.println(tca4307_status);

  Serial.print(F("**Loop** Reading from i2c bus on device address: "));
  Serial.println((uint8_t) M32Address);
  buffersize = 0;
  while ( buffersize != 4) {
    delay(I2CPOLLWAIT);
    Wire.requestFrom((uint8_t) M32Address, 4); // Request 4 bytes, 2 pressure/status and 2 temperature
    buffersize = Wire.available();
    Serial.print(F("**Loop** i2c bus returned "));
    Serial.print(buffersize);
    Serial.print(F(" bytes: "));
    if ( buffersize != 4) {
      Serial.println(F("Error, expected 4 bytes"));
      Serial.flush();
      resetStale();
    } else {
      Serial.println(F("OK"));
    }
  }
  // reset the watchdog in case the read was very slow / with multiple errors
  WDT.refresh();


  // the buffersize is 4, therefore we need to read from Wire 4 times
  // rawdata[] is needed to print out the data afterward
  Serial.println(F("reading buffered i2c byte 1/4"));
  //delay(I2CPOLLWAIT);
  rawdata[0] = Wire.read();
  //DEBUG Serial.println(rawdata[0]);
  Serial.println(F("reading buffered i2c byte 2/4"));
  //delay(I2CPOLLWAIT);
  rawdata[1] = Wire.read();
  Serial.println(F("reading buffered i2c byte 3/4"));
  //delay(I2CPOLLWAIT);
  rawdata[2] = Wire.read();
  Serial.println(F("reading buffered i2c byte 4/4"));
  //delay(I2CPOLLWAIT);
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
  i2c_status = (Press_H >> 6) & 0x03;

  // keep in Press_H only the meaningful data for pressure, first byte
  Press_H = Press_H & 0x3f;
  // combine Press_H and Press_L to get the 14 bit precision
  P_dec = (((uint16_t)Press_H) << 8 ) | Press_L; // space added after ‘8’ so code can be uploaded without emoji conversion!

  // the low part of the temperature must remove the last 5 bits, as that are always 0 (unused)
  Temp_L = (Temp_L >> 5);
  // the correct temperature must take the high part, adding 3 bit on the end, and then the remaining 3 bit on Temp_L
  T_dec = (((uint16_t)Temp_H) << 3) | Temp_L;

  // Print the data on Serial for monitoring (unneeded, anyhow)
  switch (i2c_status) {
    case 0:
      Serial.println(F("M32 ic2 data OK"));
      break;
    case 1:
      Serial.println(F("M32 i2c data error 01: Busy"));
      break;
    case 2:
      Serial.println(F("M32 i2c data error 10: Stale"));
      resetStale();
      // continue with the reading that was used
      //i2c_status = 0;
      // ignore the loop as the data are old
      return 1;
      break;
    default:
      Serial.println(F("M32 i2c data error 11: Error"));
      // TODO should be a power reset needed at this point?
      //resetStale();
      tca4307Reset();
      break;
  }

  // print raw data on serial
  Serial.println(F("raw data. S: Status, P: Pressure, T: Temperature, x: unused"));
  Serial.println(F("SSPPPPPP PPPPPPPP TTTTTTTT TTTxxxxx"));
  Serial.print(int2bin(rawdata[0]));
  Serial.print(F(" "));
  Serial.print(int2bin(rawdata[1]));
  Serial.print(F(" "));
  Serial.print(int2bin(rawdata[2]));
  Serial.print(F(" "));
  Serial.print(int2bin(rawdata[3]));
  Serial.println(F("\n"));

  // Print debug data for pressure
  Serial.print(F("raw pressure (dec count): "));
  Serial.print(P_dec);
  /* debug
  Serial.print(F(", "));
  Serial.print(F("raw pressure (binary): "));
  printBinary(P_dec, 14);
  */
  Serial.println(F(""));

  // Print debug data for temperature
  Serial.print(F("raw temp (dec count): "));
  Serial.print(T_dec);
  /* debug
  Serial.print(F(", "));
  Serial.print(F("temp (binary): "));
  printBinary(T_dec, 11);
  */
  Serial.println(F("\n"));

  // make the math on pressure
  // static_cast<double> is needed for each variable to avoid that Arduino decide to make use of int or such
  pressure = round2((static_cast<double>(static_cast<double>(P_dec)-M32ZeroCounts))/static_cast<double>(M32Span)*static_cast<double>(M32FullScaleRange));

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
  temperature = round2(( static_cast<double>(T_dec) * static_cast<double>(200) / static_cast<double>(2048) ) - static_cast<double>(50));

  // Finally print converted data
  Serial.print(F("pressure bar: "));
  Serial.println(pressure);
  Serial.print(F("temperature °C: "));
  Serial.println(temperature);

  Serial.print(F("status: "));
  Serial.println(i2c_status);
  return i2c_status;
}


// setup is the main function to setup the diagnostic serial port and the I2C port
void setup() {
  // initialize tca4307
  pinMode(tca4307_enable, OUTPUT);  // sets the pin of the "enable" as output
  pinMode(tca4307_ready, INPUT);    // sets the pin of the readiness as input
  tca4307Reset();
  //digitalWrite(tca4307_enable, HIGH);
  // Debug pin
  pinMode(11, OUTPUT);  // sets the pin of the "enable" as output
  pinMode(10, OUTPUT);  // sets the pin of the "enable" as output
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);

  // Initialize (pin used) the boot of ethernet
  Ethernet.init(10);  // Most Arduino shields

  Serial.begin(115200);

  Wire.begin();
  // Standard clock is 100000 (100 KHz). M32 supports 50 KHz to 400 KHz
  //Wire.setClock(100000);
  // https://www.arduino.cc/reference/en/language/functions/communication/wire/setwiretimeout/
  // still not supported in R4 v1.2.1
  // supported in AVR
  //Wire.setWireTimeout(2000 /* us */, true /* reset_on_timeout */);

  // https://forum.arduino.cc/t/i2c-e-pullup-resistor/129142/4
  // Se l'I2C è a 5V puoi anche lasciarle attive, 30 k in parallelo a 4.7k hanno un effetto minimo, se l'I2C è a 3.3V allora devi disabilitare le pull up interne usando le due righe che ti ha postato PaoloP.
  //digitalWrite(SDA, 0);
  //digitalWrite(SCL, 0);
  // TODO how does this relate with the 4.7KOhm requested as it is open-drain in M32JM?
  // useless as we do have a i2c "splitter"

  // wait until serial port opens for native USB devices
  // Arduino Uno/Ethernet/Mega does not need it (no CDC serial)
  // This imply that without serial connection the software does not start?
/*
  while (! Serial) {
    delay(1);
  }
*/

  // setting up the watchdog, BEFORE the ethernet initialization
  if(WDT.begin(wdtInterval)) {
    WDT.refresh();
    Serial.print("WDT interval: ");
    Serial.print(WDT.getTimeout());
    Serial.println(" ms");
  } else {
    Serial.println("Error initializing watchdog");
    while(1){}
  }
  // reset the watchdog
  WDT.refresh();

  // start the Ethernet connection:
  initializeEthernet();
  // reset the watchdog
  WDT.refresh();

  // start the MQTT client
  mqttClient.begin(MQTTBROKER, net);
  mqttConnect();
  // reset the watchdog
  WDT.refresh();
}

// main()
void loop()
{
  // update the now variable
  now = millis();
  // temp debug to see if the messages are going through near the end of the uptime
  //now = 4294967000;

  // show the current uptime
  Serial.print(F("\r\n\nuptime: "));
  Serial.println(now);

  // reset the watchdog at the start of the loop
  WDT.refresh();

  // get updated data from the sensor
  // if return != 0, i2c failed (not anymore the original scope for the variable)
  if ( fetch_i2cdata() != 0 ) {
    return;
  }

  // function return not parsed
  MQTTSend();
  //delay(1000-RUNTIME);
  // TODO Fix the code?
  delay(2000);
}
