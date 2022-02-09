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
// for the watchdog
#include <avr/wdt.h>
// variable for the local project
#include "boiler.h"

/*
 * MQTT settings
 */
// it's a public name space, so open the key & secret value.
// if you want to make a privte one, you should make a private namespace on http://shiftr.io
//#define MQTT_DeviceName "arduino_boiler"
//#define MQTT_Username "boiler"
//#define MQTT_Password "boiler"
//#define MQTT_topic_Message  "/boiler"

// tested up to 163 chars
// {"temperature":23.83,"temperature_unit":"°C","pressure":1.11,"pressure_unit":"bar","uptime":4294967000,"mqttRawData":["00000011","11001001","01011110","10000000"]}
// Sketch uses 32180 bytes (99%) of program storage space. Maximum is 32256 bytes.
// Global variables use 1063 bytes (51%) of dynamic memory, leaving 985 bytes for local variables. Maximum is 2048 bytes.
#define BUFFERSIZE 200


/*
 * M32 sensor characteristics (from the 02/2021 version of the data sheet)
 */

// M32 sensor I2C address
//const uint8_t M32Address PROGMEM = 0x28;
#define M32Address 0x28

// M32 sensor full scale range and units
const int16_t M32FullScaleRange = 6.89476; // 100 psi in bar

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
#define M32ZeroCounts M32MinScaleCounts

/*
 * end of M32 sensor characteristics
 */

/*
 * Other variables
 */
// trick: mac must be "unicast" + "locally administred", eg xE‑xx‑xx‑xx‑xx‑xx
const byte mac[] = {0x6E, 0x61, 0x6C, 0x64, 0x61, 0x69};

EthernetClient net;
IPAddress ip;
uint8_t ipObtained = false;

MQTTClient mqttClient(BUFFERSIZE);
//#define MQTTBROKER "mqtt.example.com"
//#define MQTTPORT = 1883
//#define MQTTPREFIX ""

unsigned long now = millis();


// time in ms expected to run the sketch
#define RUNTIME 48

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
  } else {
    // corner case, the hex is filled by 0
    if ( bits == 0 ) {
      for( int count = 0; count < Length; count++) {
        Serial.write('0');
      }
    }
  }
}

char * int2bin(int x)
{
  static char buffer[9];
  for (int i=0; i<8; i++) buffer[7-i] = '0' + ((x & (1 << i)) > 0);
  buffer[8] ='\0';
  return buffer;
}

// rounds a number to 2 decimal places
// example: round(3.14159) -> 3.14
double round2(double value) {
   return (int)(value * 100 + 0.5) / 100.0;
}
byte initializeEthernet() {
  Serial.println(F("Trying to configure Ethernet using DHCP"));
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Failed to initialize Ethernet using DHCP"));

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {

      Serial.println(F("Ethernet shield was not found.  Sorry, can't run without hardware. :("));

    } else if (Ethernet.linkStatus() == LinkOFF) {

      Serial.println(F("Ethernet cable is not connected."));
    }

    delay(10000);
  }
  Serial.println(F("Initialized Ethernet using DHCP"));
  // assign value to the global value that give me current status
  if ( checkDhcp() == false ) {
    Serial.println(F("Failed to configure Ethernet using DHCP"));
    delay(1000);
  }
  ipObtained=true;
  // dhcp test end

  Serial.println(F("Network configuration succeded"));

  // is it needed?
  byte macBuffer[6];  // create a buffer to hold the MAC address
  Ethernet.MACAddress(macBuffer); // fill the buffer
  Serial.print(F("MAC: "));
  for (byte octet = 0; octet < 6; octet++) {
    Serial.print(macBuffer[octet], HEX);
    if (octet < 5) {
      Serial.print(F("-"));
    }
  }
  Serial.println(F("\n"));

  // print your local IP address:
  Serial.print(F("IP: "));
  Serial.println(Ethernet.localIP());
  Serial.print(F("Gateway: "));
  Serial.println(Ethernet.gatewayIP());
  Serial.print(F("Subnet: "));
  Serial.println(Ethernet.subnetMask());
  Serial.print(F("DNS: "));
  Serial.println(Ethernet.dnsServerIP());
  Serial.println();

  return;
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
  Serial.print(F("connecting..."));

  while (!mqttClient.connect(MQTT_DeviceName, MQTT_Username, MQTT_Password)) {
//  while (!mqttClient.connect(MQTT_DeviceName, MQTT_Username, MQTT_Password) || counter == 60) {
    Serial.print(F("."));
    //counter++;
    delay(1000);
  }

  Serial.println(F("\r\nconnected!"));

  mqttClient.subscribe(MQTT_topic_Message);
}

byte MQTTSend(double &pressure, double &temperature, byte rawdata[]) {
  // variable for the JSON message to MQTT
  StaticJsonDocument<BUFFERSIZE-8> mqttData;
  char json_string[BUFFERSIZE-8];
  byte mqttreturn;

  // write data to mqtt variables
  mqttData["temperature"] = temperature;
  mqttData["temperature_unit"] = "°C";

  mqttData["pressure"] = pressure;
  mqttData["pressure_unit"] = "bar";

  mqttData["uptime"] = now;

  JsonArray mqttRawData = mqttData.createNestedArray("mqttRawData");
  mqttRawData.add(int2bin(rawdata[0]));
  mqttRawData.add(int2bin(rawdata[1]));
  mqttRawData.add(int2bin(rawdata[2]));
  mqttRawData.add(int2bin(rawdata[3]));

  serializeJson(mqttData, json_string);

  Serial.print(F("JSON String (len: "));
  Serial.print(strlen(json_string));
  Serial.println(F("):"));
  Serial.println(json_string);

  mqttClient.loop();

  if ( ! mqttClient.connected() ) {
    mqttConnect();
  }

//  mqttreturn = mqttClient.publish("/boiler", json_string);
  mqttreturn = mqttClient.publish(MQTT_topic_Message, json_string);
  if ( !mqttreturn ) {
    Serial.println(F("MQTT send message failed"));
  } else {
    Serial.println(F("MQTT send message OK"));
  }

  return mqttreturn;
}

uint8_t resetStale() {
  // wake sensor
  // if not performed, 1 reading every 2 is stale
  Serial.println(F("Read from i2c trying to remove the stale"));
  delay(100*1.2);
  //return Wire.requestFrom(M32Address, (uint8_t)0);
  return Wire.requestFrom((uint8_t)M32Address, (uint8_t)0);
  delay(100*1.2);
}


// fetch_i2cdata is the core function to do the I2C read and extraction of the three data fields
byte fetch_i2cdata(double &pressure, double &temperature, byte rawdata[]) {
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
  //Wire.requestFrom(M32Address, (uint8_t)0);
  Wire.requestFrom((uint8_t)M32Address, (uint8_t)0);

  // standard time for i2c is 100Khz
  // from the datasheet:
  // Users  that  use  “status  bit”  polling  should  select  a  frequency  slower  than  20%  more  than  the  update  time.
  delay(100*1.2);

  // read data
  // the casting is needed because Wire.requestFrom require int OR uint_8t, not a mix of such
  // a previous version was asking also for stop=true, but is creating Stale issue along with the above wake sensor
  buffersize = Wire.requestFrom((uint8_t)M32Address, static_cast<uint8_t>(4)); //Request 4 bytes, 2 pressure/status and 2 temperature
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
  //Serial.println(F("Ok "));
  break;
  case 1:
  Serial.println(F("M32 i2c Busy 01"));
  break;
  case 2:
  Serial.println(F("M32 i2c Slate 10"));
  resetStale();
  break;
  default:
  Serial.println(F("M32 i2c Error 11"));
  break;
  }

  // print raw data
  Serial.println(F("raw data. S: Status, P: Pressure, T: Temperature, x: unused"));
  Serial.println(F("SSPPPPPP PPPPPPPP TTTTTTTT TTTxxxxx"));
  printBinary(rawdata[0], 8);
  Serial.print(F(" "));
  printBinary(rawdata[1], 8);
  Serial.print(F(" "));
  printBinary(rawdata[2], 8);
  Serial.print(F(" "));
  printBinary(rawdata[3], 8);
  Serial.println(F("\n"));

  // Print debug data for pressure
  Serial.print(F("raw pressure (dec count): "));
  Serial.print(P_dat);
  Serial.print(F(", "));
  Serial.print(F("raw pressure (binary): "));
  printBinary(P_dat, 14);
  Serial.println(F(""));

  // Print debug data for temperature
  Serial.print(F("raw temperature (dec count): "));
  Serial.print(T_dat);
  Serial.print(F(", "));
  Serial.print(F("temperature (binary): "));
  printBinary(T_dat, 11);
  Serial.println(F("\n"));

  // make the math on pressure
  // static_cast<double> is needed for each variable to avoid that Arduino decide to make use of int or such
  pressure=round2((static_cast<double>(static_cast<double>(P_dat)-M32ZeroCounts))/static_cast<double>(M32Span)*static_cast<double>(M32FullScaleRange));

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
  temperature = round2(( static_cast<double>(T_dat) * static_cast<double>(200) / static_cast<double>(2048) ) - static_cast<double>(50));

  // Finally print converted data

  Serial.print(F("pressure bar: "));
  Serial.println(pressure);
  Serial.print(F("temperature °C: "));
  Serial.println(temperature);

  return _status;
}


// setup is the main function to setup the diagnostic serial port and the I2C port
void setup()
{
  // Initialize (pin used) the boot of ethernet
  Ethernet.init(10);  // Most Arduino shields
  Serial.begin(115200);
  Wire.begin();
  // wait until serial port opens for native USB devices
  // Arduino Uno/Ethernet/Mega does not need it (no CDC serial)
  while (! Serial)
  {
  delay(1);
  }

  // setting up the watchdog for 8 seconds, BEFORE the ethernet initialization
  wdt_enable(WDTO_8S);

  // start the Ethernet connection:
  initializeEthernet();

  // reset the watchdog
  wdt_reset();

  // start the MQTT client
  //mqttClient.begin(broker, net);
  mqttClient.begin(MQTTBROKER, net);
  //mqttClient.begin("homeassistant.dmz.gonzaga.retaggio.net", net);
  mqttConnect();

  // reset the watchdog
  wdt_reset();
}

// main()
void loop()
{
  // variabled defined here and passed as reference to fetch_i2cdata
  byte _status; // A two bit field indicating the status of the I2C read
  double pressure;
  double temperature;
  byte rawdata[4];

  // update the now variable
  now = millis();
  // temp debug to see if the messages are going through near the end of the uptime
  //now = 4294967000;

  Serial.print(F("\r\n\nCurrent uptime: "));
  Serial.println(now);

  // reset the watchdog
  wdt_reset();


  // get updated data from the sensor
  _status = fetch_i2cdata(pressure, temperature, rawdata);

  _status = MQTTSend(pressure, temperature, rawdata);
  // watchdog to 8s
  delay(5000-RUNTIME);
}
