#include <Wire.h>


int M3200address = 0x28;   // 0x28, 0x36 or 0x46, depending on the sensor.
float maxPressure =100;  // pressure in PSI for this sensor, 100, 250, 500, ... 10k.
byte status;


unsigned long pressureMillis = 0;
const int pressureTiming = 500; // 500ms every data acquisition


void setup (){
Serial.begin(115200);
Serial.println("M3200 initializing");
Wire.begin();
}


void wireOutside() {
  Wire.requestFrom(M3200address,4);   // request 4 bytes
  int n = Wire.available();
    if( n == 4){
      status = 0;
      uint16_t rawP;     // pressure data from sensor
      uint16_t rawT;     // temperature data from sensor
      rawP = (uint16_t) Wire.read();    // upper 8 bits
      rawP <<= 8;
      rawP |= (uint16_t) Wire.read();    // lower 8 bits
      rawT = (uint16_t)  Wire.read();    // upper 8 bits
      rawT <<= 8;
      rawT |= (uint16_t) Wire.read();   // lower 8 bits
  
  
      status = rawP >> 14;   // The status is 0, 1, 2 or 3
      rawP &= 0x3FFF;   // keep 14 bits, remove status bits
  
  
      rawT >>= 5;     // the lowest 5 bits are not used
    
   // The math could be done with integers, but I choose float for now
      float pressure = ((rawP - 1000.0) / (15000.0 - 1000.0)) * maxPressure;
      float temperature = ((rawT - 512.0) / (1075.0 - 512.0)) * 55.0;
  
  
      Serial.print("Status = ");
      Serial.println(status);
      Serial.print( "Pressure = ");
      Serial.println( pressure);
      Serial.print( "Temperature = ");
      Serial.println( temperature);
      Serial.println("-------------------------------------");
  
  
    }else{
      Serial.println( "M3200 Pressure Transducer not Detected");
    }
}


void loop(){
  if (millis()-pressureMillis >= pressureTiming){
    pressureMillis = millis();
  }
  wireOutside();
  delay(1000);
  
}
