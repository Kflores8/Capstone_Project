/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/Brosana/Documents/IoT/Capstone_Project/Capstone_Temperature/src/Capstone_Temperature.ino"
/*
 * Project Capstone_Temperature
 * Description:
 * Author:
 * Date:
 */

#include "spark-dallas-temperature.h"
#include "OneWire.h"
#include "DS18.h"

void setup();
void loop();
void printDebugInfo();
#line 12 "/Users/Brosana/Documents/IoT/Capstone_Project/Capstone_Temperature/src/Capstone_Temperature.ino"
DS18 sensor(A5);

void setup() {
  Serial.begin(9600);
  // Set up 'power' pins, comment out if not used!
  pinMode(A5, OUTPUT);
  digitalWrite(A5, HIGH);
}

void loop() {
  // Read the next available 1-Wire temperature sensor
  if (sensor.read()) {
    // Do something cool with the temperature
    Serial.printf("Temperature %.2f C %.2f F ", sensor.celsius(), sensor.fahrenheit());
    Particle.publish("temperature", String(sensor.celsius()), PRIVATE);

    // Additional info useful while debugging
    printDebugInfo();

  // If sensor.read() didn't return true you can try again later
  // This next block helps debug what's wrong.
  // It's not needed for the sensor to work properly
  } else {
    // Once all sensors have been read you'll get searchDone() == true
    // Next time read() is called the first sensor is read again
    if (sensor.searchDone()) {
      Serial.println("No more addresses.");
      // Avoid excessive printing when no sensors are connected
      delay(250);

    // Something went wrong
    } else {
      printDebugInfo();
    }
  }
  Serial.println();
}

void printDebugInfo() {
  // If there's an electrical error on the 1-Wire bus you'll get a CRC error
  // Just ignore the temperature measurement and try again
  if (sensor.crcError()) {
    Serial.print("CRC Error ");
  }

  // Print the sensor type
  const char *type;
  switch(sensor.type()) {
    case WIRE_DS1820: type = "DS1820"; break;
    case WIRE_DS18B20: type = "DS18B20"; break;
    case WIRE_DS1822: type = "DS1822"; break;
    case WIRE_DS2438: type = "DS2438"; break;
    default: type = "UNKNOWN"; break;
  }
  Serial.print(type);

  // Print the ROM (sensor type and unique ID)
  uint8_t addr[8];
  sensor.addr(addr);
  Serial.printf(
    " ROM=%02X%02X%02X%02X%02X%02X%02X%02X",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]
  );

  // Print the raw sensor data
  uint8_t data[9];
  sensor.data(data);
  Serial.printf(
    " data=%02X%02X%02X%02X%02X%02X%02X%02X%02X",
    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]
  );
}
