/*
 * Project Capstone_Qwiic_Relay
 * Description:
 * Author:
 * Date:
 */

#include "spark-dallas-temperature.h"
#include "OneWire.h"
#include "DS18.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "credentials.h"
#include "math.h"

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

//****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Publish mqtttds = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/TDS");
Adafruit_MQTT_Publish mqttsensorTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/fanTemp");
//Adafruit_MQTT_Subscribe mqttbutton1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonfeed3");
//Adafruit_MQTT_Subscribe mqttbuttonClose = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonfeed4");
/************Declare Variables*************/


DS18 sensor(A5);

//the relays connect to
int IN1 = D3;
int IN2 = D4;
int IN3 = D5;
int IN4 = D6;
int TdsSensorPin = A0;  //identiying TDS Sensor Pin
int waterPump = A4; 
const float VREF = 5.0;      // analog reference voltage(Volt) of the ADC
const int SCOUNT = 30;       //value stored size
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0; 

const bool ON = 0;
const bool OFF = 1;

unsigned long last, last2, last3, lastTime;
float averageVoltage = 0, tdsValue = 0, temperature = 25;
bool timer1_LastState = false;
String DateTime , TimeOnly;


void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);
  Time.zone(-6); //Setting time to MST
  Particle.syncTime(); //Sync time with Particle cloud
  delay(1000);

  WiFi.connect();
    while (WiFi.connecting()){
      Serial.printf(".");
    }

  //Setup MQTT subscription for onoff feed.
  //mqtt.subscribe(&mqttbutton1);
  //mqtt.subscribe(&mqttbuttonClose);

  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);  //TDS initialized
  pinMode(IN1, OUTPUT); //relay output 1 initialized
  pinMode(IN2, OUTPUT); //relay output 2 initialized
  pinMode(IN3, OUTPUT); //relay output 3 initialized
  pinMode(IN4, OUTPUT); //relay output 4 initialized
  pinMode(waterPump, OUTPUT);
  pinMode(A5, OUTPUT);
  digitalWrite(A5, HIGH);
  relay_SetStatus(OFF, OFF, OFF, OFF);
  Serial.printf("%i, %i, %i\n", Time.weekday(), Time.hour(), Time.minute());
}

void loop() {
MQTT_connect();  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
   }

   //************** Setting up each relay on a timer using Particle Sync Time **************//

    if (Time.weekday() == 4 && Time.hour() == 15 && Time.minute() == 05) {
        digitalWrite(IN1, LOW);//turn on RELAY 
        digitalWrite(IN2, LOW);//turn on RELAY
        digitalWrite(IN3, LOW);//turn on RELAY
        digitalWrite(IN4, LOW);//turn on RELAY
        Serial.printf("Drainage Pump is ON, %i, %i, %i\n", Time.weekday(), Time.hour(), Time.minute());
    }
      if (Time.weekday() == 4 && Time.hour() == 15 && Time.minute() == 06){
        digitalWrite(IN1, HIGH);//turn off RELAY
        digitalWrite(IN2, HIGH);//turn off RELAY  
        digitalWrite(IN3, HIGH);//turn off RELAY
        digitalWrite(IN4, HIGH);//turn off RELAY
        digitalWrite(waterPump, HIGH);
        delay (10000);
        digitalWrite(waterPump, LOW);
          Serial.printf("Drainage Pump is OFF\n");
      }

        if (Time.weekday() == 4 && Time.hour() == 15 && Time.minute() == 06){
        digitalWrite(waterPump, HIGH);
        delay (20000);
        digitalWrite(waterPump, LOW);
         Serial.printf("Drainage Pump is OFF\n");
      }

  //************** Reading Total Disolved Solids and pushing to the Dashboard **************//
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 1000)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    Serial.printf("voltage: %0.2f", averageVoltage);
    Serial.printf("V   \n");
    Serial.printf("TDS----Value: %0.1f", tdsValue);
    Serial.printf("ppm\n");
  }

  //************** Read the next available 1-Wire temperature sensor **************//
  if (sensor.read()) {  // read the temperature
    Serial.printf("Temperature %.2f C %.2f F ", sensor.celsius(), sensor.fahrenheit());
    Particle.publish("temperature", String(sensor.celsius()), PRIVATE);
    //printDebugInfo();     // Additional info useful while debugging
  // If sensor.read() didn't return true you can try again later
  // This next block helps debug what's wrong.
  // It's not needed for the sensor to work properly
  } else {
    // Once all sensors have been read you'll get searchDone() == true
    // Next time read() is called the first sensor is read again
    if (sensor.searchDone()) {
      //Serial.println("No more addresses.");
      // Avoid excessive printing when no sensors are connected
      delay(250);   // Something went wrong
    } 
  }

      if ((millis() - last3) > 100000){
        if(mqtt.Update()) { //if mqtt object (Adafruit.io) is available to receive data
        //Serial.printf("Publishing %f to Adafruit.io feed TDS \n",tdsValue);
        //Serial.printf("Publishing %f to Adafruit.io feed fanTemperature \n",sensor.fahrenheit());
        mqtttds.publish(tdsValue);
        mqttsensorTemp.publish(sensor.fahrenheit());
        last3 = millis();
        }
      }

      while (tdsValue > 500){
        digitalWrite(IN1, LOW);//turn off RELAY
        digitalWrite(IN2, LOW);//turn off RELAY  
        digitalWrite(IN3, HIGH);//turn off RELAY
        digitalWrite(IN4, HIGH);//turn off RELAY
      if ((millis() - lastTime) > 600000){
        digitalWrite(IN1, HIGH);//turn off RELAY
        digitalWrite(IN2, HIGH);//turn off RELAY  
        digitalWrite(IN3, LOW);//turn off RELAY
        digitalWrite(IN4, LOW);//turn off RELAY
      }
    } 
}


//set the status of relays
void relay_SetStatus( unsigned char status_1,  unsigned char status_2, unsigned char status_3,unsigned char status_4) {
  digitalWrite(IN1, status_1);
  digitalWrite(IN2, status_2);
  digitalWrite(IN3, status_3);
  digitalWrite(IN4, status_4);
}

int getMedianNum(int bArray[], int iFilterLen) { //tabulates the reading in array and devises by total readings to smooth out variabilities
  int i, j, bTemp;
  int bTab[iFilterLen];  //bTab array identifies the tabulated sum of readings stored
  for (byte i = 0; i < iFilterLen; i++) {//iFilter len is the number of times being sampled
    bTab[i] = bArray[i]; //bArray is the median of array readings of tds
  }
      for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
          if (bTab[i] > bTab[i + 1]) {
          bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
          bTab[i + 1] = bTemp;
          }
      //Serial.printf("bTab is %i, bTemp is %i \n", bTab[i], bTemp);
        }
      }
    if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2]; //subtract 1 due to the number of array starting at 0
    }    
      else {
      bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      }
  return bTemp;
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
  Serial.printf("sensor type is %c \n", type);

  // Print the ROM (sensor type and unique ID)
  uint8_t addr[8];
  sensor.addr(addr);
  Serial.printf(" ROM=%02X%02X%02X%02X%02X%02X%02X%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7] );

  // Print the raw sensor data
  uint8_t data[9];
  sensor.data(data);
  Serial.printf(" data=%02X%02X%02X%02X%02X%02X%02X%02X%02X",data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect(){
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");
}

