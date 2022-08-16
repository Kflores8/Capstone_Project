/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/Brosana/Documents/IoT/Capstone_Project/Captstone_CheckBME/src/Captstone_CheckBME.ino"
/*
 * Project Captstone_CheckBME
 * Description: Adding Air Quality and BME for inside of the house, publishing to Adafruit dashboard, and 
 * ensuring motor turns for Opening and closing of vent.
 * Author: Kevin Flores 
 * Date: 15 Aug 2022
 */


#include "Adafruit_SSD1306.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "credentials.h"
#include "Adafruit_BME280.h"
#include "Grove_Air_quality_Sensor.h"
#include "math.h"
#include "Stepper.h"

/************mapping my Stepper motor ***************/
void setup();
void loop();
void readingUpdate(void);
void MQTT_connect();
void checkBME (void);
void airQuality();
#line 21 "/Users/Brosana/Documents/IoT/Capstone_Project/Captstone_CheckBME/src/Captstone_CheckBME.ino"
Stepper myStepper(2048, D6, D4, D5, D3);

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

//****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Publish mqtttemperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish mqtthumidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish mqttgetDust = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/concentration");
Adafruit_MQTT_Publish mqttairQuality = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airValue");
Adafruit_MQTT_Subscribe mqttbutton1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonfeed");
Adafruit_MQTT_Subscribe mqttbuttonClose = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonfeed2");
/************Declare Variables*************/
unsigned long last, last2, last3, lastTime;
SYSTEM_MODE(SEMI_AUTOMATIC);
byte count, i; //8-bit integer that goes from 0 to 255

int airPin = A0;
int current_quality =-1;
int qualityValue;
int airValue; 

const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int OLED_RESET = D4;
const int SCREEN_ADDRESS = 0x3C;

String DateTime , TimeOnly;

unsigned long duration;
unsigned long starttime; 
unsigned long timems = 60000;

float tempC, pressPA, humidRH, tempF, inHG, currentTempF, lastTemp, lastHG;
bool status, buttonOnOff, buttonOnOff2, openState; 

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;
AirQualitySensor sensor(A0);

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);
  delay(1000);

  WiFi.connect();
    while (WiFi.connecting()){
      Serial.printf(".");
    }
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  Time.zone(-6);
  sensor.init();
  Particle.syncTime();
  myStepper.setSpeed(10);
  delay(1000);
  pinMode(airPin, INPUT);
  starttime = millis();

  Serial.printf("Waiting sensor to init...");
  delay(20000);
  
    if (sensor.init()) {
      Serial.printf("Sensor ready.");
    }
    else {
      Serial.printf("Sensor ERROR!");
    }

  status = bme.begin(0x76);

  Wire.setSpeed(400000);
    if (!Wire.isEnabled())
    {
      Wire.begin();
    }

    if (status == false) {
      Serial.printf("BME280 at address 0x%02X failed to start \n", 0x76);
    }
  //Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&mqttbutton1);
  mqtt.subscribe(&mqttbuttonClose);
}

void loop(){
  MQTT_connect();
  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11, 19);
  checkBME();
  airQuality();
  Serial.printf("Date and time is %s\n", DateTime.c_str());
  Serial.printf("Time is %s\n", TimeOnly.c_str());
  
  readingUpdate();
  display.clearDisplay();

  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
   }
   if ((millis()-last2)>60000){
    if (humidRH > 80 && openState == true){
    openState = false; 
    myStepper.step(2048);
    last2 = millis();
    }
   }
   else{
    if (humidRH < 80 && openState == false); {
      openState = true; 
      myStepper.step(-2048);
      }
   }


  Adafruit_MQTT_Subscribe *subscription; //via Adafruit Dashboard, manually open floor register.
  while ((subscription = mqtt.readSubscription(1000))) {
  if (subscription == &mqttbutton1) {
  buttonOnOff = atoi((char *)mqttbutton1.lastread);
      if (buttonOnOff = 1) { 
      myStepper.step(2048);
      }
      Serial.printf("Recieved %i from Adafruit.io feed ButtonFeed \n", buttonOnOff);
  } 
    if (subscription == &mqttbuttonClose) {
  buttonOnOff2 = atoi((char *)mqttbuttonClose.lastread);
      if (buttonOnOff2 = 1) { 
      myStepper.step(-2048);
      }
      Serial.printf("Recieved %i from Adafruit.io feed ButtonFeed2 \n", buttonOnOff2);
  } 
}

    if ((millis() - last3) > 100000){
      if(mqtt.Update()) { //if mqtt object (Adafruit.io) is available to receive data
      Serial.printf("Publishing %f to Adafruit.io feed temperature \n",tempF);
      Serial.printf("Publishing %f to Adafruit.io feed humidity \n",humidRH);
      Serial.printf("Sensor value: %i, \n Quality value: %i to Adafruit.io feed Air Quality\n", qualityValue);
      mqtttemperature.publish(tempF);
      mqtthumidity.publish(humidRH);
      mqttairQuality.publish(qualityValue);
      last3 = millis();
    }
  }
}


void readingUpdate(void){
  display.clearDisplay();
  display.setTextSize(1);  //Draw to scale text
  display.setTextColor(WHITE);  
  display.setCursor(10, 0);
  display.printf("Time is %s", TimeOnly.c_str());
  display.display();

  display.setTextSize(1);  //Draw to scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 10);
  display.printf("Temp is %f", tempF);
  display.display();
  delay(5000);

  display.setTextSize(1);  //Draw to scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 20);
  display.printf("Humidity is %f", humidRH);
  display.display();
  
  display.setTextSize(1);  //Draw to scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 30);
  display.printf("Air Quality is %f", qualityValue);
  display.display();
}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {
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

void checkBME (void) {
  tempC = bme.readTemperature();
  pressPA = bme.readPressure();
  humidRH = bme.readHumidity();
  tempF = tempC*9/5+32;
  inHG = pressPA/3386; 
  Serial.printf("%f, %f, %f \n", tempF, inHG, humidRH);
}


void airQuality() {
  qualityValue = sensor.slope();
  airValue = sensor.getValue(); 
  Serial.printf("Sensor value: %i, \n Quality value: %i \n", airValue, qualityValue);
  
  if (qualityValue == AirQualitySensor::FORCE_SIGNAL) {
    Serial.println("High pollution! Force signal active.");
  }
  else if (qualityValue == AirQualitySensor::HIGH_POLLUTION) {
    Serial.println("High pollution!");
  }
  else if (qualityValue == AirQualitySensor::LOW_POLLUTION) {
    Serial.println("Low pollution!");
  }
  else if (qualityValue == AirQualitySensor::FRESH_AIR) {
    Serial.println("Fresh air.");
  }
}