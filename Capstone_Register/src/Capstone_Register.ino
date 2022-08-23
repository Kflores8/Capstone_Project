/*
 * Project Captstone_CheckBME
 * Description: Subscribing to Adafruit dashboard and ensuring motor turns for opening 
 * and closing of vent with air quality conditions and buttons on the dashboard.
 * Author: Kevin Flores 
 * Date: 22 Aug 2022
 */

#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "credentials.h"
#include "math.h"
#include "Stepper.h"

/************mapping my Stepper motor ***************/
Stepper myStepper(2048, D6, D4, D5, D3);
/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
//****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe mqttbutton1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonfeed");
Adafruit_MQTT_Subscribe mqttbuttonClose = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonfeed2");
/************Declare Variables*************/
unsigned long last, last2, last3, lastTime, timer2;
byte count, i; //8-bit integer that goes from 0 to 255

String DateTime , TimeOnly;

unsigned long duration;
unsigned long starttime; 
unsigned long timems = 60000;

bool status, buttonOnOff, buttonOnOff2, openState; 


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
  myStepper.setSpeed(10);
  delay(1000);
  starttime = millis();

  Serial.printf("Waiting sensor to init...");
  delay(20000);

  //Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&mqttbutton1);
  mqtt.subscribe(&mqttbuttonClose);
}

void loop(){
  MQTT_connect();
  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11, 19);
  //Serial.printf("Date and time is %s\n", DateTime.c_str());
  //Serial.printf("Time is %s\n", TimeOnly.c_str());
  //display.clearDisplay();

  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  

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
  }
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
