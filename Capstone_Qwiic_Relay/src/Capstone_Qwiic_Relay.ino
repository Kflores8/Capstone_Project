/*
 * Project Capstone_Qwiic_Relay
 * Description:
 * Author:
 * Date:
 */

#include "DailyTimerSpark.h"

//the relays connect to
int IN1 = D3;
int IN2 = D4;
int IN3 = D5;
int IN4 = D6;
const bool ON = 0;
const bool OFF = 1;
bool timer1_LastState = false;

DailyTimer timer1( 8, 30, 9, 00, WEEKENDS);  // creates with a default fixed start time and end time
uint32_t lastUpdateTime = 0;

void setup() {
  timer1.begin();
  Serial.begin(9600);
  pinMode(D7, OUTPUT);
  //timer1.setDaysActive(WEEKDAYS);    
  relay_init();//initialize the relay
}

void loop() {
  bool timerState = timer1.isActive();  //State Change method this block
    if(timerState != timer1_LastState) {
      if(timerState){
      digitalWrite(IN1, HIGH);//turn on RELAY_1 
      Serial.printf("Drainage Pump is ON");
      }
      else{
      digitalWrite(IN1, LOW);//turn on RELAY_1 
      Serial.println("Drainage Pump is OFF");
      }
    timer1_LastState = timerState;
    }
}

  // relay_SetStatus(ON, OFF, OFF,OFF);//turn on RELAY_1 
  // delay(2000);//delay 2s
  // relay_SetStatus(OFF, ON, OFF,OFF);//turn on RELAY_2
  // delay(2000);//delay 2s
  // relay_SetStatus(OFF, OFF, ON,OFF);//turn on RELAY_3
  // delay(2000);//delay 2s
  // relay_SetStatus(OFF, OFF, OFF,ON);//turn on RELAY_3
  // delay(2000);//delay 2s
//}


void relay_init(void)//initialize the relay
{  
  //set all the relays OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  relay_SetStatus(OFF,OFF,OFF,OFF);//turn off all the relay
}
//set the status of relays
void relay_SetStatus( unsigned char status_1,  unsigned char status_2, unsigned char status_3,unsigned char status_4)
{
  digitalWrite(IN1, status_1);
  digitalWrite(IN2, status_2);
  digitalWrite(IN3, status_3);
  digitalWrite(IN4, status_4);

}