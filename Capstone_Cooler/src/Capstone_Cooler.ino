/*
 * Project Capstone_Qwiic_Relay
 * Description:
 * Author:
 * Date:
 */


//the relays connect to
int IN1 = D3;
int IN2 = D4;
int IN3 = D5;
int IN4 = D6;
const bool ON = 0;
const bool OFF = 1;
bool timer1_LastState = false;
String DateTime , TimeOnly;

void setup() {
  Serial.begin(9600);
  Time.zone(-6); //Setting time to MST
  Particle.syncTime(); //Sync time with Particle cloud
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.printf("%i, %i, %i\n", Time.weekday(), Time.hour(), Time.minute());
}

void loop() {
  DateTime = Time . timeStr ();
  TimeOnly = DateTime . substring (11 ,19) ; // Extract the Time from the DateTime String

    if(Time.weekday() == 3) {
      if(Time.hour() == 13){
        if (Time.minute() == 00){
        digitalWrite(IN1, HIGH);//turn on RELAY_1 
        digitalWrite(IN2, HIGH);//turn on RELAY_1 
        digitalWrite(IN3, HIGH);//turn on RELAY_1 
        digitalWrite(IN4, HIGH);//turn on RELAY_1 
        Serial.printf("Drainage Pump is ON, %i, %i, %i\n", Time.day(), Time.hour(), Time.minute());
    }
      }
        }
    else{
      if (Time.day() == 3){
        if (Time.hour() == 13){
          if (Time.minute() == 15){
        digitalWrite(IN1, LOW);//turn on RELAY_1 
        digitalWrite(IN2, LOW);//turn on RELAY_1 
        digitalWrite(IN3, LOW);//turn on RELAY_1 
        digitalWrite(IN4, LOW);//turn on RELAY_1 
          Serial.printf("Drainage Pump is OFF");
      }
        }
          }
  }
}


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