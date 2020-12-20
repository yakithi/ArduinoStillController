#include <FloatDefine.h>
#include <Filters.h>
#include <FilterTwoPole.h>
#include <RunningStatistics.h>
#include <FilterOnePole.h>
#include <FilterDerivative.h>


//#include <Filters.h>

#include <ZMPT101B.h>
 
#include <Adafruit_MAX31865.h>

#define RREF 430.0
#define RNOMINAL 100.0
#define MAINS_VOLTAGE 240
#define SENSOR  A0 //SENSOR analog input, here it's A0
#define COL_PIN 3
#define VAP_PIN 4
#define WASH_PIN 5

#define SWITCH_PIN 7

Adafruit_MAX31865 col_sensor = Adafruit_MAX31865(COL_PIN,11,12,13);
Adafruit_MAX31865 vap_sensor = Adafruit_MAX31865(VAP_PIN,11,12,13);
Adafruit_MAX31865 wash_sensor = Adafruit_MAX31865(WASH_PIN,11,12,13);

int col_temp;
int vap_temp;
int wash_temp;

int voltage;
//bool first_voltage_read = true;

bool powerFlag;
bool comsOk;
long powerTimeout = 5000; // if we havent heard from controller for 5 sec, turn off element

char start_char = '<';
char end_char = '>';
char seperator = ',';

float vap_read;
float col_read;
float wash_read;

long volt_read;


long lastOutComTime;
long delayTime = 1000;

bool  receiving_started = false;
String incoming_msg = "";
long lastInComTime;
long inTimeout = 5000;


float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist


float current_Volts; // Voltage


//RunningStatistics inputStats; // keeps track of readings
//FilterOnePole inputVoltage;

 
int RawValue = 0;
float Volts_TRMS;
float intercept = -.04;
float slope = .0405;
unsigned long printPeriod =1000;
unsigned long previousMillis = 0;
int sampleCount =0;
unsigned long accumulatedTotal = 0;

String make_com_string(){
  vap_temp = vap_read * 100;
  col_temp = col_read * 100;
  wash_temp = wash_read * 100;
  
//  Volts_TRMS = inputStats.sigma() * slope + intercept;
//  Volts_TRMS  = Volts_TRMS*(40.3231);
//  voltage = Volts_TRMS*100;
  String msg = start_char + String(vap_temp) + "," + String(col_temp) + "," + String(wash_temp) 
    + "," + String(voltage) + end_char;
  return msg;    
}

void temp_values(){
  vap_read = vap_sensor.temperature(RNOMINAL, RREF);
  col_read = col_sensor.temperature(RNOMINAL, RREF);
  wash_read = wash_sensor.temperature(RNOMINAL, RREF);
}

bool get_comms(){
  if (lastInComTime == 0){
     lastInComTime = millis();
  }
  if (Serial.available() > 0){
    char c = Serial.read();
    // Check to see if we have started receiving yet
      if (receiving_started){
        if (c == '>') { // end of comms
          receiving_started = false;
          lastInComTime = millis();
          return true; // finished receiving message
        }
        else { // add this char to the incoming message
          incoming_msg.concat(c);
        }
      }
      else{ // see if its a new message
        if ( c == '<'){ // its the start of a new message
        incoming_msg = "";
        receiving_started = true;
        } 
      }
  }
  return false;
}

void element(bool power){
    if (power){
      digitalWrite(SWITCH_PIN, LOW);
    }
    else{
      digitalWrite(SWITCH_PIN, HIGH);
    }
}


void ReadVoltage(){
  RawValue = analogRead(SENSOR);
  //inputStats.input(RawValue);
  //inputVoltage.input(RawValue);

  if ((unsigned long) (millis() - previousMillis) >= printPeriod){
    //Volts_TRMS = inputStats.sigma() * slope + intercept;
 
  }
  
}
 
bool handShake(){
  
  Serial.write('#');
  delay(1000);
  if (Serial.available() > 0){
    int r = Serial.read();
    if(r == 48){
      return true;
    }
  }
  return false;
}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    comsOk = false;

    pinMode(SWITCH_PIN, OUTPUT);
    digitalWrite(SWITCH_PIN, HIGH);
    
    col_sensor.begin(MAX31865_3WIRE);
    vap_sensor.begin(MAX31865_3WIRE);
    wash_sensor.begin(MAX31865_3WIRE);
    
 //   inputStats.setWindowSecs( windowLength );
//    inputVoltage.setWindowSecs( 1);
    lastOutComTime = millis();
    lastInComTime = 0;

    while (true){
      if (handShake()){
        comsOk = true;
        break;
      }
    }
}

void loop() {
  RunningStatistics inputStats; // keeps track of readings
  inputStats.setWindowSecs( windowLength );
  //inputStats.setWindowSecs( 1 );

    while(true){
      //ReadVoltage();
      

      RawValue = analogRead(SENSOR);
      if(RawValue < 0){RawValue = 0;}
      inputStats.input(RawValue);
      
      if ((millis()- lastOutComTime) > delayTime ){
        Volts_TRMS = inputStats.sigma() * slope + intercept;
        Volts_TRMS  = Volts_TRMS*(40.3231);
        if (Volts_TRMS > 240){Volts_TRMS = 240;}
        if (Volts_TRMS < 2.5 ){Volts_TRMS = 0;}
        voltage = Volts_TRMS*100;
        temp_values();
        String msg = make_com_string();
        Serial.print(msg);
       
        lastOutComTime = millis();
      }
    
      if ((millis() - lastInComTime ) > inTimeout){
        element(false);
      }
      if ( get_comms()){
        if (incoming_msg == "1"){
          element(true);
        }else if(incoming_msg == "0"){
          element(false);
        }
     }
   }
}
