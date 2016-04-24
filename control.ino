#include <Event.h>
#include <DHT22.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Timer.h>
#include <stdio.h>

//define pins
const int heaterPin[8] = {2,3,4,5,6,7,8,9};
const int fanPin = 10;
const int RHPin = A5;
const int RHFanPin = A4;

//DS18B20 for heater temperature warning pin at A2(15)
OneWire  heaterTempPin(A2);
DallasTemperature heaterTemp(&heaterTempPin);

//DHT22 for beehive temperature warning at pin A0(13)
DHT22 beehiveTemp(A0);
//DHT22 for control box temperature warning at pin A1(14)
DHT22 controlTemp(A1);

//timer for update sensors
Timer sensorClock;

//for saving sensor value [heater temperature, ambient temperature, ambient humidity, beehive temperature, beehive humidity]
float sensorVal[5] = {0};

//for setup heater pattern
char heaterPattern[9] = {0b11111111 , 0b11111101 , 0b11011101 , 0b11011001 , 0b10011001 , 0b10011000 , 0b10001000 , 0b10000000 , 0b00000000};

//for contorl parameters
float errorSum = 0;
float outVal = 0;
float errorMemory = 20;
float error = 0;

float targetTemp = 34.5;
float targetRH = 40.0;
float kp = 0.6;
float kpa = 0.50;
float ki = 0.08;

void heaterOutput(const int& val) {
  int outVal = val;
  if(val > 8)
    outVal = 8;
  else if(val < 0)
    outVal = 0;
  
  for(int i = 0; i < 8; i++)
  {
    digitalWrite(heaterPin[i],heaterPattern[outVal]&(1<<i));
  }
}

void getAllSensor() {
  //get heater temperature
  heaterTemp.requestTemperatures();
  sensorVal[0] = heaterTemp.getTempCByIndex(0); 

  //get control box temperature and RH
  controlTemp.readData();
  sensorVal[1] = controlTemp.getTemperatureC();
  sensorVal[2] = controlTemp.getHumidity();

  //get control box temperature and RH
  beehiveTemp.readData();
  sensorVal[3] = beehiveTemp.getTemperatureC();
  sensorVal[4] = beehiveTemp.getHumidity();

  errorSum = errorSum*(errorMemory-1)/errorMemory+error;

  //print out value
  Serial.print(sensorVal[0]);
  Serial.print(",");
  Serial.print(sensorVal[1]);
  Serial.print(",");
  Serial.print(sensorVal[2]);
  Serial.print(",");
  Serial.print(sensorVal[3]);
  Serial.print(",");
  Serial.print(sensorVal[4]);
  Serial.print(",");
  Serial.print(outVal);
  Serial.print(",");
  Serial.print(errorSum);
  Serial.println();
}

void setup() {
  //COM port initialize
  Serial.begin(9600);
  Serial.print("System initializing...\n");
  
	//setup heater pin mode
	for(int i = 0; i < 8; i++)
	{
		pinMode(heaterPin[i], OUTPUT);
	}

  //set up RH mode
  pinMode(RHPin, OUTPUT);
  pinMode(RHFanPin, OUTPUT);
  //setup fan pin mode
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin,LOW);
  
  //for heater initialize
  heaterOutput(0);

  //sensors intialize
  heaterTemp.begin();
  Serial.print("Sensor initializing...\n");
  delay(2000);
  getAllSensor();

  //set updating sensor time
  sensorClock.every(2000,getAllSensor);
}


void loop() {
  static boolean overload = 0;
  if(sensorVal[0] > 99)
    overload = 1;
  else if(sensorVal[0] < 90)
    overload = 0; 

  if(sensorVal[4] < targetRH)
  {
    digitalWrite(RHPin,HIGH);
    analogWrite(RHFanPin,255);
  }
  else
  {
    digitalWrite(RHPin,LOW);
    analogWrite(RHFanPin,0);
  }
    
  if(overload)
  {
    heaterOutput(0);
    analogWrite(fanPin,255);
  }
  else
  {
    float nowTemp = sensorVal[3];
    float ambientTemp = sensorVal[1];
  
    error = targetTemp-nowTemp;
  
    analogWrite(fanPin,128);
    outVal = (error)*kp+(targetTemp-ambientTemp)*kpa+errorSum*ki;
    heaterOutput((int)round(outVal));
  }
  sensorClock.update();
}
