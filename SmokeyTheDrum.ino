//Greg
#include <ESP8266WiFi.h> 
#include <Adafruit_MAX31865.h>
#include <ESP8266mDNS.h> //OTA
#include <WiFiUdp.h> //OTA
#include <ArduinoOTA.h> //OTA
#include <SimpleTimer.h>
#include <BlynkSimpleEsp8266.h>
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <Adafruit_MAX31865.h>
#include <SPI.h>
#include <PID_v1.h>

//initialize for thingspeak and wifi
SimpleTimer timer;  //Starts timer to run the thingspeak update every x milliseconds
char ssid[] = "Hesp-Davies";
char password[] = "bahamas1701";
WiFiClient client;

//initialize for Blynk
char auth[] = "GP6CQ0exEMHnUw3L-sDvE1wGJXI4dpiX";

// initialise MAX3165 (CS, SDI, SDO, CLK)
Adafruit_MAX31865 foodProbe = Adafruit_MAX31865(D4,D1,D2,D3);
Adafruit_MAX31865 ambientProbe = Adafruit_MAX31865(D5,D1,D2,D3);

// Declare Variables
int targetTemp = 125; // Target Ambient Temperature
int tempBelowTarget = 30;
int pullTemp = 95; // Meat Temperature to remove food
float foodTemperature = 25;
float ambientTemperature = 25;
float foodTemperature_Last = 25;
float ambientTemperature_Last = 25;
bool StillCooking = true;
float tempWeightedAvg = 25;
float tempWeightedAvgLast= 25;
int fanSpeed = 400;
int minSpeed = 15;

//Define Variables we'll be connecting to (PID)
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters (PID)
double aggKp=8, aggKi=0.2, aggKd=1;
double consKp=2, consKi=0.1, consKd=.5;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

BLYNK_WRITE(V0) {   //pulls data from Blynk app for target temp
  int targetTempBlynk = param.asInt();
  targetTemp = targetTempBlynk;
  Serial.printf("New target temperature = %d\n",targetTemp);
}
BLYNK_WRITE(V1) { //pulls data from Blynk app for pull temp
  int pullTempBlynk = param.asInt();
  pullTemp = pullTempBlynk;
  Serial.printf("New pull temperature = %d\n",pullTemp);
}
BLYNK_READ(V2){ //sends Blynk app the data for the fan speed
  Blynk.virtualWrite(V2, (int)(fanSpeed/10));
}
BLYNK_READ(V3){
  Blynk.virtualWrite(V3, tempBelowTarget);
}
BLYNK_READ(V4){ //sends Blynk app the data for the three sensors
  Blynk.virtualWrite(V4, tempWeightedAvgLast);
}
BLYNK_READ(V5){ //sends Blynk app the data for the food probe
  Blynk.virtualWrite(V5, foodTemperature_Last);
}


void setup() {                
  Serial.begin(9600);
  foodProbe.begin(MAX31865_3WIRE);
  ambientProbe.begin(MAX31865_3WIRE);
  pinMode(LED_BUILTIN, OUTPUT);

  delay(10);
  connectToWifi();

  //Setup PID
  Input = tempWeightedAvgLast;
  Setpoint = targetTemp;
  myPID.SetMode(AUTOMATIC);
  
  //start OTA
  ArduinoOTA.begin(); 

  //Start timers 
  timer.setInterval(5000, readTempSensors);
  timer.setInterval(1000, fanController);
}
  
void loop() {
  if(StillCooking) {
    timer.run();
    Blynk.run();
    ArduinoOTA.handle(); //OTA
    delay(500); 
  } else {
    delay(5000);
    // No idea what this does yet
    if(pullTemp == 130) {
      StillCooking = true;
    }
    timer.run();
    Blynk.run();
    ArduinoOTA.handle(); //OTA
  }
}

void readTempSensors(){
  foodTemperature = foodProbe.temperature(100.0, 430.0);
  delay(100);
  ambientTemperature = ambientProbe.temperature(100.0, 430.0);

  tempWeightedAvg = (ambientTemperature - 45); //-45 to compensate for center drum temperature difference
  foodTemperature_Last = (2 * foodTemperature_Last + foodTemperature)/3;
  ambientTemperature_Last = (2 * ambientTemperature_Last + ambientTemperature)/3;
  tempWeightedAvgLast = (2 * tempWeightedAvgLast + tempWeightedAvg)/3;
  tempBelowTarget = pullTemp - foodTemperature_Last;

  Serial.printf("pull\t|target\t|food\t|ambient\n%d\t|%d\t|%d\t%d\n\n",(int)pullTemp,(int)targetTemp,(int)foodTemperature,(int)ambientTemperature);
}

void connectToWifi() {
  Serial.printf("\nConnecting to %s\n",ssid);
  Blynk.begin(auth, ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");   
  }
  Serial.printf("\n\nWiFi connected\n\n");
}

void fanController() {
  
  // Food Temp is more than 1 degree below target
  if (tempBelowTarget > 1) { 
    Input = tempWeightedAvgLast;
    Setpoint = targetTemp;
    double gap = Setpoint - Input; // average temp distance away from target temperature
    Serial.println(gap);

    if(gap <= 0) {
      fanSpeed = minSpeed;
      //analogWrite(fanControlpin, fanSpeed);
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("Turn Fan Off. [Close to target temperature]");
    }
    else if(gap > 0 && gap <= 10) {
      myPID.SetTunings(consKp, consKi, consKd);
      myPID.Compute();
      fanSpeed = map(Output, 0, 255, minSpeed, 1000);

      if(fanSpeed > 1000) {
        fanSpeed = 1000;
      }

      //analogWrite(fanControlpin, fanSpeed);
      digitalWrite(LED_BUILTIN, LOW);
      Serial.printf("** Turned fan on %d%. \n",(int)(fanSpeed/10));
    } else {
      //Still a long way to go, use agressive tuning
      myPID.SetTunings(aggKd, aggKi, aggKd);
      myPID.Compute();
      fanSpeed = map(Output, 0, 255, minSpeed, 1000);
      if(fanSpeed > 1000) {
        fanSpeed = 1000;
      }
      //analogWrite(fanControlpin, fanSpeed);
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("** Turned fan on");

    }
  } else
  {
    fanSpeed = minSpeed;
    //analogWrite(fanControlpin, fanSpeed);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Turned fan off.");
    StillCooking = false;
    Serial.println("#### Finished! Time to pull");
  }
}