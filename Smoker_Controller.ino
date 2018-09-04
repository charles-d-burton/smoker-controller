#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include "credentials.h"

// So we can save and retrieve settings
#include <EEPROM.h>

// IO Pins for thermocouple
#define DO   3
#define CS   4
#define CLK  5

#define KP "kp"
#define SP "sp"
#define KI "ki"
#define KD "kd"
// Output Relay
#define RelayPin 7

// ************************************************
// WIFI and MQTT
// ************************************************
WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWD;
const char* smoker_id = SMOKER;
const char* mqtt_server = "mosquitto.localdomain";
const char* topic = "/homeassistant/devices/smoker";
long lastMsg = 0;

// ************************************************
// PID Variables and constants
// ************************************************
 
//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;
 
volatile long onTime = 0;
 
// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

Adafruit_MAX31855 thermocouple(CLK, CS, DO);
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;
 
float aTuneStep=500;
float aTuneNoise=1;
unsigned int aTuneLookBack=20;
boolean tuning = false;
 
PID_ATune aTune(&Input, &Output);

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = OFF;

String setTemp = "";


// Setup and connect to the wifi
void setup_wifi() {
  delay(100);
  Serial.print("Connecting to: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //randomSeed(micros());
  Serial.println("");
  Serial.println("Wifi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Gateway: ");
  Serial.println(WiFi.gatewayIP());
}

//Reconnect to the MQTT broker also check WiFi state
void reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
      setup_wifi();
  }                   
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "Smoker-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      sendBoot();
      // ... and resubscribe
      client.subscribe("/homeassistant/devices/smoker/receive");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//Process messages incoming from the broker
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  //TODO: Decode incoming message
  /*char tempSetBuf[8];
   if (Bridge.get("setTempF", tempSetBuf, 8) > 0) {
     setTemp = String(tempSetBuf);
     Setpoint = stringToDouble(setTemp);
   }
   if (Bridge.get("setRunState", tempSetBuf, 8) > 0) {
     String setState = String(tempSetBuf);
     if (setState.equals("on")){
       //turn the PID on
       myPID.SetMode(AUTOMATIC);
       windowStartTime = millis();
       opState = RUN;
     }  else if (setState.equals("off")) {
       myPID.SetMode(MANUAL);
       digitalWrite(RelayPin, LOW);
       opState = OFF;
     }
   } 
   if (Bridge.get("setStateTune", tempSetBuf, 8) > 0) {
     String setTune = String(tempSetBuf);
     if (setTune.equals("on")) {
       opState = SETP;
       StartAutoTune();
     }     
   }*/
}

//Send message announcing boot
void sendBoot() {
  StaticJsonBuffer<50> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["device_id"] = smoker_id;
  root["run_state"] = opState;
  
  char json_message[50];
  root.printTo(json_message);
  client.publish(topic,json_message);
  espClient.flush();
}

//Send the message out
void sendTemp(float c, float f) {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["device_id"] = smoker_id;
  root["run_state"] = opState;
  root["temp_cel"] = c;
  root["temp_far"] = f;
  
  char json_message[200];
  root.printTo(json_message);
  client.publish(topic,json_message);
  espClient.flush();
}

//Send message denoting an error
void sendError() {
  StaticJsonBuffer<50> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["device_id"] = smoker_id;
  root["error"] = true;
  char json_message[50];
  root.printTo(json_message);
  client.publish(topic,json_message);
  espClient.flush();
}

//Setup the WIFI, MQTT, and PID
void setup() {
  Serial.begin(115200);
  setup_wifi();
  
  //timeClient.begin();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  // Initialize Relay Control:
  //pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
  //digitalWrite(RelayPin, LOW);  // make sure it is off to start
  
  // Initialize the PID and related variables
  EEPROM.begin(512);
  LoadParameters();
  // wait for MAX chip to stabilize
  delay(2000);
  
  // Initialize the PID and related variables
  myPID.SetTunings(Kp,Ki,Kd);
 
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);
}

//Timer interrupt handler
void TimerInterrupt() {
  if (opState == OFF) {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  } else {
    DriveOutput();
  }
  //Console.println("Interrupt");
}

//Main loop
void loop() {
   Serial.println("Entering main loop");
   // digitalWrite(ledPin, LOW);
   if (!client.connected()) {
     reconnect();
   }
   client.loop();
   switch (opState) {
     case OFF:
       Serial.println("OFF");
       break;
     case SETP:
       Serial.println("LEARN");
       break;
     case RUN:
       Serial.println("ON");
       DoControl();
       break;
     case TUNE_P:
       break;
     case TUNE_I:
       break;
     case TUNE_D:
       break;  
   }   
   
   //Make sure the bridge always has the current temperature.
   //double f = thermocouple.readFarenheit();
   //double c = thermocouple.readCelsius();
   //if (isnan(f)||isnan(c)) {
   sendError();
   //}
   
   
   delay(15000);
   
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl() {
   float f = thermocouple.readFarenheit();
   float c = thermocouple.readCelsius();
   if (isnan(f)||isnan(c)) {
     Serial.println("Something wrong with thermocouple!");
     sendError();
   } else {
      Input = f;
      Serial.println(Input);
     //We're getting good data.  Make is so the bridge can see it
     /*double tempF = thermocouple.readFarenheit();
     */
    if (tuning) { // run the auto-tuner
       if (aTune.Runtime()){ // returns 'true' when done
         FinishAutoTune();
       }
    } else { // Execute control algorithm
       myPID.Compute();
       Serial.println("PID COMPUTED");
       Serial.println(Output);
       Serial.println("");
     }
  
    // Time Proportional relay state is updated regularly via timer interrupt.
    onTime = Output;  
  } 
  sendTemp(c,f);
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput() {  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize) { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime))) {
     digitalWrite(RelayPin,HIGH);
  } else {
     digitalWrite(RelayPin,LOW);
  }
  
}


// ************************************************
// Start the Auto-Tuning cycle
// ************************************************
 
void StartAutoTune() {
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();
 
   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;
 
   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();
 
   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}
// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters() {
   float setpoint, kp, ki, kd;
   
   if (Setpoint != EEPROM.get(SpAddress, setpoint)) {
      EEPROM.put(SpAddress, Setpoint);
   }
   if (Kp != EEPROM.get(KpAddress, kp)) {
      EEPROM.put(KpAddress, Kp);
   }
   if (Ki != EEPROM.get(KiAddress, ki)) {
      EEPROM.put(KiAddress, Ki);
   }
   if (Kd != EEPROM.get(KdAddress, kd)) {
      EEPROM.put(KdAddress, Kd);
   }
   EEPROM.commit();
}
 
// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters() {
   Serial.println("Reading in EEPROM vars");
  // Load from EEPROM
   EEPROM.get(SpAddress, Setpoint);
   EEPROM.get(KpAddress, Kp);
   EEPROM.get(KiAddress, Ki);
   EEPROM.get(KdAddress, Kd);
   EEPROM.commit();
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint)) {
     Setpoint = 60;
   }
   if (isnan(Kp)) {
     Kp = 500;
   }
   if (isnan(Ki)) {
     Ki = 0.5;
   }
   if (isnan(Kd)) {
     Kd = 0.1;
   }  
}
 
 
// ************************************************
// Write floating point values to EEPROM
// ************************************************
/*void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
   EEPROM.commit();
}*/
 
// ************************************************
// Read floating point values from EEPROM
// ************************************************
/*double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   EEPROM.commit();
   return value;
}*/

//Helpers for String to double conversions
String doubleToString(double input,int decimalPlaces){
  if(decimalPlaces!=0){
    String string = String((int)(input*pow(10,decimalPlaces)));
    if(abs(input)<1){
      if(input>0)
        string = "0"+string;
      else if(input<0)
        string = string.substring(0,1)+"0"+string.substring(1);
    }
    return string.substring(0,string.length()-decimalPlaces)+"."+string.substring(string.length()-decimalPlaces);
    } else {
      return String((int)input);
  }
}

double stringToDouble(String input) {
    char floatbuf[8];
    input.toCharArray(floatbuf, sizeof(floatbuf));
    return atof(floatbuf);
}
