

/*************************************************** 
  This is an example for the Adafruit Thermocouple Sensor w/MAX31855K

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/products/269

  These displays use SPI to communicate, 3 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <Bridge.h>
#include <Console.h>
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <TimerOne.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// IO Pins for thermocouple
#define DO   3
#define CS   4
#define CLK  5
//Keys for the Bridge interface
#define DEGF "degF"
#define DEGC "degC"
#define TARGEF "targetF"

#define KP "kp"
#define SP "sp"
#define KI "ki"
#define KD "kd"
// Output Relay
#define RelayPin 7

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
 
double aTuneStep=500;
double aTuneNoise=1;
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

void setup() {
  Bridge.begin();
  Console.begin();
  // Initialize Relay Control:
  pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(RelayPin, LOW);  // make sure it is off to start
  
  // Initialize the PID and related variables
  LoadParameters();
  // wait for MAX chip to stabilize
  delay(2000);
  
  // Initialize the PID and related variables
  myPID.SetTunings(Kp,Ki,Kd);
 
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);
  
  //Initialize the interrupt timer
  Timer1.initialize(15000);
  Timer1.attachInterrupt(TimerInterrupt);
}

//Timer interrupt handler
void TimerInterrupt() {
  if (opState == OFF) {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  } else {
    DriveOutput();
  }
}

void loop() {
  char tempSetBuf[8];
   if (Bridge.get("setTemp", tempSetBuf, 8) > 0) {
     setTemp = String(tempSetBuf);
     Setpoint = stringToDouble(setTemp);
     Console.println(Setpoint);
     //opState = RUN;
   }
   if (Bridge.get("state", tempSetBuf, 8) > 0) {
     String setState = String(tempSetBuf);
     Console.println(setState);
     if (setState.equals("on")){
       //turn the PID on
       myPID.SetMode(AUTOMATIC);
       windowStartTime = millis();
       opState = RUN; 
     }  else {
       myPID.SetMode(MANUAL);
       digitalWrite(RelayPin, LOW);
       opState = OFF;
     }
   } 
   if (Bridge.get("tune", tempSetBuf, 8) > 0) {
     String setTune = String(tempSetBuf);
     Console.println(setTune);
     if (setTune.equals("on")) {
       opState = SETP;
       StartAutoTune();
     }     
   }
   
   switch (opState) {
     case OFF:
       Bridge.put("state", "off");
       Console.println("OFF");
       break;
     case SETP:
       Bridge.put("state", "learn");
       Console.println("LEARN");
       break;
     case RUN:
       Bridge.put("state", "on");
       Console.println("ON");
       DoControl();
       break;
     case TUNE_P:
       break;
     case TUNE_I:
       break;
     case TUNE_D:
       break;  
   }   
   
   if (Input > 0) {
     Bridge.put(DEGF, doubleToString(Input, 2));
   }
   Bridge.put(TARGEF, doubleToString(Setpoint,0));
   
   delay(500);
   
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl() {
  double f = thermocouple.readFarenheit();
   if (isnan(f)) {
     Bridge.put("error", "sensor_error");
     //Console.println("Something wrong with thermocouple!");
   } else {
      Input = f;
      Console.println(Input);
     //We're getting good data.  Make is so the bridge can see it
     /*double tempF = thermocouple.readFarenheit();
     */
    if (tuning) { // run the auto-tuner
       if (aTune.Runtime()){ // returns 'true' when done
        FinishAutoTune();
       }
    } else { // Execute control algorithm
       myPID.Compute();
    }
  
    // Time Proportional relay state is updated regularly via timer interrupt.
    onTime = Output;  
  } 
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
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}
 
// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 500;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}
 
 
// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}
 
// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

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
