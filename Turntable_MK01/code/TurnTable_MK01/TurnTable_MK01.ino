/**********************************

Turntable Arduino Code

by Martin Fröhlich / http://beamstreamer.com

für 

NEMA13 Stepper motor mit Pololu Stepper-Driver DRV8825
LTN Encoder Mod. G24WRLDBI-0512-133-05
Contelec PL300 - 10k 360 Grad Durchdrehendes Plastikfolien Potentionmeter

together with turntable shield A V0.1

************************************/

#include <avr/io.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#include <AccelStepper.h>
//AccelStepper Library: http://www.airspayce.com/mikem/arduino/AccelStepper/

#include <CmdMessenger.h>  // CmdMessenger
//Messanger Library: http://playground.arduino.cc/Code/CmdMessenger

#include <digitalWriteFast.h> 
//Fast Read Library: https://code.google.com/p/digitalwritefast/

#include <Encoder.h>
//ENCODER LIBRARY: http://www.pjrc.com/teensy/td_libs_Encoder.html

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

int potiAnalogPin[3] = {0, 1, 2};

//Pin connected to STEP on driverbreakout
int stepPin = 9;
//Pin connected to DIR on driverbreakout
int directionPin = 10;
//Pin connected to ENBL on driverbreakout
int enablePin = 8;

const int signalPinA = 2;
const int signalPinB = 3;
const int signalPinZ = 4;

int incomingByte = 0;

AccelStepper stepper(1, stepPin, directionPin); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

Encoder myEnc(signalPinA, signalPinB);

// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial);

boolean sendTX = false;

// This is the list of recognized commands. These can be commands that can either be sent or received. 
// In order to receive, attach a callback function to these events
enum
{
  kSetSpeed            , // Command to set speed
  kNextFrame           , // Command to report next frame
  kData                , // Command to report Data
};

// Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kSetSpeed, OnSetSpeed);
  cmdMessenger.attach(kNextFrame, OnNextFrame);
}

// Called when a received command has no attached function
void OnUnknownCommand()
{
  //cmdMessenger.sendCmd(kStatus,"Command without attached callback");
}

// Callback function that sets led on or off
void OnSetSpeed()
{
  // Read speed argument, interpret string as int
  int speedState = cmdMessenger.readIntArg();
  
  setMotorSpeed(speedState);
}

void OnNextFrame(){
  sendTX = true;
}

void setup(){
  //driverbreakout requires an inverted enable signal
  stepper.setPinsInverted(false,false, true);
  //specify the enable pin
  stepper.setEnablePin(enablePin);
  stepper.setMaxSpeed(500);
  setMotorSpeed(350);

  //stepper.setAcceleration(20);
  //stepper.moveTo(500);
  
  pinMode(signalPinA, INPUT);
  pinMode(signalPinB, INPUT);
  pinMode(signalPinZ, INPUT);

  for(int i = 0; i < 3; i++){
    pinMode(potiAnalogPin[i], INPUT);
  }

  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_64;    // set our own prescaler to 64 

  //stepper.disableOutputs();

  Serial.begin(115200);
  //Serial.begin(57600);
  // Adds newline to every command
  cmdMessenger.printLfCr();   

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();
}

boolean newPinA, newPinB, newPinZ;
boolean pinA, pinB, pinZ;

int flankCounter = 0;
int newFlankCounter = 0;
boolean flankEvent = false;
boolean calibrated = false;

int turningDir = 0;

int newAnalogValue[3];
int analogValue[3];
int analogDir[3];

byte pinZMask = 0;

void loop(){
  flankEvent = false;

  // The hardware interface allows to attach up to three potis
  for(int i = 0; i < 3; i++){
    newAnalogValue[i] = analogRead(potiAnalogPin[i]);
    if(newAnalogValue[i] != analogValue[i]){
       analogDir[i] += analogValue[i] - newAnalogValue[i];
       analogValue[i] = newAnalogValue[i];
      //sendTX = true;
    }
  }

  // the turntabe turns to z axis zero before it starts
  if(!calibrated && digitalReadFast(signalPinZ)){
    myEnc.write(2048); 
    setMotorSpeed(0);
    calibrated = true;
    sendTX = true;
  }else{
    newFlankCounter =  myEnc.read();
    if(flankCounter != newFlankCounter){
      if(newFlankCounter == 0 || newFlankCounter == 4096){
        myEnc.write(2048); 
        turningDir += (flankCounter - newFlankCounter > 0)?-1:1;
        flankCounter = 2048;
      } else {
        turningDir += (flankCounter - newFlankCounter > 0)?1:-1;
        flankCounter = newFlankCounter;
      }
    }
  }
  
  if(sendTX){
    //Serial.println(analogValue);
    TX();
    sendTX = false;
  }
  
  // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();
  //stepper.run();
  stepper.runSpeed();
    
}

void setMotorSpeed(int mSpeed){
  if(mSpeed == 0)
    stepper.disableOutputs();
   else 
     stepper.enableOutputs();
  stepper.setSpeed(mSpeed);
  
}

byte sendBuffer[14];

void TX(){
    word value = (flankCounter -1 ) % 2048;
    //word value = flankCounter;
    sendBuffer[0] = (calibrated)? 1: 0; // gate
    sendBuffer[1] = (analogDir[0] & 255) + 127; // analog0
    sendBuffer[2] = (analogDir[1] & 255) + 127; // analog0
    sendBuffer[3] = (analogDir[2] & 255) + 127; // analog0
    sendBuffer[4] = analogValue[0] >> 7;
    sendBuffer[5] = analogValue[0] & 127;
    sendBuffer[6] = analogValue[1] >> 7;
    sendBuffer[7] = analogValue[1] & 127;
    sendBuffer[8] = analogValue[2] >> 7;
    sendBuffer[9] = analogValue[2] & 127;
    sendBuffer[10] = turningDir + 127;
    sendBuffer[11] = value >> 7;
    sendBuffer[12] = value & 127;
    sendBuffer[13] = 255;
    Serial.write(sendBuffer, 14); 
    turningDir = 0; 
    analogDir[0] = 0;
    analogDir[1] = 0;
    analogDir[2] = 0;
}
