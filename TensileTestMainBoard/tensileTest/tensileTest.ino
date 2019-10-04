#include "PID_v1.h"
#include <HX711_ADC.h>
#include <Adafruit_NeoPixel.h>

//led stuff
#define LED_PIN  3
#define LED_COUNT 72
#define BAR_HEIGHT_INPUT A1
#define BAR_COLOR_INPUT A2
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


const int doutPin = 19; //mcu > HX711 dout pin, must be external interrupt capable
const int sckPin = 7; //mcu > HX711 sck pin
HX711_ADC LoadCell(doutPin, sckPin);
const int eepromAdress = 0;
long t;

//pins on arduino
const int encoderPin = 5;
const int motorPin = 9;
const int downwardEndStop = 3;
const int upwardEndStop = 4;
const int relayPin = 24;
int LoadCelltoRGBStripPin = A0;
int DialIndicatorToRGBStripPin = A1;
int joystickPin = A10;
// int joySwitchPin = 2;
char serialData;

//time variables
unsigned long timeElapsed; //ms
unsigned long timePrevious; //ms

//constants
float speedToPWMConversion = 120;
float fullRotation = 360.0; //degrees
float fullRotationDistance = 0.0392; //mm
float encoderSlots = 10;
float distancePerStep = fullRotationDistance/encoderSlots; //mm
const float linearizationConst = fullRotationDistance/360.0; //mm/degree
const float LoadCellToNewtons = 9.81/2.205;
float msConversion = 1000.0;

//variables
float time = 0; //ms
int encoderSteps = 1;
float angularVelocity = 0;
int numberOfLoops = 0;
int jogOutput = 0;
int lastEncoderValue;
bool RunTest = false;
float linearDistance = 0.0; //mm

//PID requirements
//Input, Output and Setpoint for PID library
double linearVelocity;
double motorOutput = 0;
double motorSetpoint;
//Gain variables
double Kp = 5, Ki = 50, Kd = 7.5;

//load cell values
float cellReading;
float calValue = 2150.0; // calibration value
long stabilisingtime = 2000; // tare precision can be improved by adding a few seconds of stabilising time 

//declare PID
PID myPID(&linearVelocity, &motorOutput, &motorSetpoint, Kp, Ki, Kd, DIRECT);

// dial indicator constants
#define DIAL_INDICATOR_CLOCK_PIN 2
#define DIAL_INDICATOR_DATA_PIN  17
#define DIAL_INDICATOR_CLOCK_INTERRUPT (digitalPinToInterrupt(DIAL_INDICATOR_CLOCK_PIN))
#define DIAL_INDICATOR_PACKET_START_PULSE_MICROS 5000
#define DIAL_INDICATOR_MILLIMETERS_SCALE_FACTOR 100.00

// dial indicator last output
float dialIndicatorReadingMillimeters;

// dial indicator internal variables
volatile int dialIndicatorPreviousClockState;
volatile unsigned long dialIndicatorLastClockRiseMicroseconds;
volatile int dialIndicatorPacketIndex;
volatile long dialIndicatorPartialPacket;
volatile bool dialIndicatorNextReadingIsPositive;
volatile bool dialIndicatorReadingIsInches;
volatile bool dialIndicatorNextReadingIsInches;

char DialIndicatorReading[10];
char *DialIndicatorReadingToPrint;
char PartialDialIndicatorReading[10];
int DialIndicatorReadingIndex;


void setup() 
{
  //turn on serial
  Serial.begin(9600);
  Serial2.begin(9600);
  pinMode(relayPin, OUTPUT);
  pinMode(LoadCelltoRGBStripPin, OUTPUT);
  pinMode(DialIndicatorToRGBStripPin, OUTPUT);

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  //initialize load cell library
  LoadCell.begin();
  LoadCell.start(stabilisingtime);
  if(LoadCell.getTareTimeoutFlag()) 
  {
    Serial.println("Tare timeout, check MCU>HX711 wiring and pin designations");
  }
  else 
  {
    LoadCell.setCalFactor(calValue); // set calibration value (float)
    Serial.println("Startup + tare is complete");
  }
  attachInterrupt(digitalPinToInterrupt(doutPin), whenreadyISR, FALLING);

  //turn on PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  motorSetpoint = 0.08;
  lastEncoderValue = 0;
  angularVelocity = 0;
  delay(2000);

  //setup dial indicator connection
  setupDialIndicator();
  DialIndicatorReadingIndex = -1;
}

//interrupt routine:
void whenreadyISR() 
{
  LoadCell.update();
}

void loop() 
{
  while (Serial2.available())
  {
    char lastReceived = Serial2.read();
    if( lastReceived == '\n')
    {
      DialIndicatorReadingIndex = 0;
    }
    else if( lastReceived == '\r')
    {
      PartialDialIndicatorReading[DialIndicatorReadingIndex] = '\0';
      strcpy(DialIndicatorReading,PartialDialIndicatorReading);
      DialIndicatorReadingIndex = -1;
    }
    else if(DialIndicatorReadingIndex >= 0)
    {
      PartialDialIndicatorReading[DialIndicatorReadingIndex] = lastReceived;
      DialIndicatorReadingIndex++;
    }
  }
  if(DialIndicatorReading[0])
  {
    // Serial.print(DialIndicatorReading);
    DialIndicatorReadingToPrint = DialIndicatorReading;
  }
  int joystickValue = analogRead(joystickPin);
  if (Serial.available() > 0) 
  {
    serialData = Serial.read();
    Serial.println(serialData);
  
    if (serialData == '1')
    {
      RunTest = true;
      motorOutput = 0;
      motorSetpoint = 0.08;
      lastEncoderValue = 0;
      angularVelocity = 0;
    } 
    else if (serialData == '0')
    {
      RunTest = false;
      analogWrite(motorPin, 0);
    }
  }
  if ((joystickValue >= 100) && (joystickValue <= 950))
  {
    if(RunTest == true)
    {
      timeElapsed = millis();
      int encoderValue = digitalRead(encoderPin);
      int downwardEndStopValue = digitalRead(downwardEndStop);
      digitalWrite(relayPin, HIGH);
      analogWrite(motorPin, motorOutput);
        //if new encoder reaches new slot, do something
        if (encoderValue == 1 && lastEncoderValue != encoderValue)
        {
          // encoderSteps++;
          // numberOfLoops++;
          //new load cell library
          cellReading = LoadCell.getData();
          cellReading = cellReading * LoadCellToNewtons;
          //change last encoder value to 1
          lastEncoderValue = encoderValue;
          //time between encoder readings = current time - previous time (ms)
          time = timeElapsed - timePrevious;
          timePrevious = millis();
          //linear distance travelled = last recorded distance + step distance (mm)
          linearDistance = linearDistance + distancePerStep;
          //angular velocity = full rotation degrees / full rotation slots / time in seconds (degree/sec)
          angularVelocity = fullRotation/encoderSlots*(msConversion/time);
          //linear velocity, convert angular velocity to linear (mm/s)
          linearVelocity = angularVelocity * linearizationConst;

          SendDataToRGBStrip();
          // LogTestDataToSerial();
        }
        if(encoderValue == 0)
        {
          lastEncoderValue = encoderValue; //change last encoder value to 0
        }
      myPID.Compute();
    }
    else
    {
      SystemIdleRGB();
      analogWrite(motorPin, 0);
    }
  } 
  else 
  {
    RunTest = false;
    if (joystickValue >= 951) //go downward
    { 
      digitalWrite(relayPin, LOW);
      jogOutput = map(joystickValue, 951, 1023, 0, 250);
      analogWrite(motorPin, jogOutput);
      JogDownRBG();
    }

    if (joystickValue <= 99) //go upward 
    { 
      digitalWrite(relayPin, HIGH);
      jogOutput = map(joystickValue, 99, 0, 0, 250);
      analogWrite(motorPin, jogOutput);
      JogUpRBG();
    }
  }
}

void SendDataToRGBStrip (){
  float LoadCelltoRGBStripColor = (constrain(cellReading,0,2500))*((255)/2500);
  float DialIndicatorReadingValue; 
  DialIndicatorReadingValue = atof(DialIndicatorReadingToPrint);
  float DialIndicatorToRGBStripHeight = (constrain(DialIndicatorReadingValue,-12.85,0))*((LED_COUNT-1)/-12.85)+1;
  Serial.print(DialIndicatorReadingValue);
  Serial.print(" ");
  Serial.print(DialIndicatorToRGBStripHeight);
  Serial.print(" ");
  Serial.println(LoadCelltoRGBStripColor);
  strip.fill(strip.Color(0, 0, 0), DialIndicatorToRGBStripHeight, LED_COUNT-DialIndicatorToRGBStripHeight);
  strip.fill(strip.Color(LoadCelltoRGBStripColor, 0, 255-LoadCelltoRGBStripColor), 0, DialIndicatorToRGBStripHeight);
  strip.show();
  // Serial.print(cellReading);
  // Serial.print(" ");
  // Serial.print(LoadCelltoRGBStripColor);
  // Serial.print(" ");
  // Serial.print(DialIndicatorReadingToPrint);
  // Serial.print(" ");
  // Serial.println(DialIndicatorToRGBStripHeight);
}
void SystemIdleRGB(){
  strip.fill(strip.Color(0, 255, 0), 0, LED_COUNT); //green
  strip.show();
}
void JogUpRBG(){
  int LEDjog = map(jogOutput, 0, 255, 0, LED_COUNT);
  strip.fill(strip.Color(0, 255, jogOutput), 0, LEDjog);
  strip.show();
}
void JogDownRBG(){
  int LEDjog = map(jogOutput, 0, 255, 0, LED_COUNT);
  strip.fill(strip.Color(0, 255, jogOutput), LEDjog, 0);
  strip.show();
}


void LogTestDataToSerial (){
  Serial.print(linearDistance, 2);
  Serial.print(" ");
  Serial.print(linearVelocity, 4);
  Serial.print(" ");
  Serial.print(cellReading, 2);
  Serial.print(" ");
  Serial.println(DialIndicatorReadingToPrint);
}


void setupDialIndicator(){
  pinMode(DIAL_INDICATOR_CLOCK_PIN, INPUT_PULLUP);
  pinMode(DIAL_INDICATOR_DATA_PIN, INPUT_PULLUP);
  attachInterrupt(DIAL_INDICATOR_CLOCK_INTERRUPT, dialIndicatorClock, RISING);

  dialIndicatorPreviousClockState = -1;
  dialIndicatorLastClockRiseMicroseconds = 0;
  dialIndicatorReadingMillimeters = -666;
  dialIndicatorPacketIndex = -1;
  dialIndicatorPartialPacket = 0;
}

void dialIndicatorClock() {
    if( (dialIndicatorPacketIndex == -1 ) &&
        ((micros()-dialIndicatorLastClockRiseMicroseconds) > DIAL_INDICATOR_PACKET_START_PULSE_MICROS))
    {
      dialIndicatorPacketIndex = 0;
      dialIndicatorPartialPacket = 0;
    }
    else if(dialIndicatorPacketIndex>=0)
    {

      bool dataPinValue = digitalRead(DIAL_INDICATOR_DATA_PIN)?false:true;
      if(dialIndicatorPacketIndex<20)
      {
        if(dataPinValue){
          dialIndicatorPartialPacket |= 1<<dialIndicatorPacketIndex;
        }
        dialIndicatorPacketIndex++;
      }
      else if(dialIndicatorPacketIndex==20)
      {
        dialIndicatorNextReadingIsPositive = dataPinValue;
        dialIndicatorPacketIndex++;
      }
      else if((dialIndicatorPacketIndex>20)&&(dialIndicatorPacketIndex<23))
      {
        dialIndicatorPacketIndex++;
      }
      else if(dialIndicatorPacketIndex==23)
      {
        float signAdjustment = dialIndicatorNextReadingIsPositive?-1.0:1.0;

        dialIndicatorReadingMillimeters = ((float)dialIndicatorPartialPacket*signAdjustment)/(DIAL_INDICATOR_MILLIMETERS_SCALE_FACTOR);
        dialIndicatorPacketIndex = -1;
        //Serial.println(dialIndicatorReadingMillimeters, 2);
      }
      //Serial.print(dialIndicatorPacketIndex);
      //Serial.print(" ");
      //Serial.println(dialIndicatorPartialPacket, BIN);
    }
    dialIndicatorLastClockRiseMicroseconds = micros();
  }
