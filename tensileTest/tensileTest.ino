#include "PID_v1.h"
#include <HX711_ADC.h>
#include <EEPROM.h>

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
int joystickPin = A10;
int joySwitchPin = 2;
char serialData;

//time variables
unsigned long timeElapsed; //ms
unsigned long timePrevious; //ms

//constants
int lastEncoderValue;
float speedToPWMConversion = 120;
float fullRotation = 360.0; //degrees
float fullRotationDistance = 2.0; //mm
float linearDistance = 0.0; //mm
float encoderSlots = 50.0;
float time = 0; //ms
float distancePerStep = fullRotationDistance/encoderSlots; //mm
const float linearizationConst = (2.0/360.0); //mm/degree
float msConversion = 1000.0;
int encoderSteps = 1;
float angularVelocity = 0;
int numberOfLoops = 0;
int jogOutput = 0;
bool RunTest = false;

String TestData;

//PID requirements
  //Input, Output and Setpoint for PID library
  double linearVelocity;
  double motorOutput = 0;
  double motorSetpoint;
  double mappedLinearVelocity;
  double mappedMotorSetpoint;

  //Gain variables
  // double Kp = 0, Ki = 0, Kd = 0;
  double Kp = 10, Ki = 80, Kd = 5;

  //declare PID
  PID myPID(&linearVelocity, &motorOutput, &motorSetpoint, Kp, Ki, Kd, DIRECT);

  // dial indicator constants
  #define DIAL_INDICATOR_CLOCK_PIN 2
  #define DIAL_INDICATOR_DATA_PIN  17
  #define DIAL_INDICATOR_CLOCK_INTERRUPT (digitalPinToInterrupt(DIAL_INDICATOR_CLOCK_PIN))
  #define DIAL_INDICATOR_PACKET_START_PULSE_MICROS 1000
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

void setup() {


  //turn on serial
  Serial.begin(9600);
  pinMode(relayPin, OUTPUT);
  //pinMode(joySwitchPin, INPUT);

  //new load cell library
  float calValue; // calibration value
  calValue = 2150.0; // uncomment this if you want to set this value in the sketch
  #if defined(ESP8266)
  //EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
  #endif
  //EEPROM.get(eepromAdress, calValue); // uncomment this if you want to fetch the value from eeprom
  LoadCell.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  if(LoadCell.getTareTimeoutFlag()) {
    Serial.println("Tare timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calValue); // set calibration value (float)
    Serial.println("Startup + tare is complete");
  }
  attachInterrupt(digitalPinToInterrupt(doutPin), whenreadyISR, FALLING);

  //turn on PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  motorSetpoint = .2;

  lastEncoderValue = 0;
  angularVelocity = 0;
  delay(2000);
  int numberOfLoops = 0;

  //setup dial indicator connection
  setupDialIndicator();

}

//interrupt routine:
void whenreadyISR() {
  LoadCell.update();
}

void loop() {
  Serial.println(dialIndicatorReadingMillimeters, 2);
   int joystickValue = analogRead(joystickPin);
  //  Serial.println(digitalRead(joySwitchPin));

        if (Serial.available() > 0) {
        serialData = Serial.read();
        Serial.println(serialData);

        if (serialData == '1'){
          RunTest = true;
        } else if (serialData == '0'){
          RunTest = false;
          analogWrite(motorPin, 0);
          Serial.print(TestData);
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
          if (encoderValue == 1 && lastEncoderValue != encoderValue){

            encoderSteps++;
            numberOfLoops++;

            //new load cell library
            float cellReading = LoadCell.getData();

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

            LogTestDataToSerial(cellReading);
          }

          if(encoderValue == 0){
            //change last encoder value to 0
            lastEncoderValue = encoderValue;
          }
        myPID.Compute();
      }else{
        analogWrite(motorPin, 0);
      }

    } else {

    RunTest = false;
    if (joystickValue >= 951){ //go downward
      digitalWrite(relayPin, LOW);
      jogOutput = map(joystickValue, 951, 1023, 0, 250);
      analogWrite(motorPin, jogOutput);
    }

    if (joystickValue <= 99) { //go upward
      digitalWrite(relayPin, HIGH);
      jogOutput = map(joystickValue, 99, 0, 0, 250);
      analogWrite(motorPin, jogOutput);
    }
  }
  }

void LogTestDataToSerial (float cellReading){
  Serial.print(linearDistance, 2);
  Serial.print(" ");
  Serial.print(linearVelocity, 2);
  Serial.print(" ");
  Serial.print(cellReading, 2);
  Serial.print(" ");
  Serial.println(dialIndicatorReadingMillimeters, 2);
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
