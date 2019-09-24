  // This sketch takes data from a dial indicator or calipers and pushes it out the arduiono serial port for use by another device.
  // The dial indicator reading is written to the serial port as an ascii-string. Each reading is terminated by a \r\n
  // This code only works with the indicator in mm mode, not inch.
  
  // dial indicator constants

  // clk pin must be connectedf to a hardware interrupt pin.
  #define DIAL_INDICATOR_CLOCK_PIN 2
  
  #define DIAL_INDICATOR_DATA_PIN  3
  #define DIAL_INDICATOR_CLOCK_INTERRUPT (digitalPinToInterrupt(DIAL_INDICATOR_CLOCK_PIN))
  #define DIAL_INDICATOR_PACKET_START_PULSE_MILLISECONDS 100
  #define DIAL_INDICATOR_MILLIMETERS_SCALE_FACTOR 100.00

  // dial indicator last output
  float dialIndicatorReadingMillimeters;

  // dial indicator internal variables
  volatile unsigned long dialIndicatorLastClockRiseMilliseconds;
  volatile int dialIndicatorPacketIndex;
  volatile long dialIndicatorPartialPacket;
  volatile bool dialIndicatorNextReadingIsPositive;
  volatile bool dialIndicatorNextReadingIsInches;
  volatile bool dialIndicatorNewReadingReceived;

void setup() {
  //turn on serial
  Serial.begin(9600);
 // Serial1.begin(9600);

  //setup dial indicator connection
  setupDialIndicator();
}

void loop() {
    if(dialIndicatorNewReadingReceived){
      Serial.println(dialIndicatorReadingMillimeters, 2);
      //Serial1.println(dialIndicatorReadingMillimeters, 2);
      dialIndicatorNewReadingReceived = false;
    }
}

void setupDialIndicator(){
  pinMode(DIAL_INDICATOR_CLOCK_PIN, INPUT_PULLUP);
  pinMode(DIAL_INDICATOR_DATA_PIN, INPUT_PULLUP);
  attachInterrupt(DIAL_INDICATOR_CLOCK_INTERRUPT, dialIndicatorClock, RISING);

  dialIndicatorLastClockRiseMilliseconds = 0;
  dialIndicatorReadingMillimeters = -666;
  dialIndicatorPacketIndex = -1;
  dialIndicatorPartialPacket = 0;
  dialIndicatorNewReadingReceived = false;
}

void dialIndicatorClock() {
    unsigned long currTime = millis();
    // each data packet is preceeded by a clk-high for 100-150ms
    if( (dialIndicatorPacketIndex < 0) && ((currTime-dialIndicatorLastClockRiseMilliseconds) > DIAL_INDICATOR_PACKET_START_PULSE_MILLISECONDS)) 
    {
      dialIndicatorPacketIndex = 0;
      dialIndicatorPartialPacket = 0;
    }
    else if(dialIndicatorPacketIndex>=0)
    {

      // data pin value is inverted at in hardware with a transistor due to voltage differences
      bool dataPinValue = digitalRead(DIAL_INDICATOR_DATA_PIN)?false:true; 

      if(dialIndicatorPacketIndex<20) // first 20 bits of packet are dial reading in 100ths of a millmetre
      {
        if(dataPinValue){
          dialIndicatorPartialPacket |= 1<<dialIndicatorPacketIndex;
        }
        dialIndicatorPacketIndex++;
      }
      else if(dialIndicatorPacketIndex==20) // 20th bit is sign of dial reading. Positive = true.
      {
        dialIndicatorNextReadingIsPositive = dataPinValue;
        dialIndicatorPacketIndex++;
      }
      else if((dialIndicatorPacketIndex>20)&&(dialIndicatorPacketIndex<23)) // bits 21-22 are unused
      {
        dialIndicatorPacketIndex++;
      }
      else if(dialIndicatorPacketIndex==23)  // bit 23 is onstensibly used to indicate if the value is in mm or inches but it seems like a lot of dial indicators don't accurately report this. This bit is ignored in this implementation
      {
        float signAdjustment = dialIndicatorNextReadingIsPositive?-1.0:1.0;

        dialIndicatorReadingMillimeters = ((float)dialIndicatorPartialPacket*signAdjustment)/(DIAL_INDICATOR_MILLIMETERS_SCALE_FACTOR);
        dialIndicatorPacketIndex = -1;
        dialIndicatorNewReadingReceived = true;
      }
    }
    dialIndicatorLastClockRiseMilliseconds = currTime;
  }
