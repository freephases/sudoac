/*
 Free Phases AC sq wave simulator with current sense and power control via TTL
 Voltage is fixed at 48.01 volts
*/

#include <OnOff.h>
#include <SoftwareSerial.h>
#define DEBUG_TO_SERIAL 1
#define ROB_WS_MAX_STRING_DATA_LENGTH 120

OnOff pwmMosfet(10);
OnOff fan(11);
OnOff forward(6);
OnOff backwards(5);
OnOff led(13);
SoftwareSerial controller(12, 3);
volatile int8_t segment = 0;
int timer1_counter;
const int acs715port = A0;
const unsigned long currentReadMillisInterval = 30;
unsigned long lastCurrentReadMillis = 0;
const int volts = 48;
float averageAmps = 0.0;
float watts = 0.0;
long sampleAmpVal = 0;
long avgSAV = 0;
long sensorValue = 0;
long currentReadCount = 0;
const int samplesToRead = 168;
unsigned long fanTurnOffTimeOut = 0;
unsigned long fanManageLastMillis = 0;
char sendBuf[60];
volatile boolean turnedOn = false;
volatile boolean turnedOff = true;
unsigned long turnOnStartMillis = 0;
short pos = 0; // position in read serialBuffer
char serialBuffer[ROB_WS_MAX_STRING_DATA_LENGTH + 1];
char inByte = 0;

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);  // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV return result;
  return result;
}

void sendData()
{
  String tempStr = String(averageAmps);
  char ampBuf[24];
  tempStr.toCharArray(ampBuf, 24);
  
  String tempStr2 = String(watts);
  char wattsBuf[24];
  tempStr2.toCharArray(wattsBuf, 24);
  
  sprintf(sendBuf, "R|%s|%s|!\n", ampBuf, wattsBuf);
  controller.print(sendBuf);
  Serial.println(sendBuf);
}

void readCurrent()
{
  sensorValue = analogRead(acs715port);
  //  if (sensorValue<100) sensorValue = 100;
  sampleAmpVal += sensorValue;
  currentReadCount++;
  if (currentReadCount == samplesToRead) {
    avgSAV = sampleAmpVal / samplesToRead;

    //reset counters
    sampleAmpVal = 0;
    currentReadCount = 0;

    // convert to milli amps //5003 is 5.003 v
    // Serial.println(readVcc());//489 worked before ;)
    long currentR = (((long)avgSAV * 5002 / 1023) - 493 ) * 1000 / 134;
    if (currentR<0) currentR=0;
  
    averageAmps = (float)currentR / 1000.000;
    watts = averageAmps * 48.000;
    if (watts<0.000) {
      watts = 0.000;
    }

    sendData();
    if (DEBUG_TO_SERIAL == 1) {
      Serial.print(currentR);
      Serial.print(" mA, ");
      Serial.print(averageAmps, DEC);
      Serial.print(" A, ");
      Serial.print(watts, DEC);
      Serial.println(" W");
    }
  }
}

void currentTimer() {
  if (millis() - lastCurrentReadMillis > currentReadMillisInterval) {
    lastCurrentReadMillis = millis();
    readCurrent();
    if (!turnedOff) { 
      if (averageAmps>0.100) led.toggle(); 
      else led.off();
    }    
  }
}

/**
* Turn on fan when using 1 amp or more and turn off fan 5 minutes after using less then 1 amp
*/
void manageFan()
{
  if (millis() - fanManageLastMillis > 5500) {
    fanManageLastMillis = millis();
    if (fanTurnOffTimeOut != 0 && millis() - fanTurnOffTimeOut > 60000*5 && fan.getIsOn()) {
      fan.off();
    }

    if (fan.getIsOn() && averageAmps < 1 && fanTurnOffTimeOut == 0) {
      fanTurnOffTimeOut = millis();
    } else if (averageAmps > 0.9) {
      fanTurnOffTimeOut = 0;
      if (!fan.getIsOn()) {
        fan.on();
      }
    }
  }
}



void scanForIncoming()
{
  // send data only when you receive data:
  while (controller.available() > 0)
  {

    // read the incoming byte:
    inByte = controller.read();

    if (inByte == '\r') continue;

    // add to our read serialBuffer
    serialBuffer[pos] = inByte;
    //   Serial.println(inByte);
    pos++;


    //Serial.println(inByte);
    if (inByte == '\n' || pos == ROB_WS_MAX_STRING_DATA_LENGTH - 1) //end of max field length
    {
      serialBuffer[pos - 1] = 0; // delimit
      if (DEBUG_TO_SERIAL == 1) {
        Serial.print("REQUEST: ");
        Serial.println(serialBuffer);
      }
      processIncoming();
      serialBuffer[0] = '\0';
      pos = 0;
    }

  }
}

void processIncoming() {
  char recordType = serialBuffer[0];
  switch (recordType) {
    case '+' : // on 
      hbTurnOn();
      controller.println("OK|+|!");
      break;
    case '-' : // off      
      hbTurnOff();
      controller.println("OK|-|!");      
    default : 
      break;
  }
}




void setup() {  
  // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  // timer1_counter = 62496;// preload timer 65536 - (62500/15000Hz)
  //timer1_counter = 65305;// preload timer 65536 - (62500/24000Hz)
  timer1_counter = 65519;// preload timer 65536 - (62500/3600Hz) - 3600 / 6 = 600 - scaled to 3600 / 12 to allow for gap to stop shorting
  // timer1_counter = 65328;// preload timer 65536 - (62500/300Hz)

  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
  led.on();
  Serial.begin(9600);
  controller.begin(9600);
  delay(30);
  //let the main logger/controller know we are here
  controller.println("OK|go|!");
  turnOnStartMillis = millis();
  led.off();
}


/*ISR(TIMER1_OVF_vect)        // interrupt service routine for 2 pulse sq wave form ;)
{
  TCNT1 = timer1_counter;   // preload timer

  //1 segments per 1/300th of a second
// 1 cycle is made up up 10 segments:
// [1][0][1][0][0][-1][0][-1][0][0]
  switch(segment){
    case 2:
    case 0:  forward.on(); // + pulse
        break;
    case 3:
    case 1: forward.off();
        break;
    case 9:
    case 4: //space
        break;
    case 7:
    case 5: backwards.on();
        break;
    case 8:
    case 6: backwards.off();
        break;
  }
  segment++;
  if (segment>9) segment = 0;
}
*/

/**
* interrupt service routine for sudo AC with 600 AC pulses per sec
*/
ISR(TIMER1_OVF_vect)        
{
  TCNT1 = timer1_counter;   // preload timer
  if (!turnedOn && !turnedOff) {
    forward.off();
    backwards.off();
    segment = 0;
    turnedOff = true;
    led.off();
  } else if (turnedOn) {     
    //1 segmentment 12/3600Hz with 6 pulses each with a Zero after wards
    // 1 cycle is made up of 1 phase x 3:
    // [1][0][-1][0][1][0][-1][0][1][0][-1][0]
  
    switch (segment) {
      case 0:  forward.on(); // + pulse
        break;
      case 1: forward.off();
        break;
      case 2: backwards.on();// - pulse
        break;
      case 3: backwards.off();
        break;
    }
    segment++;
    if (segment > 3) segment = 0;
  }
}

void hbTurnOn() 
{
  if (!turnedOn) {
    led.on();
    turnedOn = true;
    turnedOff = false;
    turnOnStartMillis = 0;
  }
}

void hbTurnOff() 
{
  if (!turnedOff) {
    turnedOn = false;
  }
}

void loop() {  
  currentTimer();
  scanForIncoming();
  manageFan();
  
  //delayed turn on if not turned off by contoller within 15 secs
 // if (turnOnStartMillis!=0 && millis()-turnOnStartMillis>15000) {
 //   hbTurnOn();
 // }
}
