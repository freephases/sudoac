/***
* Free Phases AC sq wave simulator with current sense and power control via TTL
* Voltage is fixed at 42.00 volts for 4.3 ohm coil
* 
* For the Public domain!
* 
* Your free to criticise and free to reuse whatever you want - Have Fun!
**/

//on/off class see: https://github.com/freephases/arduino-onoff-lib
#include <OnOff.h>
#include <SoftwareSerial.h>
#define DEBUG_TO_SERIAL 1

/**
* Max length of serial data we want to read as a single resquest/command
*/
#define MAX_SERIAL_DATA_LENGTH 30

/**
* H-Bridge heat sink fan
*/
OnOff fan(11);

/**
* H-Bridge positive/forwards
*/
OnOff positive(6);

/**
* H-Bridge negitive/backwards
*/
OnOff negitive(5);

/**
* Led flashes when H-Bridge is on
*/
OnOff led(13);

/**
* Controller serial
*/
SoftwareSerial controller(12, 3);

/**
* Wave form counter for wave form segment 
*/
volatile int8_t segment = 0;

/**
* 16MHz/256 timer time
*/
volatile long timer1_counter;

/**
* Current sensor port
*/
const int acs715port = A0;

/**
* Interval between reading current sensor in millis
*/
const unsigned long currentReadMillisInterval = 22;
/**
* samplesToRead = Number of samples from current sensor to read
*/
const int samplesToRead = 80;

/**
* Last time current was read in millis
*/
unsigned long lastCurrentReadMillis = 0;

/**
* DC input voltage
*/
const float fixedVoltage = 49.9;

/**
* Average DC input Amps
*/
float averageAmps = 0.0;
float totalAverageValues = 0.0;
/**
* Power in Watts (Not RMS)
*/
float watts = 0.0;

/**
* Vars use for calculating current
*/
long sampleAmpVal = 0;
long avgSAV = 0;
long sensorValue = 0;
long currentReadCount = 0;



/**
* fanTurnOffTimeOut - used to switch off fan after a set time after h-bridge is turned off
*/
unsigned long fanTurnOffTimeOut = 0;

/**
* fanManageLastMillis - last time fan management was called
*/
unsigned long fanManageLastMillis = 0;

/**
* vars for sendign serial data and reading it into a char array
*/
char sendBuf[60];
short pos = 0; // position in read serialBuffer
char serialBuffer[MAX_SERIAL_DATA_LENGTH];
char inByte = 0;

int lastPercentage = 0;

const float powerSupplyMaxWatts = 390.00;// is 400 but say 390 to be on the safe side, that is if the fixedVoltage var specified above is correct for given PSU
/**
* turnedOn - true if not yet fully turned off
* turnedOff - true if fully turned off
* Need to redo. thsi was to make things a bit safer, it allows for interupt to turn bridge off not serial within serial request in case interupt was about to turn on a mosfet 
*/
volatile boolean turnedOn = false;
volatile boolean turnedOff = true;


//NOT USED DOES NOT WORK ON MINI PRO 
//readVcc() {
//  long result;
//  // Read 1.1V reference against AVcc
//  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//  delay(2);
//  // Wait for Vref to settle
//  ADCSRA |= _BV(ADSC);  // Convert
//  while (bit_is_set(ADCSRA, ADSC));
//  result = ADCL;
//  result |= ADCH << 8;
//  result = 1126400L / result; // Back-calculate AVcc in mV return result;
//  return result;
//}



/**
* Return a value from within a CSV string where index is the coloumn count, 
* is zero based, 0=1, 1=2 and so on...
*/
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {
    0, -1        };
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
      found++;
      strIndex[0] = strIndex[1]+1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/**
* sendData back to controller
*/
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

/**
* Read and process Current sensor using avg of raw analog values
*/
void readCurrentAvg1()
{
  sensorValue = analogRead(acs715port);
  sampleAmpVal += sensorValue;
  currentReadCount++;
  if (currentReadCount == samplesToRead) {
    avgSAV = sampleAmpVal / samplesToRead;

    //reset counters
    sampleAmpVal = 0;
    currentReadCount = 0;

    // Serial.println(readVcc());//readVcc not working on mini pro ;)
    long currentR = ( ((long)avgSAV * 5006 / 1024) - 500 ) * 1000 / 133;
    
    averageAmps = (float)currentR / 1000.000;
    if (averageAmps<=0.09) averageAmps=0.0;
  
    watts = averageAmps * fixedVoltage;
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

/**
* Read and process Current sensor using avg of calculated analog values
*/
//void readCurrentAvg2()
//{
//  sensorValue = analogRead(acs715port);
//  //sampleAmpVal += sensorValue;
//  long currentR = ( ((long)sensorValue * 5006 / 1024) - 500 ) * 1000 / 133;
//  if (currentR<0) currentR = 0;
//  totalAverageValues += (float)currentR / 1000.000;
//  currentReadCount++;
//  if (currentReadCount == samplesToRead) {
//    averageAmps = totalAverageValues / samplesToRead;
//    
//    //reset counters
//    totalAverageValues = 0;
//    currentReadCount = 0;
//   
//
//    if (averageAmps<=0.09) averageAmps=0.0;
//  
//    watts = averageAmps * fixedVoltage;
//    if (watts<0.000) {
//      watts = 0.000;
//    }
//
//    sendData();
//    if (DEBUG_TO_SERIAL == 1) {
//     // Serial.print(currentR);
//     // Serial.print(" mA, ");
//      Serial.print(averageAmps, DEC);
//      Serial.print(" A, ");
//      Serial.print(watts, DEC);
//      Serial.println(" W");
//    }
//  }
//}

void checkOverLoading()
{ 
  if (!turnedOff) {
    if (watts>powerSupplyMaxWatts) {
      hbTurnOff(); //force stop, using way to much power for way to long      
      controller.println("E|HA|!"); //High Amps message, have to reboot to get out of this, it should not happen if you have calculated coil ohm's correctly.      
    } 
  }
}

void readCurrent() 
{
  if (millis() - lastCurrentReadMillis > currentReadMillisInterval) {
    lastCurrentReadMillis = millis();
    readCurrentAvg1();
    checkOverLoading();
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


/**
* Read known incoming serial data in to char array
*/
void scanForIncoming()
{
  // send data only when you receive data:
  while (controller.available() > 0)
  {

    // read the incoming byte:
    inByte = controller.read();
//Serial.print(inByte);
    if (inByte == '\r') continue; //ignore 
    
    if (inByte == '\n' || pos == MAX_SERIAL_DATA_LENGTH - 1) //we have the end of command line or reached the max we can read 
    {
      serialBuffer[pos] = 0; // delimit end of command char array
      if (DEBUG_TO_SERIAL == 1) {
        Serial.print("REQUEST: ");
        Serial.println(serialBuffer);
      }
      processIncoming();
      serialBuffer[0] = '\0';
      pos = 0;
    } else {
      // add to our read serialBuffer
        serialBuffer[pos] = inByte;
        pos++;
    }

  }
}

/**
* Set speed of h-bridge 0 is around 25Hz AC and 100 is 2.1kHz AC with 48v input - 
* last checked AC freq.: 10th March 2016
* TODO: improve range of times, 0-512 may be better stepping
*/
void setHbSpeed(int percentage)
{
  long newCounter = map((long)percentage, 0, 100, 65225, 65531);  
  
  lastPercentage = percentage;
 
  //keep in range
  if (newCounter>65531) newCounter = 65531;
  else if (newCounter<65225) newCounter = 65225;
  
  timer1_counter = newCounter;
  Serial.print("New counter: ");
  Serial.println(newCounter, DEC);
}

/**
* Process serial commands
*/
void processIncoming() 
{
  char command = serialBuffer[0];
  switch (command) {
    case '+' : // on 
      hbTurnOn();
      controller.println("OK|+|!");
      break;
    case '-' : // off      
      hbTurnOff();
      controller.println("OK|-|!");
      break;    
    case 's' : // off      
      setHbSpeed(getValue(serialBuffer, '|', 1).toInt());
      controller.println("OK|s|!");
      break;    
    default : 
      break;
  }
}

/**
* Interrupt service routine for basic sq wave form with gaps to allow mosfets to close due to back emf from coil
*
* 1 cycle consists of 12 fractions of equal time:
* direction:   [+][+][+][+][0][0][-][-][-][-][0 ][0 ]
* pos:         [0][1][2][3][4][5][6][7][8][9][10][11]
*              
* wave form     | |_   _| |_   _| |_   _ 
*                   |_|     |_|     |_|
*/                  
ISR(TIMER1_OVF_vect)        
{
  TCNT1 = timer1_counter;   // preload timer
  if (!turnedOn && !turnedOff) {
    
    positive.off();
    negitive.off();
    segment = 0;
    turnedOff = true;
    led.off();
    
  } else if (turnedOn) {      
    
    switch(segment){
      case 0 ... 3: positive.on(); // + pulse
          break;
      case 4 ... 5: positive.off();
          break;
      case 6 ... 9: negitive.on();// - pulse
          break;
      case 10 ... 11: negitive.off();
          break;
    }

   segment++;
   if (segment>11) segment = 0;    
    
  }
}

/**
* Interrupt service routine for basic 2/2 pulse sq wave form
*
* 1 cycle consists of 10 fractions of equal time:
* direction:   [+][0][+][0][0][-][0][-][0][0]
* pos:         [0][1][2][3][4][5][6][7][8][9]
*
* wave form  |_|__ _ __
*                 | |
*/
//ISR(TIMER1_OVF_vect)        
//{
//  TCNT1 = timer1_counter;   // preload timer
//  if (!turnedOn && !turnedOff) {
//    
//    positive.off();
//    negitive.off();
//    segment = 0;
//    turnedOff = true;
//    led.off();
//    
//  } else if (turnedOn) {      
//    
//    switch(segment){
//      case 2:
//      case 0:  positive.on(); // + pulse
//          break;
//      case 3:
//      case 1: positive.off();
//          break;
//      case 9:
//      case 4: //space with nothing to do
//          break;
//      case 7:
//      case 5: negitive.on();// - pulse
//          break;
//      case 8:
//      case 6: negitive.off();
//          break;
//    }
//    segment++;
//    if (segment>8) segment = 0;
//    
//  }
//}



///**
//* interrupt service routine for sudo AC sq wave
//*/
//ISR(TIMER1_OVF_vect)        
//{
//  TCNT1 = timer1_counter;   // preload timer
//  if (!turnedOn && !turnedOff) {
//    positive.off();
//    negitive.off();
//    segment = 0;
//    turnedOff = true;
//    led.off();
//  } else if (turnedOn) {     
//    //1 segmentment 12/3600Hz with 2 pulses 
//    // 1 cycle is:
//    // [P][0][N][0]
//  
//    switch (segment) {
//      case 0:  positive.on(); // + pulse
//        break;
//      case 1: positive.off();
//        break;
//      case 2: negitive.on();// - pulse
//        break;
//      case 3: negitive.off();
//        break;
//    }
//    segment++;
//    if (segment > 3) segment = 0;
//  }
//}

/**
* Switch on H-Bridge
*/
void hbTurnOn() 
{
  if (!turnedOn) {
    led.on();
    turnedOn = true;
    turnedOff = false;
  }
}

/**
* Inform H-Bridge interrupt to turn off H-Bridge
*/
void hbTurnOff() 
{
  if (!turnedOff) {
    turnedOn = false;
  }
}

/* Start up and main/looper *********************************************************************************/


void setup()
{  
  // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 65519;// preload timer default timer 65536 - (62500/3600Hz)
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
  led.on();
  Serial.begin(9600);
  controller.begin(9600);
  delay(30);
  //let the main logger/controller know we are here and ready to start dancing (when required)
  controller.println("OK|go|!");
  led.off();
}


void loop() 
{  
  readCurrent();
  scanForIncoming();
  manageFan();  
}
