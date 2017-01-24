/***
  Free Phases AC sq wave simulator with current sense and power control via TTL
  Voltage is fixed at 42.00 volts for 4.3 ohm coil

  For the Public domain!

  Your free to criticise and free to reuse whatever you want - Have Fun!
**/

//on/off class see: https://github.com/freephases/arduino-onoff-lib
#include <OnOff.h>
#include <SoftwareSerial.h>
#define DEBUG_TO_SERIAL 0

/**
  Max length of serial data we want to read as a single resquest/command
*/
#define MAX_SERIAL_DATA_LENGTH 30

/**
  H-Bridge heat sink fan
*/
OnOff fan(11);

/**
  H-Bridge positive/forwards
*/
OnOff positive(6);

/**
  H-Bridge negitive/backwards
*/
OnOff negitive(5);

/**
  Led flashes when H-Bridge is on
*/
OnOff led(13);

/**
   step down
*/
//SoftwareSerial stepdown(2, 9);

/**
  Controller serial
*/
SoftwareSerial controller(12, 3);

/**
  Wave form counter for wave form segment
*/
volatile int8_t segment = 0;

/**
  16MHz/256 timer time
*/
volatile long timer1_counter;

/**
  Current sensor port
*/
const int acs715port = A0;

/**
  Interval between reading current sensor in millis
*/
const unsigned long currentReadMillisInterval = 1000;
/**
  samplesToRead = Number of samples from current sensor to read
*/
//const int samplesToRead = 66;

/**
  Last time current was read in millis
*/
unsigned long lastCurrentReadMillis = 0;

/**
  DC input voltage
*/
const float fixedVoltage = 50.0;//as of 13-12-2016

/**
  Average DC input Amps
*/
float averageAmps = 0.0;
//float totalAverageValues = 0.0;
/**
  Power in Watts (Not RMS)
*/
float watts = 0.0;

/**
  Vars use for calculating current
*/
long sampleAmpVal = 0;
long avgSAV = 0;
long sensorValue = 0;
long currentReadCount = 0;

int8_t maxSegments = 7;//0 based 7=8 segments, stop tim extended by 'd' command


/**
  fanTurnOffTimeOut - used to switch off fan after a set time after h-bridge is turned off
*/
unsigned long fanTurnOffTimeOut = 0;

/**
  fanManageLastMillis - last time fan management was called
*/
unsigned long fanManageLastMillis = 0;

/**
  vars for sendign serial data and reading it into a char array
*/
char sendBuf[70];
short pos = 0; // position in read serialBuffer
char serialBuffer[MAX_SERIAL_DATA_LENGTH];
char inByte = 0;

int lastPercentage = 0;

const float powerSupplyMaxWatts = 500.00;// is 400 but say 390 to be on the safe side, that is if the fixedVoltage var specified above is correct for given PSU
/**
  turnedOn - true if not yet fully turned off
  turnedOff - true if fully turned off
  Need to redo. thsi was to make things a bit safer, it allows for interupt to turn bridge off not serial within serial request in case interupt was about to turn on a mosfet
*/
volatile boolean turnedOn = false;
volatile boolean turnedOff = true;

char stepdownInChar;
char stepdownSerialBuf[MAX_SERIAL_DATA_LENGTH];
int stepdownBufPos = 0;
float stepdownVoltage = 0.000;

#define LL_WF_MAX_PATS 40
short waveFormPatterns[LL_WF_MAX_PATS] = {1,1,0,2,2,0};
short waveFormTotalFrames = 6;
volatile waveFormFramePlayingNow = 0;


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
  Return a value from within a CSV string where index is the coloumn count,
  is zero based, 0=1, 1=2 and so on...
*/
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {
    0, -1
  };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/**
  sendData back to controller
*/
void sendData()
{
  String tempStr = String(averageAmps);
  char ampBuf[24];
  tempStr.toCharArray(ampBuf, 24);

  String tempStr2 = String(watts);
  char wattsBuf[24];
  tempStr2.toCharArray(wattsBuf, 24);

  String tempStr3 = String(stepdownVoltage);
  char voltsBuf[24];
  tempStr3.toCharArray(voltsBuf, 24);

  sprintf(sendBuf, "R|%s|%s|%s|!\n", ampBuf, wattsBuf, voltsBuf);
  controller.print(sendBuf);
  //Serial.println(sendBuf);
}

void requestStepdownStats()
{
  Serial.println(":04rvjw\r\n");
}

/**
  Read and process Current sensor using avg of raw analog values
*/
//void readCurrentAvg1XX()
//{
//  sensorValue = analogRead(acs715port);
//  sampleAmpVal += sensorValue;
//  currentReadCount++;
//  if (currentReadCount == samplesToRead) {
//    avgSAV = sampleAmpVal / samplesToRead;
//
//    //reset counters
//    sampleAmpVal = 0;
//    currentReadCount = 0;
//
//    // Serial.println(readVcc());//readVcc not working on mini pro ;)
//    long currentR = ( ((long)avgSAV * 4889 / 1024) - 500 ) * 1000 / 133;
//
//    averageAmps = (float)currentR / 1000.000;
//    if (averageAmps <= 0.09) averageAmps = 0.0;
//
//    watts = averageAmps * fixedVoltage;
//    if (watts < 0.000) {
//      watts = 0.000;
//    }
//
//    sendData();
//    if (DEBUG_TO_SERIAL == 1) {
//      Serial.print(currentR);
//      Serial.print(" mA, ");
//      Serial.print(averageAmps, DEC);
//      Serial.print(" A, ");
//      Serial.print(watts, DEC);
//      Serial.println(" W");
//    }
//  }
//}

/**
  Read and process Current sensor using avg of calculated analog values
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
    if (watts > powerSupplyMaxWatts) {
      hbTurnOff(); //force stop, using way to much power for way to long
      controller.println("E|HA|!"); //High Amps message, have to reboot to get out of this, it should not happen if you have calculated coil ohm's correctly.
    }
  }
}

void readCurrent()
{
  if (millis() - lastCurrentReadMillis > currentReadMillisInterval) {
    lastCurrentReadMillis = millis();
    //readCurrentAvg1();
    requestStepdownStats();
    //checkOverLoading();
    if (!turnedOff) {
      if (averageAmps > 0.100) led.toggle();
      else led.off();
    }
  }
}

/**
  Turn on fan when using 1 amp or more and turn off fan 5 minutes after using less then 1 amp
*/
void manageFan()
{
  if (millis() - fanManageLastMillis > 5500) {
    fanManageLastMillis = millis();
    if (fanTurnOffTimeOut != 0 && millis() - fanTurnOffTimeOut > 60000 * 5 && fan.getIsOn()) {
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


void processStepdownSerial()
{
  if (stepdownSerialBuf[0] != ':') return; //not valid

  if (strlen(stepdownSerialBuf) > 5 && stepdownSerialBuf[3] == 'r') {
    stepdownSerialBuf[strlen(stepdownSerialBuf) - 1] = 0; //remove hash check

    String rStr = String(stepdownSerialBuf).substring(5);

    switch (stepdownSerialBuf[4]) {
      case 'v' : // voltage
        stepdownVoltage = float(rStr.toInt() / 100.0);
        break;

      case 'j' : // current
        averageAmps = float(rStr.toInt() / 100.0);
        break;

      case 'w' : // milliwatts as watts
        watts = float(rStr.toInt() / 1000.0);
        sendData();//send data once watts are read
        break;

      default:
        break;
    }
  }
}

void readStepdownSerial()
{
  while (Serial.available() > 0)
  {
    // read the incoming byte:
    stepdownInChar = Serial.read();
    if (stepdownInChar == '\r') continue; //ignore

    if (stepdownInChar == '\n' || stepdownBufPos == MAX_SERIAL_DATA_LENGTH - 1) //we have the end of command line or reached the max we can read
    {
      stepdownSerialBuf[stepdownBufPos] = 0; // delimit end of command char array
      processStepdownSerial();
      stepdownSerialBuf[0] = '\0';
      stepdownBufPos = 0;
    } else {
      // add to our read serialBuffer
      stepdownSerialBuf[stepdownBufPos] = stepdownInChar;
      stepdownBufPos++;
    }
  }
}



/**
  Read known incoming serial data in to char array
*/
void scanForIncoming()
{
  // send data only when you receive data:
  //controller.listen();
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
  readStepdownSerial();

}

void setCycleGap(int cycleStartDelayCount)
{
  maxSegments = 7 + cycleStartDelayCount;
  if (maxSegments < 7) maxSegments = 7; //stop nagitive values, must always be 4 steps 0=1
  else if (maxSegments > 99) maxSegments = 99;
}

/**
  Set speed of h-bridge 0 is around 25Hz AC and 255 is 2.1kHz AC with 48v input -
  last checked AC freq.: 10th March 2016
*/
void setHbSpeed(int value)
{
  long newCounter = map((long)value, 0, 255, 65225, 65531);

  lastPercentage = percentage;

  //keep in range
  if (newCounter > 65531) newCounter = 65531;
  else if (newCounter < 65225) newCounter = 65225;

  timer1_counter = newCounter;
  //  Serial.print("New counter: ");
  //  Serial.println(newCounter, DEC);
}

void setVoltage(double newVoltage) {
  //Serial.println(newVoltage);
  int integerPart = (int)newVoltage;
  int decimalPart = ((int)(newVoltage * 100) % 100);
  String s = "";
  if (decimalPart == 0) s = "0";
  char buf[16];
  char buf2[2];
  s.toCharArray(buf2, 2);
  sprintf(buf, ":04su%d%d%s\r\n", integerPart, decimalPart, buf2);
  Serial.println(buf);
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setVoltageByPwmValue(int pwmValue)
{
  double newVoltage  = mapDouble((double)pwmValue, 0.00, 255.00, 10.00, fixedVoltage - 2.5);
  if (newVoltage > fixedVoltage - 2.5) newVoltage = fixedVoltage - 2.5;
  if (newVoltage < 10.00) newVoltage = 10.00;
  setVoltage(newVoltage);
}

void setWaveForm(String wf)
{
  char wfC[80];
  wf.toCharArray(wfC);
  waveFormTotalFrames = 1;
  for(int i=0;i<wf.length(); i++) {
     if (wfC[i]=',') waveFormTotalFrames++;     
  }
  for(int i=0; i<waveFormTotalFrames; i++) {
    waveFormPatterns[i] = getValue(wfC, ',', i).toInt();
  } 
}


/**
  Process serial commands
*/
void processIncoming()
{
  char command = serialBuffer[0];
  switch (command) {
    case '+' : // on
      hbTurnOn();
      controller.println("OK|+|!");
      delay(30);
      break;
    case '-' : // off
      hbTurnOff();
      controller.println("OK|-|!");
      delay(30);
      break;
    case 's' : // set speed
      setHbSpeed(getValue(serialBuffer, '|', 1).toInt());
      controller.println("OK|s|!");
      delay(30);
      break;
    case 'd' : // delay between circles
      setCycleGap(getValue(serialBuffer, '|', 1).toInt());
      controller.println("OK|d|!");
      delay(30);
      break;
    case 'v' : // set voltage where 0=10v, 255=fixedVoltage
      setVoltageByPwmValue(getValue(serialBuffer, '|', 1).toInt());
      controller.println("OK|v|!");
      delay(30);
      break;
     case 'w' : // set waveform
      if (!turnedOn) {
        setWaveForm(getValue(serialBuffer, '|', 1));
        controller.println("OK|w|!");
        delay(30);
      } else {
        controller.println("E|already in wave formation, stop to set|!");
      }
      break;
    case '?' : // handshake requested
      controller.println("OK|go|!");//here is our handshake
      delay(30);
      break;
    default:
      break;
  }
}



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
//    switch (segment) {
//      case 0 ... 2 :  positive.on(); // + pulse
//        break;
//      case 3 : positive.off();
//        break;
//      case 4 ... 6: negitive.on();// - pulse
//        break;
//      case 7 ... 99: negitive.off();
//        break;
//    }
//
//    to do waveFormPatterns in this shit!!!!
//
//    segment++;
//    if (segment > maxSegments) {
//      segment = 0;
//    }
//  }
//}

ISR(TIMER1_OVF_vect)
{
  TCNT1 = timer1_counter;   // preload timer
  if (!turnedOn && !turnedOff) {

    positive.off();
    negitive.off();
    waveFormFramePlayingNow = 0;
    turnedOff = true;
    led.off();

  } else if (turnedOn) {

    switch (waveFormPatterns[waveFormFramePlayingNow]) {
       case 1 : positive.on();
        break;
      case 2: negitive.on();// - pulse
        break;
       default : if ( positive.getIsOn()) positive.off(); 
                else negitive.off();
       }

   waveFormFramePlayingNow++;
   if (waveFormFramePlayingNow>=waveFormTotalFrames) {
    waveFormFramePlayingNow = 0;
   }
    
  }
}


/**
  Interrupt service routine for basic sq wave form with gaps to allow mosfets to close due to back emf from coil

  1 cycle consists of 16 fractions of equal time:
  direction:   [+][+][+][+][+][+][0][0][-][-][- ][- ][- ][- ][0 ][0 ]
  pos:         [0][1][2][3][4][5][6][7][8][9][10][11][12][13][14][15]
  we modify th gap at end of each cycle to control power
  wave form     | |_   _| |_   _| |_   _[_(*X)]
                    |_|     |_|     |_|
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
//      case 0 ... 5: positive.on(); // + pulse
//          break;
//      case 6 ... 7: positive.off();
//          break;
//      case 8 ... 13: negitive.on();// - pulse
//          break;
//      case 14 ... 15: negitive.off();
//          break;
//
//    }
//
//   segment++;
//   if (segment>15) { segment = 0;
//        //delau start of necxt circle
//   }
//
//
//  }
//}

/**
  Interrupt service routine for basic 2/2 pulse sq wave form

  1 cycle consists of 10 fractions of equal time:
  direction:   [+][0][+][0][0][-][0][-][0][0]
  pos:         [0][1][2][3][4][5][6][7][8][9]

  wave form  |_|__ _ __
                  | |
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
  Switch on H-Bridge
*/
void hbTurnOn()
{
  if (!turnedOn) {
    led.on();
    turnedOn = true;
    turnedOff = false;
    Serial.println(":04so1\r\n");
    //Serial.println("01su1500");
    //Serial.println(":01su1500");
  }
}

/**
  Inform H-Bridge interrupt to turn off H-Bridge
*/
void hbTurnOff()
{
  if (!turnedOff) {
    turnedOn = false;
    Serial.println(":04so0\r\n");
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
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  //stepdown.begin(9600);

  controller.begin(11000);

  delay(30);
  //let the main logger/controller know we are here and ready to start dancing (when required)
  controller.println("OK|go|!");
  led.off();
  Serial.println(":04so0\r\n");
  delay(60);
  setVoltageByPercentage(0);
  //Serial.println(":04su1000\r\n");
  delay(60);
  Serial.println(":04si1000\r\n");
}

void loop()
{
  readCurrent();
  scanForIncoming();
  manageFan();
}
