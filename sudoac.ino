/***
   _____             _                    _____
  / ____|           | |            /\    / ____|
  | (___   _   _   __| |  ___      /  \  | |
  \___ \ | | | | / _` | / _ \    / /\ \ | |
  ____) || |_| || (_| || (_) |  / ____ \| |____
  |_____/  \__,_| \__,_| \___/  /_/    \_\\_____| 0.2

  Free Phases AC sq wave simulator with current sense and power control via TTL
  Voltage controlled by stepdown module via TTL serial 

  For the Public domain!

  Your free to criticise and free to reuse whatever you want - Have Fun!

  This is for use with Lenr Logger/Pid, see:
  https://github.com/freephases/lenr-logger2

  Uses:
  Mini pro 5v
  Stepdown module with serial connection:
   - DPS-6015A - 900W 0-60V 15A Programmable DC-DC Step-down Switch Power Supply Converter M0F0
     https://www.dropbox.com/s/wl114xaxgbhkgfc/E1600%20User%27s%20manual%20of%20DPS-6015A.pdf?dl=0
     (had to traslate serial protocal doc to english, ask me for it if you want it)
     http://r.ebay.com/Zwn4K1

  Uses main Serial (0) for Stepdown module so have to unplug wires if uploading to mini pro
  Could not get software serial to read more then one port even when using lisern due to speed i think
  Software serial is being used for comms with host controllor (the LENR legger/pid)

   Copyright (c) 2015-2017 free phases research

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
**/

//on/off class see: https://github.com/freephases/arduino-onoff-lib
#include <OnOff.h>
#include <SoftwareSerial.h>
#define DEBUG_TO_SERIAL 0

/**
  Max length of serial data we want to read as a single resquest/command
*/
#define MAX_SERIAL_DATA_LENGTH 54

/**
  Mosfet control for H-Bridge heatsink fan, it's 12v so this just switching mosfet
*/
OnOff fan(11);

/**
  H-Bridge normal
*/
OnOff hbNormal(6);

/**
  H-Bridge inverted
*/
OnOff hbInverted(5);

/**
  Led flashes when H-Bridge is on
*/
OnOff led(13);

/**
   step down
*/
//SoftwareSerial stepdown(2, 9);//cannot use with Controller software serial at the mo!

/**
  Controller serial
*/
SoftwareSerial controller(12, 3);

/**
  16MHz/256 timer time
*/
volatile long timer1_counter;


/**
  Interval between reading current sensor in millis
*/
const unsigned long currentReadMillisInterval = 1000;

/**
  Last time current was read in millis
*/
unsigned long lastCurrentReadMillis = 0;

/**
  DC input voltage
*/
const float fixedVoltage = 50.0;//as of 13-12-2016, no get voltage from stepdown mod!!

/**
  Average DC input Amps
*/
float averageAmps = 0.0;
//float totalAverageValues = 0.0;
/**
  Power in Watts from stepdown module serial
*/
float watts = 0.0;

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

int lastHbSpeed = 0;
double stepdownLastVoltage = 0.00;//last voltage set via serial, so we do not repete ;)

const float powerSupplyMaxWatts = 500.00;// is 400 but say 390 to be on the safe side, that is if the fixedVoltage var specified above is correct for given PSU
/**
  turnedOn - true if not yet fully turned off
  turnedOff - true if fully turned off
  Need to redo. thsi was to make things a bit safer, it allows for interupt to turn bridge off not serial within serial request in case interupt was about to turn on a mosfet
*/
volatile boolean turnedOn = false;//used t tell system to turn on or off does not mean it is off
volatile boolean turnedOff = true;//if ture then we are off, see you on line 153...

/*
 * step down vars
 */
char stepdownInChar;
char stepdownSerialBuf[MAX_SERIAL_DATA_LENGTH];
int stepdownBufPos = 0;
float stepdownVoltage = 0.000;

/**
 * Max number of waveFormPatterns sent over serial and therefore in our array
 */
#define LL_WF_MAX_PATS 40
/**
   waveFormPatterns, up to LL_WF_MAX_PATS, where each value is equal to one
   segment of the timer call based on current speed/frequency set, see setHbSpeed

   1 = (+)(-) normal side on h-bridge
   2 = (-)(+) inverted to first side
   0 = open/off (or anything not a 1 or 2)

   There must be a 0 always before a 1 or a 2. We always start as off (0) so starting with 0 will just delay the start!

    Good Examples:
               _
    1,0,2,0 = | |_   _
                  |_|
              +- 0 -+ 0
                   __
    1,1,0,2,2,0 = |  |_    _
                       |__|
                  +-  0 -+ 0 
           _
    1,0 = | |_
          +- 0
    2,0 =    _
          |_|
          -+ 0
          
   The last 2 will give normal PWM type DC, all the others are AC nd will invert direction of current
   
   Bad examples where x marks a bad cross over are:
                 ___   _
      1,2,0,1 = |   |_| |
                  |_|
                +-xxx0 +-
                     ___
      2,0,1,2 =    _|   |
                |_|   |_|
                -+ 0 +xxx

  No off means certain death for the h-bridge!
    
  1,1,0,2,2,0 is the default wave form, this is longer on (1 or 2) than off (0) type AC sq waveform [+-][+-][0][-+][-+][0]
  
  Others good ones to play with are:
  1,1,1,0,2,2,2,0 - slows freq down, can keep going repeating 1s or 2s
  1,1,0,1,1,0,2,2,0,2,2,0 - maybe closest to wave form that was suggested by someone ;)
  2,2,0,1,0 -- same as 1,1,0,2,0 except it's the other way around!
    
  DC only waveform examples:
  1 - constant +- normal dc
  2 - same as above but someone swiched the wires HA HA!
  1,0 - simple dc pwm
  2,0 - dc pwm but the other way around ;)
  
*/
short waveFormPatterns[LL_WF_MAX_PATS] = {1, 1, 0, 2, 2, 0}; 
//internally used...
short waveFormTotalFrames = 6;
volatile int waveFormFramePlayingNow = 0;

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

/**
 * Send request to get stats from stepdown (r)  
 * 
 * stats requested are: voltage (v) amps (j) and watts (w) 
 */
void requestStepdownStats()
{
  //serial for stepdown is :[channel][x]\r\n but :[channel][x]\r\n\n worked better so using println
  //[channel] is always 04 for this Sudo Ac mod,[x] is one of more actions, see processStepdownSerial for more details
  Serial.println(":04rvjw\r\n");
}


//stepdown does this, may put back later
//void checkOverLoading()
//{
//  if (!turnedOff) {
//    if (watts > powerSupplyMaxWatts) {
//      hbTurnOff(); //force stop, using way to much power for way to long
//      controller.println("E|!|HA|!"); //High Amps message, have to reboot to get out of this, it should not happen if you have calculated coil ohm's correctly.
//    }
//  }
//}

void readCurrent()
{
  if (millis() - lastCurrentReadMillis > currentReadMillisInterval) {
    lastCurrentReadMillis = millis();
    requestStepdownStats();
    //checkOverLoading();//stepdown does this
    if (!turnedOff) {
      //flash led so that human knows we are alive
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

/**
 * Process data returned from the stepdown mod
 */
void processStepdownSerial()
{
  if (stepdownSerialBuf[0] != ':') return; //not valid, ignore

  if (strlen(stepdownSerialBuf) > 5 && stepdownSerialBuf[3] == 'r') {
    stepdownSerialBuf[strlen(stepdownSerialBuf) - 1] = 0; //remove hash check

    String rStr = String(stepdownSerialBuf).substring(5);

    switch (stepdownSerialBuf[4]) {
      case 'v' : // voltage
        stepdownVoltage = float(rStr.toInt() / 100.0);
        break;

      case 'j' : // current / amps
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

/**
 * Scan from incomming data from stepdown (sadly on main serial port)
 */
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
  Set speed of h-bridge 0 is around 25Hz AC and 255 is 2.1kHz AC with 48v input -
  last checked AC freq.: 10th March 2016
*/
void setHbSpeed(int value)
{
  if (value != lastHbSpeed) {
    long newCounter = map((long)value, 0, 255, 65225, 65531);
    lastHbSpeed = value;

    //keep in range
    if (newCounter > 65531) newCounter = 65531;
    else if (newCounter < 65225) newCounter = 65225;

    timer1_counter = newCounter;
    //  Serial.print("New counter: ");
    //  Serial.println(newCounter, DEC);
  }
}

/**
 * Set stepdown voltage
 */
void setVoltage(double newVoltage)
{
  if (stepdownLastVoltage != newVoltage) {
    stepdownLastVoltage = newVoltage;
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
}

/**
 * set step down voltage using a 0-255 range,
 */
void setVoltageByValue(int value)
{
  if (value<0) value = 0;
  else if (value>255) value = 255;
  
  double newVoltage  = mapDouble((double)value, 0.00, 255.00, 10.00, fixedVoltage - 2.5);
  if (newVoltage > fixedVoltage - 2.5) newVoltage = fixedVoltage - 2.5;
  if (newVoltage < 10.00) newVoltage = 10.00;
  
  setVoltage(newVoltage);
}

/**
   Returns false if waveform is invalid
    
   To control H-Bridge direction we use the following:
   
   0 to mean off/Zero/Open
   1 to mean (+)(-)
   2 to mean (-)(+)

   We need to make sure wave form is always 0 (open) before a 1 or 2 (closed) 
   i.e: 1,2,0,1 is bad while 1,0,2,0 is good
   
*/
boolean validateWaveForm()
{
  if (waveFormPatterns[waveFormTotalFrames - 1] != 0 && waveFormPatterns[0] != 0) {
    //need to zero somewhere, must be at end or at start, end is better!
    return false;
  }

  boolean onOne = false;
  boolean onTwo = false;
  for(int replay=0; replay<2; replay++) {
    //check wave form twice, but on 2nd time we only go up to next Zero
    for (int i = 0; i < waveFormTotalFrames; i++) {
      if (waveFormPatterns[i] == 1) { 
        // hbNormal
        onOne = true;
      }
      else if (waveFormPatterns[i] == 2) { 
        //hbInverted
        onTwo = true;
      } else { 
        //anything else means off - zero, open, nothing! - this is very good!
        onOne = false;
        onTwo = false;
        if (replay==1) {
          break; //break out, checked what we need
        }
      }
  
      if (onOne && onTwo) {
        //can't have both on without going to Zero, return false ERROR!
        return false; 
      }
    }
  }

  return true;
}

/**
 * Set the wave form to memory
 */
void setWaveForm(String wf)
{
  char wfC[80];
  wf.toCharArray(wfC, 80);
  waveFormTotalFrames = 1;
  for (int i = 0; i < wf.length(); i++) {
    if (wfC[i] = ',') waveFormTotalFrames++;
  }
  for (int i = 0; i < waveFormTotalFrames; i++) {
    waveFormPatterns[i] = getValue(wfC, ',', i).toInt();
  }
}

/**
 * Display current wave form to host controller
 */
void displayWaveForm()
{
  String wfStr(waveFormPatterns[0]);

  for (int i = 1; i < waveFormTotalFrames; i++) {
    wfStr.concat(",");
    wfStr.concat(waveFormPatterns[i]);
  }

  controller.println("OK|W|" + wfStr);
}

/**
  Process incomming serial commands from the host/controller

  Commands are:
  
  + Power On
  - Off
  s Set speed
  v Set voltage where 0=10v, 255=whatever fixedVoltage is
  w Set wave form csv
  W Display current wave form
  ? display handshake (OK|go)
  
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
      //setCycleGap(getValue(serialBuffer, '|', 1).toInt());
      controller.println("E|d|not supported now|!");
      delay(30);
      break;
    case 'v' : // set voltage where 0=10v, 255=fixedVoltage
      setVoltageByValue(getValue(serialBuffer, '|', 1).toInt());
      controller.println("OK|v|!");
      delay(30);
      break;
    case 'w' : // set waveform if off and wave form is valid/safe to use
      if (!turnedOn) {
        if (validateWaveForm()) {
          setWaveForm(getValue(serialBuffer, '|', 1));
          controller.println("OK|w|" + getValue(serialBuffer, '|', 1) + "|!");
          delay(30);
        } else {
          controller.println("E|w|bad wave form, cannot use|!");
        }
      } else {
        controller.println("E|w|running, stop to set wave form|!");
      }
      break;
    case 'W' : // display current waveform
      displayWaveForm();
      break;
    case '?' : // handshake requested, do somehting back so they know we are alive and happy!
      controller.println("OK|go|!");//here is our handshake you smuck!
      delay(30);
      break;
    default:
      break;
  }
}

/**
  Read known incoming serial data from controller in to char array
  to then process
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


/**
 * The timer that does the buiness switching the H-Bridge 
 */
ISR(TIMER1_OVF_vect)
{
  TCNT1 = timer1_counter;   // preload timer for next time
  if (!turnedOn && !turnedOff) {
    hbNormal.off();
    hbInverted.off();
    waveFormFramePlayingNow = 0;
    turnedOff = true;
    led.off();
  } else if (turnedOn) { 
    switch (waveFormPatterns[waveFormFramePlayingNow]) {
      case 1 :
        hbNormal.on();//forwards (+ -)
        break;
      case 2:
        hbInverted.on();//backwards (- +)
        break;
      default :   //anything else turn off
        hbNormal.off();
        hbInverted.off();
    }
    waveFormFramePlayingNow++;
    if (waveFormFramePlayingNow >= waveFormTotalFrames) {
      waveFormFramePlayingNow = 0;
    }

  }
}


/**
  Switch on H-Bridge
*/
void hbTurnOn()
{
  if (!validateWaveForm()) {
    controller.println("E|!|Bad waveform cannot run|!");
    return;
  }

  if (!turnedOn) {
    led.on();
    turnedOn = true;
    turnedOff = false;
    Serial.println(":04so1\r\n");//turn on step down mod
  }
}

/**
  Inform H-Bridge interrupt to turn off H-Bridge and make power open on step down
*/
void hbTurnOff()
{
  if (!turnedOff) {
    turnedOn = false;
    Serial.println(":04so0\r\n");//turn off step down
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

  controller.begin(11000);

  delay(30);
  //let the main logger/controller know we are here and ready to start dancing (when required)
  controller.println("OK|go|!");
  led.off();
  Serial.println(":04so0\r\n");
  delay(60);
  setVoltageByValue(0);
  //Serial.println(":04su1000\r\n");//su = set voltage onstep down, seting to 10v
  delay(60);
  Serial.println(":04si0800\r\n");//si=set max amps, setting it here to 8 amps
}

void loop()
{
  readCurrent();
  scanForIncoming();
  manageFan();
}
