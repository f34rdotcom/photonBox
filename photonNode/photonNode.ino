/******************************************************************************\
 * 
 * MODULE: PhotonNode.c
 * 
 * Copyright (c) 2018 Sean Mathews <coder@f34r.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * 
 * Notes:
 *  This sketch assumes the Moteino inside the SwitchMote has been configured
 *  using the one time SwitchMoteConfig sketch before it has been installed. 
 *  See this link for details: http://github.com/lowpowerlab/SwitchMote
 *
 \******************************************************************************/
#include <EEPROMex.h>      //get it here: http://playground.arduino.cc/Code/EEPROMex
#include <RFM69.h>         //get it here: http://github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming
#include <SPI.h>           //comes with Arduino
#include <TimerOne.h>
#include <Wire.h>
#include <RTClib.h>
#include <avr/wdt.h>
#include <Time.h>
#include <Timezone.h>    //https://github.com/JChristensen/Timezone
#include <MemoryFree.h>
///#include <avr/pgmspace.h>

// *************************************************************************************************************
// hard coded settings
#define GATEWAY_ID             1 // address of the node that is our gateway
#define BUTTON_BOUNCE_MS     200 //timespan before another button change can occur
#define SERIAL_EN                //serial debugging
#define LED_PERIOD_ERROR    1000 //led indicator error duration
#define LED_PERIOD_OK      10000 //led indicator OK duration
#define DS3231_I2C_ADDRESS   104

// enable radio code
//#define RADIO

// enable RTC
#define RTC

// enable PUMPS
#define PUMPS

// enable LIGHTS
#define LIGHTS

// tactile control buttons
//#define BUTTONS

// DEBUG button bounce code
//#define DEBUG_BTN

// PWM FAN CONTROLLER
#define PWMFAN

// reset trick by turning on WDT.
#define Reset_AVR() wdt_enable(WDTO_30MS); while(1) {}


#ifdef SERIAL_EN
  #define SERIAL_BAUD    115200
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

/* FAST DIGITAL IO FUNCTIONS */
#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

// pin hardware state routines
#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

// pin hardware abstraction
#define HWLED_ACT           9  // digital pin for activity LED 15 on Arduino Uno 13 for Moteino

#ifdef LIGHTS
#define HWRELAY1            7  // D7 digital pin connected to Relay 1
#define HWRELAY2            3  // D3 digital pin connected to Relay 2
#endif

#ifdef BUTTONS
#define HWBTN1              A2 // A2 pin of manual button 1
#define HWBTN2              A3 // A3 pin of manual button 2
#endif

#ifdef PWMFAN
#define HWPWM1              10 // pwm PIN for fan control
#endif

#ifdef PUMPS
#define HWPUMPFILL          A2  // A2 to fill relay control pin
#define HWPUMPDRAIN         A3  // A3 to drain relay control pin
#endif

// constants
#define ON                  1
#define OFF                 0
#define PRESSED             0
#define RELEASED            1

// radio configuration
struct configuration {
  byte frequency;
  byte isHW;
  byte nodeID;
  byte networkID;
  char encryptionKey[16];
  byte null1;      // separators needed to keep strings from overlapping
  byte start_hour; // 0-23
  byte start_min;  // 0-59
  byte end_hour;   // 0-23
  byte end_min;    // 0-59
} CONFIG;

#ifdef LIGHTS
// internal relay state ID's
enum RELAYS {
    RELAY1 = 1,
    RELAY2
};
#endif

#ifdef PUMPS
// internal relay state ID's
enum PUMP_STATES {
    PUMP_OFF = 0,
    PUMP_FILL,
    PUMP_DRAIN
};
#define PUMP_BOOT_DRAIN_TIME 5 * 60
#define PUMP_DRAIN_TIME 20 * 60
#define PUMP_FILL_TIME  40 * 60 
//#define PUMP_DRAIN_TIME 5 * 60
//#define PUMP_FILL_TIME  10 * 60
#endif

// *************************************************************************************************************
// forward decls
void action(byte whichDevice, byte whatState, boolean notifyNetwork=true);

// *************************************************************************************************************
// Globals
RTC_DS3231 rtc;
RFM69 radio;
SPIFlash flash(8, 0xEF30);     //FLASH MEM CS pin is wired to Moteino D8
time_t tnow=0;
time_t last=0;
byte inputLen=0;
char tbuffer[64];

int rtcOK         = false;

#ifdef BUTTONS
int btn1State     = RELEASED;
int btn2State     = RELEASED;

int btn1LastState = RELEASED;
int btn2LastState = RELEASED;

unsigned long btn1LastPress = 0;
unsigned long btn2LastPress = 0;
#endif

#ifdef LIGHTS
int relay1State   = ON;
int relay2State   = ON;
#endif

#ifdef PUMPS
uint8_t previousHour;
uint8_t pumpState = PUMP_OFF;
uint8_t lastPumpState = 99;
uint8_t nextPumpState = 0;
unsigned long pumpNextTime;
#endif


//US Pacific Time Zone (Los Angeles)
const TimeChangeRule myDST = {"PDT", Second, Sun, Mar, 2, -420};    //Daylight time = UTC - 4 hours
const TimeChangeRule mySTD = {"PST", First, Sun, Nov, 2, -480};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

// *************************************************************************************************************
// called during chip boot
void setup(void)
{

  // set LED to output
  pinMode(HWLED_ACT, OUTPUT);

#ifdef LIGHTS
  // set relay pins to output
  pinMode(HWRELAY1, OUTPUT);
  pinMode(HWRELAY2, OUTPUT);
#endif

#ifdef BUTTONS
  // by writing HIGH while in INPUT mode, the internal pullup is activated
  // the button will read 1 when RELEASED (because of the pullup)
  // the button will read 0 when PRESSED (because it's shorted to GND)
  pinMode(HWBTN1, INPUT);digitalWrite(HWBTN1, HIGH); //activate pullup
  pinMode(HWBTN2, INPUT);digitalWrite(HWBTN2, HIGH); //activate pullup
#endif

#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif
  DEBUGln();
  DEBUGln("!DBG: setup()");

#ifdef RTC
  DEBUG("!DBG: rtc[");
  if (rtc.begin()) {
    DEBUG("OK");
    rtcOK = true;
  } else {
    DEBUG("not found");
  }
  DEBUGln("]");

  if (rtcOK) {
    if (rtc.lostPower()) {
      DEBUGln("!DBG: rtc lost power. Date and time lost.");
      rtc.adjust(DateTime(2016, 10, 26, 4, 20, 0));
    }
    
    setSyncProvider(getRTCTime);   // the function to get the time from the RTC
    if (timeStatus()!= timeSet) 
      Serial.println("!DBG: Unable to sync with the RTC");
    else
      Serial.println("!DBG: RTC has set the system time"); 

    TimeChangeRule *tcr; 
    tnow = myTZ.toLocal(now(), &tcr);
    senddate(tnow,tcr -> abbrev);
  }
#endif

  Wire.begin();
  
  // read radio settings from eeprom
  DEBUGln("!DBG: Reading configuration");
  EEPROM.readBlock(0, CONFIG);
  
  // not configured? expected [4,8,9]
  if (CONFIG.frequency!=RF69_433MHZ && CONFIG.frequency!=RF69_868MHZ && CONFIG.frequency!=RF69_915MHZ)
  {
    DEBUGln("!DBG: Not configured. No valid config found in EEPROM.");
  } else {
    DEBUG("!DBG: FREQENCY:");DEBUGln(CONFIG.frequency);    
    DEBUG("!DBG: isHW:");DEBUGln(CONFIG.isHW);    
    DEBUG("!DBG: NETWORKID:");DEBUGln(CONFIG.networkID);    
    DEBUG("!DBG: NODEID:");DEBUGln(CONFIG.nodeID);
    DEBUG("!DBG: EncryptKey:");DEBUGln(CONFIG.encryptionKey);
    // initialize the radio and put it to sleep
    radio.initialize(CONFIG.frequency,CONFIG.nodeID,CONFIG.networkID);
    DEBUGln("!DBG: radio.initialize()");
    // boost power level if supported
    //radio.setPowerLevel(10);
    if (CONFIG.isHW) {
      DEBUG("!DBG: setHighPower()");
      radio.setHighPower(); //use with RFM69HW ONLY!
      DEBUGln("..done");
    }
    
    if (CONFIG.encryptionKey[0]!=0) {
      DEBUG("!DBG: encryptionKey(");
      DEBUG(CONFIG.encryptionKey);      
      radio.encrypt(CONFIG.encryptionKey);
      DEBUGln(")..done");      
    }
    
    DEBUGln("!DBG: radio.sleep()");
    radio.sleep();
    
    DEBUGln("!DBG: radio init done");
  }
#ifdef RADIO


#else
  DEBUGln("!DBG: radio disabled.");
#endif

#ifdef PUMPS
  previousHour = rtc.now().hour();
  // set PUMP 1 pin to low to turn off the FET
  //pinMode(HWPUMPFILL, INPUT_PULLUP);
  pinMode(HWPUMPFILL, OUTPUT);
  digitalWrite(HWPUMPFILL, LOW);

  // set PUMP 2 pin to low to turn off the fet
  //pinMode(HWPUMPDRAIN, INPUT_PULLUP);
  pinMode(HWPUMPDRAIN, OUTPUT);
  digitalWrite(HWPUMPDRAIN, LOW);
#endif

#ifdef LIGHTS
  // load from FLASH the buttons last known state
  // fixme for now just default init() state
  action(RELAY1, relay1State, false);
  action(RELAY2, relay2State, false);
#endif

#ifdef PWMFAN
  // start PWM for fan controller
  DEBUGln("!DBG: PWM init");
  pinMode(HWPWM1, OUTPUT);
  Timer1.initialize(40);                // initialize timer1, and set a 25khz period
  Timer1.pwm(HWPWM1, 1024);                  // slowest speed ( inverted because of level shifting )
  Timer1.attachInterrupt(PWMcallback);  // attaches callback() as a timer overflow interrupt
#endif

  // wait 1/2 second to settle the radio and hardware
  delay(500);

  // blink indicate ready
  blinkLED(HWLED_ACT,LED_PERIOD_ERROR,LED_PERIOD_ERROR,3);

  // report the current schedule
  showSched();

#ifdef PUMP  
  // drain at boot just to be safe if we died while filling
  lastPumpState = pumpState = PUMP_DRAIN;
  nextPumpState = PUMP_OFF;
  pumpNextTime = tnow + PUMP_BOOT_DRAIN_TIME;
#ifdef VERBOSE
  DEBUG("!DBG: PUMP[");
  DEBUG(pumpNextTime);
  DEBUG(",");      
  DEBUG(lastPumpState);
  DEBUG(",");
  DEBUG(pumpState);
  DEBUGln("]");
#endif  
#endif

  // Report ready  
  DEBUGln("!DBG: Listening for commands...");
  
}

// *************************************************************************************************************
// PWM interrupt callback routine
void PWMcallback()
{
  digitalLow(9);  
}


// *************************************************************************************************************
// service routine called every N
void loop()
{

  // grab local time from our UTC global time now()
  TimeChangeRule *tcr; 
  time_t tnow = myTZ.toLocal(now(), &tcr);
      
  // blink indicate ready. 0 timer so interruptable
  blinkLEDNB(HWLED_ACT,LED_PERIOD_OK,LED_PERIOD_OK,0);

#ifdef BUTTONS
  // read our button pin states
  btn1State = digitalRead(HWBTN1);
  btn2State = digitalRead(HWBTN2);
#endif

  // found in WirelessHEX.cpp blocks for 1 second(ish)
  if (Serial.available()) 
  {
    inputLen = readSerialLine((char *)tbuffer);
    tbuffer[inputLen]=0;

    DEBUG("!DBG: SER:"); DEBUG(inputLen); 
    DEBUG(",'");
    for(int a = 0; a < inputLen; a++) {
      Serial.print(tbuffer[a]);
    }
    DEBUGln("'");
    
    if (inputLen > 0)
    {
      byte colonIndex;    
      // reboot?
      if (tbuffer[0] == '=') {
        DEBUGln("!DBG: RESET()");
        Reset_AVR();
      }

      //
      // CMD:RESET
      // CMD:FREQ:9
      // CMD:NETWORKID:42
      // CMD:NODEID:1
      // CMD:KEY:AABBCCDDEEFFAABBCCCCCCCCCCCC
      // CMD:DATE:1501817210
      // CMD:SCHED:13:15,23:10
      // CMD:SCHED:19:05,07:05
      // CMD:SCHED:19:05,13:05
      //
      // Get the current key ( remove for production danger low hanging fruit )
      // CMD:KEY?
      //
      // Set te current 16 byte encryption key
      // CMD:KEY:AABBCCDDEEFFAABB
      //          Option  16 Byte key
      //
      // Set this nodes ID
      // CMD:NODEID:123
      //          Options 1-255
      //
      // Set this nodes network membership 
      // CMD:NETWORKID:
      //          Options 0-255
      // 
      // Set this nodes frequency depending on actual radio being used.
      // CMD:FREQ:9
      //          Options 9:915mhz, 8:868mhz, 4:433mhz 
      // 
      // Set this nodes date / time by unix time UTC
      // CMD:DATE:NNNNNNNNN
      //          on linux to get utc time >date +%s 
      //
      // Set the start and end time
      // CMD:SCHED:HH:MM,HH:MM
      //           ON   ,OFF time
      //           HH is 0 - 23 MM is 00 - 59
      // Example: CMD:SHED:13:15,23:10
      //           Turn ON at 1:15PM and back off at 11:10PM
      
      // Extract the first part. The directive
      char *d = strtok(tbuffer,":");
      
      DEBUG("!DBG: DIR:"); DEBUGln(d);
      
      if (strcmp(d, "CMD") == 0)
      {
        // grab the command removing prefix CMD:
        char *c = strtok(null,":");
        char *a = strtok(null,"");
        
        DEBUG("!DBG: CMD[");
        DEBUG(c);
        DEBUG("], [");
        DEBUG(a);
        DEBUGln("]");
        
        // Get the current date time human readable with time zone
        if (strcmp(c, "DATE?") == 0) {
          senddate(tnow,tcr -> abbrev);
        }
  

        // Set the RTC current time based upon UTC unix time
        if (strcmp(c, "DATE") == 0) {
          TimeChangeRule *tcr; 
          // grab the unix time in UTC and save to rtc clock chip.
          uint32_t t = atol(a);

          // was this a valid number?
          if (t>0) {
            rtc.adjust(DateTime(t));
            // and update local time         
            setTime(rtc.now().unixtime());
          }
          
          // echo back current date / time
          senddate(tnow,tcr -> abbrev);          
        }

        // Display current SCHEDULE value into EEPROM
        if (strcmp(c, "SCHED?") == 0) {
          showSched();
        }
        
        // Set the current SCHEDULE value into EEPROM
        if (strcmp(c, "SCHED") == 0) {
          // a = '13:15,23:10'
          char *tt = strtok(a,":");
          byte SH = atoi(tt);
          byte SM = atoi(strtok(null,","));
          
          byte EH = atoi(strtok(null,":"));
          byte EM = atoi(strtok(null,""));

          if (SH < 24 && SM < 60 && EH < 24 && EM < 60)
          {
              CONFIG.start_hour = SH; // 0-23
              CONFIG.start_min  = SM; // 0-59
              CONFIG.end_hour   = EH; // 0-23
              CONFIG.end_min    = EM; // 0-59

              // the data is all good lets save it
              EEPROM.writeBlock(0, CONFIG);
          }
          
          // echo back the new schedule
          showSched();
        }

        // Reset CONFIG eeprom to factory 
        if (strcmp(c, "RESET") == 0) {
          memset(&CONFIG,0,sizeof(CONFIG));
          EEPROM.writeBlock(0, CONFIG);
          DEBUGln("!DBG: RESET");
        }
          
        // Get the current KEY value from EEPROM
        // DANGER! Remove this low handing fruit when done testing code      
        if (strcmp(c, "KEY?") == 0) {
          // echo back the current key
          DEBUG("!DBG: EncryptKey:");DEBUGln(CONFIG.encryptionKey);        
        }
  
        // Set the current KEY value into EEPROM
        if (strcmp(c, "KEY") == 0) {
          int keylen = strlen(a);
          int kx;
          for(kx = 0; kx < keylen && kx < 16; kx++) {
            CONFIG.encryptionKey[kx] = a[kx];
          }
          
          // the data is all good lets save it
          EEPROM.writeBlock(0, CONFIG);
          
          // echo back the new key   
          DEBUG("!DBG: EncryptKey:");DEBUGln(CONFIG.encryptionKey);        
        }
  
        // Set the current NETWORKID value into EEPROM
        if (strcmp(c, "NETWORKID") == 0) {
          
          int c = atoi(a);
          if (c <= 255) {
            CONFIG.networkID = c;
            
            // the data is all good lets save it            
            EEPROM.writeBlock(0, CONFIG);          
          }
          
          // echo back the current network id
          DEBUG("!DBG: NETWORKID:");DEBUGln(CONFIG.networkID);        
        }
  
        // Set the current NODEID value into EEPROM
        if (strcmp(c, "NODEID") == 0) {
          
          int c = atoi(a);
          if (c > 0 && c <= 255) {
            CONFIG.nodeID = c;
            
            // the data is all good lets save it
            EEPROM.writeBlock(0, CONFIG);
          }

          // echo back the current network id
          DEBUG("!DBG: NODEID:");DEBUGln(CONFIG.nodeID);
        }
        
        // Set the current FREQ value into EEPROM
        if (strcmp(c, "FREQ") == 0) {        
          byte oldFreq = CONFIG.frequency;
          int c = atoi(a);
         
          switch(c)
          {
            case 4: CONFIG.frequency = RF69_433MHZ; break;
            case 8: CONFIG.frequency = RF69_868MHZ; break;
            case 9: CONFIG.frequency = RF69_915MHZ; break;
          }
          
          // changed? save is needed.
          if (oldFreq != CONFIG.frequency) {
            // the data is all good lets save it
            EEPROM.writeBlock(0, CONFIG);
          }
          
          // echo back the current radio frequency
          DEBUG("!DBG: FREQUENCY:");DEBUGln(CONFIG.frequency);
        }
        
        // all done
        return;
      }

      byte targetId = atoi(d); //extract ID if any
      // grab the rest of the string
      char *a = strtok(null,"");
      int msgLen = strlen(a);
      
      if (targetId > 0 && targetId != CONFIG.nodeID && targetId != RF69_BROADCAST_ADDR && colonIndex>0 && colonIndex<4 && msgLen>0)
      {
        
        // it is me sending 
        parsecommand(CONFIG.networkID, a);
        
  #ifdef RADIO      
        if (radio.sendWithRetry(targetId, a, strlen(a)))
        {
          DEBUGln("ACK:OK");
        }
        else
          DEBUGln("ACK:NOK");
  #endif        
      }
    }
  } // serial available

  // Every N seconds kick states
  if (tnow - last > 10) 
  {
    last = tnow;
    
    freeRam();
    float tempC = get3231Temp();
    DEBUG("!DBG: TEMP[");
    DEBUG(tnow);
    DEBUG(",");
    DEBUG(tempC);
    DEBUGln("]");

#ifdef RADIO      

    // notify network of our SPD state
    sprintf(tbuffer, "%d:TEMP:%d.%02d", CONFIG.nodeID, (int)tempC, (int)(tempC * 100.0) % 100);
    DEBUG("!DBG: TX '");
    DEBUG(tbuffer);
    DEBUGln("'");
    if (radio.sendWithRetry(GATEWAY_ID, tbuffer, strlen(tbuffer)))
      {DEBUGln("!DBG: RADIO TX[OK]");}
    else {DEBUGln("!DBG: RADIO TX[NOK]");}
#endif

#ifdef PUMPS
    // only start pumps of already off
    if (pumpState == PUMP_OFF) {    
      // every EVEN hour start pump cycle
      if (rtc.now().hour() != previousHour) {
        previousHour = rtc.now().hour();
        if (previousHour % 2 == 0) {
          pumpState = PUMP_FILL;
        }
      }
    }
    
    // IDLE STATE
    if (pumpState == PUMP_OFF) {
       // fill pump off
       digitalWrite(HWPUMPFILL, LOW);
       // drain pump off
       digitalWrite(HWPUMPDRAIN, LOW);
    }
    
    // FILL STATE
    if (pumpState == PUMP_FILL) {
       // fill pump ON
       digitalWrite(HWPUMPFILL, HIGH);
       // drain pump on
       digitalWrite(HWPUMPDRAIN, HIGH);
       // set next state and end time
       if (pumpState != lastPumpState) {
         pumpNextTime = tnow + PUMP_FILL_TIME;         
         nextPumpState = PUMP_DRAIN;
       }
    }
    
    // DRAIN STATE
    if (pumpState == PUMP_DRAIN) {
       // fill pump off
       digitalWrite(HWPUMPFILL, LOW);
       // drain pump on
       digitalWrite(HWPUMPDRAIN, HIGH);
       // set next state and end time
       if (pumpState != lastPumpState) {
         pumpNextTime = tnow + PUMP_DRAIN_TIME;         
         nextPumpState = PUMP_OFF;
       }
    }
    
    // pump state change report it
    if (pumpState != lastPumpState) {
      DEBUG("!DBG: PUMP[");
      DEBUG(pumpNextTime);
      DEBUG(",");      
      DEBUG(lastPumpState);
      DEBUG(",");
      DEBUG(pumpState);
      DEBUGln("]");

      lastPumpState = pumpState;
#ifdef RADIO  
      // notify network of our REL state change
      sprintf(tbuffer, "PUMP:%d", pumpState);
      if (radio.sendWithRetry(GATEWAY_ID, tbuffer, strlen(tbuffer)))
        {DEBUGln("!DBG: RADIO TX[OK]");}
      else {DEBUGln("!DBG: RADIO TX[NOK]");}
#endif 
    } else {
      // same state check if we timeout and change state
      if (pumpNextTime < tnow) {
#ifdef VERBOSE        
        DEBUG("!DBG: PUMP STATE TIMEOUT[");
        DEBUG("[");
        DEBUG(tnow);
        DEBUGln("]");
#endif        
        pumpState = nextPumpState;
      }
    }
#endif

#ifdef LIGHTS
    DEBUGln("!DBG: LIGHTS TEST");
    
    // check if the RELAY should be on or off depending on schedule
    // and the current time.
    

    // is our start time before current time?
    // If so we need the RELAY to be ON
    // get start and stop times
    int startT = (CONFIG.start_hour * 60) + CONFIG.start_min;
    int endT = (CONFIG.end_hour * 60) + CONFIG.end_min;
    int actT = (hour(tnow) * 60) + minute(tnow);
    DEBUG("!DBG: TIMER SECONDS START:"); 
    DEBUG(startT); DEBUG(" END:");
    DEBUG(endT); DEBUG(" ACTUAL:");
    DEBUG(actT); DEBUGln();

    // TIMER SECONDS START:795 END:1390 ACTUAL:1422
    // TIMER SECONDS START:1390 END:780 ACTUAL:554

    // if endT == startT then lights are ON 24/7
    if (endT == startT) {
      if (relay2State != ON) { 
        // turn it on
        action(RELAY2, ON);
      }      
    } else {

      // I cant brain today I have the dumbs. 
      // An alg to <>><!- my way around this exists...
      
      // 0-1439 or 1440 test MAX
      for( int ct = 0; ct < (24 * 60 /*min/day*/); ct++) {
        int nextT = actT+1;
        if (nextT >= (24 * 60 /*min/day*/) ) nextT = 0; 

        // ok we must have been before startT so we need to be off
        if ( nextT == startT ) {
          // DEBUG("!DBG: CA:"); DEBUG(nextT); DEBUGln();
          
          if (relay2State != OFF) { 
            // Turn it on
            action(RELAY2, OFF);
          }        
          break;
        }
  
        // ok we must have been before endT so we need to be ON
        if (nextT == endT) {
          // DEBUG("!DBG: CB:"); DEBUG(nextT); DEBUGln();
          
          if (relay2State != ON) { 
            // Turn it off
            action(RELAY2, ON);
          }        
          break;
        }
  
        actT=nextT;
        //// loop back to 0 if we go past 23:59
        //if (actT >= (24 * 60 /*min/day*/) ) actT = 0; 
        
      }
    }
    
#ifdef BRAINS    
    if ((actT > startT && actT < endT) || (actT < endT && ... :( ))
    {
      DEBUGln("!DBG: RELAY2 ON");
      if (relay2State!=ON) { 
        // toggle button state
        action(RELAY2, ON);
      }      
    } else {
      DEBUGln("!DBG: RELAY2 OFF");
      if (relay2State!=OFF) { 
        // toggle button state
        action(RELAY2, OFF);
      }
    }
#endif

#endif // LIGHTS

    senddate(tnow,tcr -> abbrev);

  } // end N second state machine cycle

#ifdef BUTTONS  
#ifdef DEBUG_BTN
  if (btn1State != btn1LastState) 
  {
    DEBUG("!DBG: BTN1[");
    DEBUG(btn1State);
    DEBUG("]:");
    DEBUGln(btn1LastState);
  }
#endif

  // check state of button 1 pin
  if (btn1State != btn1LastState && tnow-btn1LastPress >= BUTTON_BOUNCE_MS) //button event happened
  {
    // update last state
    btn1LastState = btn1State;
    if (btn1State == PRESSED) btn1LastPress = tnow;    

    // if normal button press, do the RELAY/LED action
    if (btn1State == RELEASED)
    {
      // toggle button state
      action(RELAY1, relay1State==ON ? OFF : ON);
    }
  }

#ifdef DEBUG_BTN

  if (btn2State != btn2LastState) 
  {
    DEBUG("!DBG: BTN2[");
    DEBUG(btn2State);
    DEBUG("]:");
    DEBUGln(btn2LastState);
  }
#endif
  // check state of button 2 pin
  if (btn2State != btn2LastState && tnow-btn2LastPress >= BUTTON_BOUNCE_MS) //button event happened
  {
    // update last state    
    btn2LastState = btn2State;
    if (btn2State == PRESSED) btn2LastPress = tnow;    

    // if normal button press, do the RELAY/LED action
    if (btn2State == RELEASED)
    {
      // toggle button state
      action(RELAY2, relay2State==ON ? OFF : ON);
    }
  }
#endif

#ifdef RADIO
  // radio RX test
  if (radio.receiveDone())
  {
    byte senderID = radio.SENDERID;
    DEBUG("!DBG: RADIO RX[");DEBUG(senderID);DEBUG("] ");
//    for (byte i = 0; i < radio.DATALEN; i++)
//      DEBUG((char)radio.DATA[i]);
    DEBUG(" [RX_RSSI:");DEBUG(radio.RSSI);DEBUGln("]");
    
    // wireless programming token check
    // needed for wireless oil change
    //CheckForWirelessHEX(radio, flash, true, HWLED_ACT);

    if (radio.DATALEN > 0 && radio.DATALEN < 64) {
      // move the packet locally for now...
      for (byte i = 0; i < radio.DATALEN; i++)
        tbuffer[i] = radio.DATA[i];
    
      tbuffer[radio.DATALEN] = 0;

      parsecommand(senderID, tbuffer);
    }

    if (radio.ACKRequested()) //dont ACK broadcasted messages except in special circumstances (like SYNCing)
    {
      radio.sendACK();
      DEBUGln("!DBG: RADIO TX[ACK sent]");
      //delay(5);
    }
  } //radio RX test
#endif
}

// *************************************************************************************************************
// parse and execute commands
void parsecommand(byte SenderID, char* cmd) {
    
    // listen for RELx:y commands where x={1,2}, y={0,1}
    if (strlen(cmd) == 6
        && cmd[0]=='R' && cmd[1]=='E' && cmd[2]=='L' && cmd[4] == ':'
        && (cmd[3]>='0' && cmd[3]<='2') && (cmd[5]=='0' || cmd[5]=='1'))
    {
      char relIndex = cmd[3]-'0';
      action(relIndex, (cmd[5]=='1'?ON:OFF)); //senderID!=CONFIG.networkID
    }
    
    // listen for pwm speed requests: SPDX:YYY commands
    if (strlen(cmd) > 5 &&  strlen(cmd) < 9
        && cmd[0]=='S' && cmd[1]=='P' && cmd[2]=='D' && cmd[4] == ':'
        && (cmd[3]>='0' && cmd[3]<='2') && (1 /*todo validate speed 0-100 */))
    {
      
      // grab and sanatize speed value 0-100 then 
      // invert it because we have an inverting level shifter
      uint32_t speed;
      speed = atoi((const char*)&cmd[5]);
      if (speed>100) speed=100;
      
      // 0-1023
      speed = 1024 * (100-speed) / 100;
      if (speed) speed--;

      // set the new duty cycle
      Timer1.pwm(9, speed);
            
      DEBUG("!DBG: speed[");DEBUG(speed); DEBUGln("]");
#ifdef RADIO
      // notify network of our SPD state
      char pwmIndex = cmd[3]-'0';      
      sprintf(tbuffer, "SPD%d:%d", pwmIndex, speed);
      if (radio.sendWithRetry(GATEWAY_ID, tbuffer, strlen(tbuffer)))
        {DEBUGln("!DBG: RADIO TX[OK]");}
      else {DEBUGln("!DBG: RADIO TX[NOK]");}
#endif
    }

}

// *************************************************************************************************************
// simple wrapper to adjust state of DEVICES and report if needed
void action(byte whichDevice, byte whatState, boolean notifyNetwork)
{
  DEBUG("!DBG: action[");
  DEBUG(whichDevice);
  DEBUG("]:");
  DEBUGln(whatState==ON?"ON ":"OFF");

  // relay 1 action
  if (whichDevice==RELAY1) 
  {
    // update the current state    
    relay1State = whatState;
    digitalWrite(HWRELAY1, whatState == ON ? HIGH : LOW);
  }
  
  // relay 2 action
  if (whichDevice==RELAY2)
  {
    // update the current state
    relay2State = whatState;
    digitalWrite(HWRELAY2, whatState == ON ? HIGH : LOW);
  }
  
#ifdef RADIO  
  // notify network of our REL state change
  if (notifyNetwork)
  {
    sprintf(tbuffer, "REL%d:%d", whichDevice, whatState);
    if (radio.sendWithRetry(GATEWAY_ID, tbuffer, strlen(tbuffer)))
      {DEBUGln("!DBG: RADIO TX[OK]");}
    else {DEBUGln("!DBG: RADIO TX[NOK]");}
  } 
#endif  
}

// *************************************************************************************************************
// blink the LED NON blocking call
void blinkLEDNB(byte LEDpin, byte periodON, byte periodOFF, byte repeats)
{  
  // if anything changes reset state
  static byte lastLEDPin=0;
  static byte lastPeriodON=0;
  static byte lastPeriodOFF=0;
  static byte lastRepeats=0;
  //static byte currentRepeats=0;
  static byte currentState = LOW;
  static unsigned long lastLED = 0;
  static unsigned long start = 0;
  
  // get the current time
  unsigned long current = millis();
  
  // if we are in an interruptable blink state clear our start time
  if (lastRepeats==0) start = current;
  
  // do not allow changes to led state untill its time has expired
  unsigned long seconds = (current - start)/1000;
  
   
  if ((lastLEDPin    != LEDpin ||
     lastPeriodON  != periodON ||
     lastPeriodOFF != periodOFF ||
     lastRepeats   != repeats))
  {
     byte changeOK = 1;
     
     // ok if we are in repeat period we must ignore this request
     if (lastRepeats)
     {
       if (seconds < lastRepeats)
       {
         changeOK = 0;       
       }
     }
     // ok not blocking so let it change
     if (changeOK) 
     {     
       // get our current time and new states   
       lastLED       = current;
       lastPeriodON  = periodON;
       lastPeriodOFF = periodOFF;
       lastLEDPin    = LEDpin;
       lastRepeats   = repeats;
       start         = current;
     }
  }
  
  
  if (currentState == ON)
  {
    if (current - lastLED > periodON)
    {
      lastLED = current;
      // all done turn it off
      digitalWrite(LEDpin, LOW);
      currentState = OFF;
      
      // 
    }
  } else
  if (currentState == OFF)
  {
    if (current - lastLED > periodOFF)
    {
      lastLED = current;
      // all done turn it on
      digitalWrite(LEDpin, HIGH);
      currentState = ON;      
    }
  }
  
}

// *************************************************************************************************************
// blink the LED blocking call
void blinkLED(byte LEDpin, byte periodON, byte periodOFF, byte repeats)
{
  while(repeats-->0)
  {
    digitalWrite(LEDpin, HIGH);
    delay(periodON);
    digitalWrite(LEDpin, LOW);
    delay(periodOFF);
  }
}

// *************************************************************************************************************
// return the temp from the RTC clock board
float get3231Temp()
{
  
  byte tMSB, tLSB;
  float temp3231=0;

  //temp registers (11h-12h) get updated automatically every 64s
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
  if (Wire.available()) {
    tMSB = Wire.read(); //2's complement int portion
    tLSB = Wire.read(); //fraction portion
   
    temp3231 = (tMSB & B01111111); //do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ); //only care about bits 7 & 8
  }
  else {
    //oh noes, no data!
  }
   
  // now convert to Fahrenheight
  float temperatureF = (temp3231 * 9.0 / 5.0) + 32.0;   
  return temperatureF;
}

// *************************************************************************************************************
// Left pad a number with 0's and send it out serial port
void padprint(int number, int chars) {
 for (int i = pow(10, chars - 1); number < i - 1; i /= 10) Serial.print("0");
 Serial.print(number);
}

// *************************************************************************************************************
// format the date and send it out the serial port as a !INF message
void senddate(time_t t, char *tz) {
    Serial.print("!INF: DATE:");
    Serial.print(year(t), DEC);
    Serial.print('/');
    Serial.print(month(t), DEC);
    Serial.print('/');
    Serial.print(day(t), DEC);
    Serial.print(" (");
    Serial.print(dayStr(weekday(t)));
    Serial.print(") ");
    padprint(hour(t),2);
    Serial.print(':');
    padprint(minute(t),2);
    Serial.print(':');
    padprint(second(t),2);
    Serial.print(" ");
    Serial.print(tz);
    Serial.println();
}

// *************************************************************************************************************
// callback to set now() to the RTC clock time.
time_t getRTCTime()
{
  DEBUG("!DBG: getRTCTime()");  
  time_t t = rtc.now().unixtime();
  DEBUGln("");
  return t;
}

// *************************************************************************************************************
// show the amount of mem we have free for testing
int freeRam () 
{ 
  DEBUG("!DBG: MEM:"); DEBUGln(freeMemory());
}

// *************************************************************************************************************
// show the current schedule
void showSched() {
  // echo back the new schedule
  DEBUG("!INF: SCHED: "); 
  padprint(CONFIG.start_hour,2); DEBUG(":"); padprint(CONFIG.start_min,2); DEBUG(",");
  padprint(CONFIG.end_hour,2); DEBUG(":"); padprint(CONFIG.end_min,2); DEBUGln("");
}
