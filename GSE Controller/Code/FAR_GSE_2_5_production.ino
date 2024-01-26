
/*   ***********  FAR OPEN GSE Controller - 2019-2024 by Mike and Preston Brinker  ***********

NOTES:
 *  
 *    
 * V1 = Fuel line vent
 * V2 = Fuel pnuematic disconnect valve
 * V3 = Ox line vent
 * V4 = Fuel pressurization valve (haz)
 * V5 = Ox pressurization valve (haz)
 * V6 = Ox pnuematic disconnect
 * V7 = Fuel rocket vent
 * V8 = Ox rocket vent
 * 
 * 
 * NEW Codes:
 *   POX = pressure ox
 *   PFL = pressure fuel
 *   PPS = Pressure pad spare
 *   PRS = pressure rocket spare
 *   
 * COMPILE:  
 *   Compile at 396 Mhz for best performance/heat management
 *   
 * TO DO:    
 *   - fuse error handling using 
 *   
 */




  bool serialDebug = false;  // set to true to get Serial output for debugging
  bool debugMode = false;
  bool mute = false;  // silence the buzzer for testing

 // Pin configuration
  #define RXD1 0
  #define TXD1 1
  #define RXD2 7
  #define TXD2 8
  #define pin_valve1 20
  #define pin_valve2 17
  #define pin_valve3 16
  #define pin_valve4 22  //haz needs arm (was 23 in 2.0)
  #define pin_valve5 23  //haz needs arm (was 22 in 2.0)
  #define pin_valve6 14
  #define pin_valve7 41 
  #define pin_valve8 40
  #define pin_valve9 15 // unused on Rocket2 blocks
  #define pin_altAarmed 37 // Altimeter A armed
  #define pin_altBarmed 36 // Altimeter B armed
  #define pin_rocketDetect 38 // PU 
  #define pin_igniterContinuity 32 
  #define pin_continuitySwitch 31 // PU
  #define pin_led1 28 //
  #define pin_led2 29 //
  #define pin_buzzer 30 //
  #define pin_igniterFIRE 33 // SSR2 fire
  #define pin_mainValveOn 21 // Main valvue actuator on (requires Haz armed)
  #define pin_mainValveOff 39 // Main valvue actuator off
  #define pin_recoveryOff 34 // pulse off
  #define pin_recoveryOn 35 //  pulse on
  #define pin_radioSet 27
  #define pin_internalLED 13
  #define pin_ext5v 5
  #define pin_strobes 4
  #define pin_fuse5v 9
  #define pin_fuseMain 10
  #define pin_fuseHaz 11
  #define pin_fan 26
 // GND to 2,3,6,12,24,25
  
  
//Teensy
  #include "Arduino.h"
  #include <TeensyThreads.h>
  int id_sampling;
  int id_logging;


  struct configStruct {  // master configuration
     // PT ranges - assume all are .5v-4.5v 
     byte POXenabled = 1;
     int POXrange = 500;
     float POXzero = .5;
     int POXfill = 400;
     int POXalarm = 425;
     
     byte PFLenabled = 1;
     int PFLrange = 500;
     float PFLzero = .50;
     int PFLfill = 400;
     int PFLalarm = 425;
     
     byte PPSenabled = 0;
     int PPSrange = 3000;
     float PPSzero = .50;
     int PPSalarm = 3100;
     
     byte PRSenabled = 0;
     int PRSrange = 3000;
     float PRSzero = .5;
     int PRSalarm = 3100;

     int pressureWaitTime = 2000; //only send changes every 2 seconds
     int pressureMinChange = 2;
      
     int lineVentDelay = 5000; // five seconds
     int mainVoltage2412 = 24;
     int hazVoltage2412 = 24;
     int recoveryArmA = 0;
     int recoveryArmB = 1; // used for remote for PGO logic
     int valve9 = 0;
     int holdMainOn = 1;
     int holdMainOff = 0;
     int padRadioPower = 1;
     int remoteRadioPower = 1;
     int CPUtempAlarm = 170;
     int DAQauto = 1;
     int DAQtimeout = 30000;
     
             
  };
  configStruct configuration;

  struct localStruct {
    // used for local code configuration / non-configurable
    float batt24low = 21.0;
    float batt12low = 10.0;
    int mainHold = 10000; // ten seconds to hold main valves open/closed
    int DAQflush = 10000; // millis to safety close/open/flush file
    int logRate = 10; // 100 hz (10ms pause)
    int DAQsuperTimeout = 360000; // six minutes super timeout
    int FTPsize = 100; // FTP packet size - increase sendChunk below
   
  };
  localStruct local;


  struct timerStruct {  // timer variables
    unsigned long radioRXTimer = 0;
    unsigned long radioTXTimer = 0;
    unsigned long radioSendCallSignTimer = 90000;
    unsigned long healthTimer = 0;
    int radioRXInterval = 300;
    int radioTXInterval = 1200;
    int radioSendCallSignInterval = 90000;
    unsigned long housekeepingInput = 0;
    int housekeepingInputInterval = 500;  // set to 500 ms for now
    unsigned long fullStatusTimer = 0;
    int fullStatusInterval = 10000; // send full status every 10 seconds (and when an event)
    unsigned long pressureTimer = 0;
    unsigned long igniterTimer = 0;
    unsigned long fuelLineTimer = 0;
    unsigned long oxLineTimer = 0;
    unsigned long mainOff = 0;
    unsigned long mainOn = 0;
    unsigned long samplePulse = 0; // sample thread health check
    unsigned long loggingPulse = 0; // logging thread health check
    unsigned long health = 0; // error check

  };
  timerStruct timers;

  struct workingStruct {  // working variables

    char statusSentenceString[200]; // full status to send
    char configString[200]; // full config send
    int lastPPS = 0;
    int lastPOX = 0;
    int lastPFL = 0;
    int lastPRS = 0;
    bool POXzero = false;
    bool PFLzero = false;
    bool PPSzero = false;
    bool PRSzero = false;
    // DAQ, Logging, and File TX
    byte DAQstart = 0; // 1 = start, 2 = stop
    byte DAQrecording = 0;
    char DAQfilename[20];
    int flashFormat = 0;
    //FTP send controls
    volatile bool requestDir = false;
    int dirFileCount = 0;
    int totalPackets = 0;
    volatile bool sendFlag = false;
    char sendFile[20];
    char sendChunk[110];
    int currentPacket = 0;
    bool ftpMode = false;
    bool getFile = false;
    elapsedMillis superTimeout = 0;  
    bool beepErrors = false; // for first health check beeping

  };
  workingStruct working;

  struct errorsStruct {

    int mainBatt = 2;
    int hazBatt = 2;
    int MCUtemp = 2;
    int PT = 2;
    int radio = 2;
    int flashMem = 2;
    int flashFull = 2;
    int ADC = 2;
    int MCUcrash = 2;
    int errorCount = 0;
  };
  errorsStruct errors;

  struct padStatusStruct {

    //new PT values 
    int POX = 0;  // Rocket Ox PT
    int PFL = 0;  // Rocket Fuel PT
    int PPS = 0;  // Pad Spare PT
    int PRS = 0;  // Rocket Spare PT
    

    int tankConfig = 2; // valid is 1,2,3
    byte rocketConnected = 0;
    byte padHot = 0; //pad safety plug in/out (haz 24v) 
    byte igniterArmed = 0;
    byte valvesArmed = 0;
    byte armedState = 0;
    byte igniterContinuity = 0;
    float mainVolts = 0.0;
    float hazVolts = 0.0;
    int tankTemperature = 0; // Pressure tank temp
    byte valveOne = 0; //closed = 0, open = 1
    byte valveTwo = 0;
    byte valveThree = 0;
    byte valveFour = 0;
    byte valveFive = 0;
    byte valveSix = 0;
    byte valveSeven = 0;
    byte valveEight = 0;
    byte valveNine = 0;
    byte mainValvesOn = 0;
    byte mainValvesOff = 0;
    byte mainValvesState = 0;
    byte tankThreeActuatorOn = 0;
    byte tankThreeActuatorOff = 0;
    byte processC1 = 0; // can be 0- not run, 1 - complete, 2 - running, 3-stopped
    byte processF1 = 0;
    byte processF2 = 0;
    byte processO1 = 0;
    byte processO2 = 0;
    byte processAbort = 0;
    byte processAC = 0;
    byte process8 = 0;
    byte oxTankDisconnect = 0;
    byte fuelTankDisconnect = 0;
    byte pressureTankDisconnect = 0;
    byte recoveryPower = 0;
    byte armedA = 0;
    byte armedB = 0;
    int CPUtemp = 0;
    int fuelFill = 0; //zzz
    int oxFill = 0;
    byte mainOff = 0;
    byte mainOn = 0;

  };
  padStatusStruct padStatus;

  struct ADCvalueStruct {
    //int is the interval to check in ms
    //used to throttle sample speed in thread
    int   mainVoltageInt = 5000;
    int   hazVoltageInt = 500;
    int   padSparePTint = 10;
    int   oxPTint = 10;
    int   fuelPTint = 10;
    int   rocketSparePTint = 10;
  };
  ADCvalueStruct ADCvalue;


// *** Radio Setup
  char radioHeader[7];
  char radioMessage[150];
  char radioMessageS[200];
  int newWord = 0;
  char theWord[120];

//added for send queue
  char rSend[15][300];
  byte rSendCount = 0;
  byte rSendLast = 0;
  byte rSendPos = 0;
  char vFullMessageR[300];


//ADS1115 ADC
  #include "ADS1115-Driver.h"
  ADS1115 ads1115a = ADS1115(0x49);
  ADS1115 ads1115b = ADS1115(0x48);



// For the flash memory and logging
  //Little FS flash
  #include <LittleFS.h>
  LittleFS_QPINAND myfs; // used for fully loaded T4.1
  File configFile;
  #include <stdarg.h>
  #include <TimeLib.h>  // for logging time

// Temperature from CPU
  extern float tempmonGetTemp(void);  



void setup() {

  Serial.begin(115200);
  if(CrashReport) { // capture if the MCU core dumps
    if(serialDebug) Serial.print(CrashReport);
    errors.MCUcrash = 1;
  } else {
    errors.MCUcrash = 0;
  }

  pinMode(pin_led1, OUTPUT);  digitalWrite(pin_led1, LOW);
  pinMode(pin_led2, OUTPUT);  digitalWrite(pin_led2, LOW);
  pinMode(pin_valve1, OUTPUT);  digitalWrite(pin_valve1, LOW);
  pinMode(pin_valve2, OUTPUT);  digitalWrite(pin_valve2, LOW);
  pinMode(pin_valve3, OUTPUT);  digitalWrite(pin_valve3, LOW);
  pinMode(pin_valve4, OUTPUT);  digitalWrite(pin_valve4, LOW);
  pinMode(pin_valve5, OUTPUT);  digitalWrite(pin_valve5, LOW);
  pinMode(pin_valve6, OUTPUT);  digitalWrite(pin_valve6, LOW);
  pinMode(pin_valve7, OUTPUT);  digitalWrite(pin_valve7, LOW);
  pinMode(pin_valve8, OUTPUT);  digitalWrite(pin_valve8, LOW);
  pinMode(pin_valve9, OUTPUT);  digitalWrite(pin_valve9, LOW);
  pinMode(pin_buzzer, OUTPUT);  digitalWrite(pin_buzzer, LOW);
  pinMode(pin_igniterFIRE, OUTPUT);  digitalWrite(pin_igniterFIRE, LOW);
  pinMode(pin_mainValveOn, OUTPUT);  digitalWrite(pin_mainValveOn, LOW);
  pinMode(pin_mainValveOff, OUTPUT);  digitalWrite(pin_mainValveOff, LOW);
  pinMode(pin_recoveryOff, OUTPUT);  digitalWrite(pin_recoveryOff, LOW);
  pinMode(pin_recoveryOn, OUTPUT);  digitalWrite(pin_recoveryOn, LOW); 
  pinMode(pin_igniterContinuity, INPUT_PULLUP);   
  pinMode(pin_altAarmed, INPUT_PULLUP);  
  pinMode(pin_altBarmed, INPUT_PULLUP);   
  pinMode(pin_rocketDetect, INPUT_PULLUP);  
  pinMode(pin_continuitySwitch, INPUT_PULLUP);  
  pinMode(pin_fuse5v, INPUT_PULLUP);
  pinMode(pin_fuseMain, INPUT_PULLUP);
  pinMode(pin_fuseHaz, INPUT_PULLUP);
  pinMode(pin_radioSet, OUTPUT);digitalWrite(pin_radioSet, LOW);
  pinMode(pin_internalLED, OUTPUT); digitalWrite(pin_internalLED, LOW);
  pinMode(pin_ext5v, OUTPUT);digitalWrite(pin_ext5v, HIGH);  //activate 5v ouput immediately
  pinMode(pin_strobes, OUTPUT);digitalWrite(pin_strobes, LOW);
  pinMode(pin_fan, OUTPUT);digitalWrite(pin_fan, LOW);
  
  delay(5000); // give radio time to power up

  readConfig(); 


   //create separate thread loop for sampling
  id_sampling = threads.addThread(sample_thread,0, 2048,0);
  threads.setMicroTimer(1000);//5
  threads.yield();    
  threads.delay(100);

   //create separate thread loop for DAQ
  id_logging = threads.addThread(DAQ_thread,0, 2048,0);
  threads.yield();  

  //pre-run some checks
  CPUtemp();
  timers.healthTimer = millis() + 6000;
  
  // Initialize Radio 
  radioTest();
  Serial1.begin(19200);


  if(serialDebug) Serial.println("Starting up....");
  threads.delay(2000);
  mainValveOff();
  
}


//***********************************************************        MAIN LOOP    ************************************************************
//***********************************************************        MAIN LOOP    ************************************************************

void loop() {



// housekeeping timers


  if(millis() > timers.radioRXTimer) {
    checkRadio();
    timers.radioRXTimer = millis() + timers.radioRXInterval;
    
  }

  if(millis() > timers.radioTXTimer && rSendCount > 0) {
    RadioSendQueue();
    timers.radioTXTimer = millis() + timers.radioTXInterval; //1200 default spacing
  }

  if(millis() > timers.housekeepingInput) {

    checkPressure();                // Check Pressures
    checkRocketConnect();           // Check Rocket Detect
    checkIgniterContinuity();                 // Check igniter continuity
    checkVoltage();                 // Check voltages and pad hot 
    checkContinuitySwitch();        // Check Continuity switch
    checkRecoveryArmed();      // Check recovery continuity
    igniterCheck();                 // Check igniter to turn off
    processCheck();                 // Check the active processes
    strobeCheck();                  // Turn on strobes if Haz key enabled
    
    timers.housekeepingInput = millis() + timers.housekeepingInputInterval;
  }

  if(millis() > timers.health) {
     checkHealth();                  // check for new errors
     timers.health = millis() + 30;
  }

  if(millis() > timers.fullStatusTimer) {
    sendFullStatus();
  }

  threads.yield();


}

//***********************************************************        END     ************************************************************



void processCheck() {

  if(padStatus.processF1 == 2) {  // check fuel pressurization
    if(padStatus.PFL >= padStatus.fuelFill) { 
      valveClose(4);
      padStatus.processF1 = 1; //mark as complete
      sendFullStatus(); 
      strcpy(radioHeader, "#M");
      strcpy(radioMessage, "Fuel pressurization complete");
      RadioSend(); 
    }
  }

  if(padStatus.processO1 == 2) {  // check ox pressurization
    if(padStatus.POX >= padStatus.oxFill) { 
      valveClose(5);
      padStatus.processO1 = 1; //mark as complete
      sendFullStatus(); 
      strcpy(radioHeader, "#M");
      strcpy(radioMessage, "Ox pressurization complete");
      RadioSend(); 
    }
  }

  if(padStatus.processF2 == 2) { // check fuel disconnect
    if(millis() > timers.fuelLineTimer) {
      valveOpen(2); // open the disconnect 
      padStatus.processF2 = 1;
      sendFullStatus(); 
      strcpy(radioHeader, "#M");
      strcpy(radioMessage, "Fuel disconnect complete");     
    }
  }

  if(padStatus.processO2 == 2) { // check ox disconnect
    if(millis() > timers.oxLineTimer) {
      valveOpen(6); // open the disconnect 
      padStatus.processO2 = 1;
      sendFullStatus(); 
      strcpy(radioHeader, "#M");
      strcpy(radioMessage, "Ox disconnect complete");     
    }
  }

  if(padStatus.mainOff == 2) { // check to stop power
    if(millis() > timers.mainOff) {
      digitalWrite(pin_mainValveOff, LOW);
      padStatus.mainOff = 0;
    }
  }
  if(padStatus.mainOn == 2) { // check to stop power
    if(millis() > timers.mainOn) {
      digitalWrite(pin_mainValveOn, LOW);
      padStatus.mainOn = 0;    
    }
  }


}

void CPUtemp() {

      float CPUtemp = 0.0;
      CPUtemp  = (tempmonGetTemp() * 9.0f / 5.0f) + 32.0f;
      padStatus.CPUtemp = (int)CPUtemp;
      if(padStatus.CPUtemp > configuration.CPUtempAlarm) {
        errors.MCUtemp = 1;
      } else {
        errors.MCUtemp = 0;
      }
}

void strobeCheck() {
  
  if(digitalRead(pin_fuseHaz) == LOW) {
    digitalWrite(pin_strobes, HIGH);
  } else {
    digitalWrite(pin_strobes, LOW);
  }

  
}


void igniterCheck() {
  if(timers.igniterTimer > 0) {
    if(millis() > timers.igniterTimer) {
      digitalWrite(pin_igniterFIRE, LOW);
      threads.delay(10);
      timers.igniterTimer = 0;
      checkIgniterContinuity();
    }
  }
}


void sendFullStatus() {
      // Send full status
    statusSentence(); // set the status sentence
    if(serialDebug) Serial.println(F("Sending FULL STATUS"));
    if(serialDebug) Serial.println(working.statusSentenceString);
    strcpy(radioHeader, "#FS");
    strcpy(radioMessage, working.statusSentenceString);
    RadioSend();  
    timers.fullStatusTimer = millis() + timers.fullStatusInterval;
}

void checkRecoveryArmed() {

  if(digitalRead(pin_altAarmed) == LOW) {
    padStatus.armedA = 1;
  } else {
    padStatus.armedA = 0;
  }
  if(digitalRead(pin_altBarmed) == LOW) {
    padStatus.armedB = 1;
  } else {
    padStatus.armedB = 0;
  }
}

void checkContinuitySwitch() {   //physical pad switch

  int tempRead = 0;
  tempRead = digitalRead(pin_continuitySwitch);
  if(tempRead == 0) {  // button pressed down
      checkIgniterContinuity(); 
      if(padStatus.igniterContinuity == 1) {
        if(serialDebug) Serial.println(F("Good Continuity"));
        digitalWrite(pin_led2, HIGH);
        if(!mute) digitalWrite(pin_buzzer, HIGH);
        threads.delay(2000);
        digitalWrite(pin_led2, LOW);
        digitalWrite(pin_buzzer, LOW);
      } else {
        if(serialDebug) Serial.println(F("Bad Continuity"));
        digitalWrite(pin_led1, HIGH);
        threads.delay(250);
        digitalWrite(pin_led1, LOW);
        threads.delay(250);
        digitalWrite(pin_led1, HIGH);
        threads.delay(250);
        digitalWrite(pin_led1, LOW);
        threads.delay(250);
        digitalWrite(pin_led1, HIGH);
        threads.delay(500);
        digitalWrite(pin_led1, LOW);
      }
  }
}


void checkHealth() {

    CPUtemp();
  
    if((padStatus.POX == 99999 && configuration.POXenabled == 1) || (padStatus.PFL == 99999 && configuration.PFLenabled == 1) || (padStatus.PPS == 99999 && configuration.PPSenabled == 1) || (padStatus.PRS == 99999 && configuration.PRSenabled == 1)) {
      errors.PT = 1;
    } else {
      errors.PT = 0;
    }
  
    int theCount = 0;
    bool sendNow = false;
  
    if(errors.mainBatt == 1) theCount++;
    if(errors.hazBatt == 1) theCount++;
    if(errors.MCUtemp == 1) theCount++;
    if(errors.PT == 1) theCount++;
    if(errors.radio == 1) theCount++;
    if(errors.flashMem == 1) theCount++;
    if(errors.flashFull == 1) theCount++;
    if(errors.ADC == 1) theCount++;
    if(errors.MCUcrash == 1) theCount++;

    if(errors.errorCount != theCount) sendNow = true;
    errors.errorCount = theCount;


    if(sendNow || millis() > timers.healthTimer) {

      // Check to see if both threads are healthy w/watchdog timers
      if(timers.loggingPulse < millis()) {
        errors.flashMem = 1;
        if(serialDebug) Serial.println("Logging thread has crashed");
      }
      if(timers.samplePulse < millis()) {
        if(serialDebug) Serial.println("Sample thread has crashed");
        errors.ADC = 1;
      }

      //------------ Startup Beeps -------------
      if(!working.beepErrors) {  // Three beeps is A-OK
       if(errors.errorCount == 0) {
         digitalWrite(pin_led2, HIGH);
         beepCount(3);
         digitalWrite(pin_led2, LOW);
       } else {
          digitalWrite(pin_led1, HIGH);
          if(!mute) digitalWrite(pin_buzzer, HIGH);
          threads.delay(3000);
          digitalWrite(pin_buzzer, LOW);
          threads.delay(1000);
          if(errors.mainBatt == 1) {beepCount(2); threads.delay(1000);}
          if(errors.hazBatt == 1) {beepCount(4); threads.delay(1000);}
          if(errors.MCUtemp == 1) {beepCount(5); threads.delay(1000);}
          if(errors.PT == 1) {beepCount(1); threads.delay(1000);}
          if(errors.radio == 1) {beepCount(6); threads.delay(1000);}
          if(errors.flashMem == 1) {beepCount(7); threads.delay(1000);}
          if(errors.ADC == 1) {beepCount(8); threads.delay(1000);}
          if(errors.MCUcrash == 1) {beepCount(9); threads.delay(1000);}
          digitalWrite(pin_led1, LOW);
       }


        working.beepErrors = true;
      }
      
    
      if(serialDebug) Serial.print(F("Health Check Error Count is: "));
      if(serialDebug) Serial.println(errors.errorCount);
  
      char healthString[100];
      char comma[5] = ",";
      char str_int[16];
    
      strcpy(healthString, "");  //zero out the string
      //------------ START ------------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , errors.errorCount);
      strcat(healthString, str_int);strcat(healthString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , errors.mainBatt);
      strcat(healthString, str_int);strcat(healthString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , errors.hazBatt);
      strcat(healthString, str_int);strcat(healthString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , errors.MCUtemp);
      strcat(healthString, str_int);strcat(healthString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , errors.PT);
      strcat(healthString, str_int);strcat(healthString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , errors.radio);
      strcat(healthString, str_int);strcat(healthString, comma);
      //-------------                         
      strcpy(str_int, ""); sprintf (str_int, "%d" , errors.flashMem);
      strcat(healthString, str_int);strcat(healthString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , errors.flashFull);
      strcat(healthString, str_int);strcat(healthString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , errors.ADC);
      strcat(healthString, str_int);strcat(healthString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , errors.MCUcrash);
      strcat(healthString, str_int);
      //-------------
    
     if(serialDebug) {
      Serial.print(F("Sending health data:  ")); Serial.println(healthString);
     }
     strcpy(radioHeader, "#HER");   
     strcpy(radioMessage, healthString);
     RadioSend(); 
     timers.healthTimer = millis() + 60000;
    }
  
}

void beepCount(int num) {
  for (int i = 0; i <= (num-1); i++) {
        if(!mute) digitalWrite(pin_buzzer, HIGH);
        threads.delay(500);
        digitalWrite(pin_buzzer, LOW);
        threads.delay(500);
  }  
}

void checkVoltage() {   //also checks pad hot from haz voltage

  // voltage reads moved to ADC thread

  if(padStatus.padHot == 0 && padStatus.hazVolts > 5.0) {  // the pad just got hot
      padStatus.padHot = 1;
    //send a full status
    statusSentence(); // set the status sentence
    if(serialDebug) Serial.println(F("Pad is now HOT"));
    strcpy(radioHeader, "#FS");
    strcpy(radioMessage, working.statusSentenceString);
    RadioSend();  
    timers.fullStatusTimer = millis() + timers.fullStatusInterval;    
    //send a message
    strcpy(radioHeader, "#M");
    strcpy(radioMessage, "The Pad is now HOT");
    RadioSend();       
  }
  
  if(padStatus.padHot == 1 && padStatus.hazVolts <= 5.0) {  // the pad just got cold
      padStatus.padHot = 0;
      padStatus.valvesArmed = 0;
      padStatus.armedState = 0;
    // close any haz valves
      valveClose(4); // close haz fuel pressurization
      valveClose(5); // close haz ox pressurization
      mainValveOff();
      if (padStatus.processF1 == 2) padStatus.processF1 = 0; // stop fill processes if running
      if (padStatus.processO1 == 2) padStatus.processO1 = 0; // stop fill processes if running
      threads.delay(50);
      checkIgniterContinuity();  
      
      //send a full status
      statusSentence(); // set the status sentence
      if(serialDebug) Serial.println(F("Pad is now COLD"));
      strcpy(radioHeader, "#FS");
      strcpy(radioMessage, working.statusSentenceString);
      RadioSend();  
      timers.fullStatusTimer = millis() + timers.fullStatusInterval;    
      //send a message
      strcpy(radioHeader, "#M");
      strcpy(radioMessage, "The Pad is now COLD");
      RadioSend();       
  }
  // health checking batteries
  
  if(padStatus.padHot == 1) {
    errors.hazBatt = 0;
    if(configuration.hazVoltage2412 == 24 && padStatus.hazVolts < local.batt24low) errors.hazBatt = 1;
    if(configuration.hazVoltage2412 == 12 && padStatus.hazVolts < local.batt12low) errors.hazBatt = 1;
  } else {
    errors.hazBatt = 2;
  }
  if(configuration.mainVoltage2412 == 24) {
    if(padStatus.mainVolts < local.batt24low) {
      errors.mainBatt = 1;
    } else {
      errors.mainBatt = 0;
    }
  } else {
    if(padStatus.mainVolts < local.batt12low) {
      errors.mainBatt = 1;
    } else {
      errors.mainBatt = 0;
    }
  }
  
}

void checkIgniterContinuity() { //new

  int tempRead = 0;
  tempRead = digitalRead(pin_igniterContinuity);
  
  if(padStatus.igniterContinuity == 0 && tempRead == 0) {  // the igniter just got continuity
    padStatus.igniterContinuity = 1;
    //send a full status
    statusSentence(); // set the status sentence
    if(serialDebug) {
      Serial.print(F("Igniter now has continuity - reading: "));
      Serial.println(tempRead);
    }
    strcpy(radioHeader, "#FS");
    strcpy(radioMessage, working.statusSentenceString);
    RadioSend();  
    timers.fullStatusTimer = millis() + timers.fullStatusInterval;    
    //send a message
    strcpy(radioHeader, "#M");
    strcpy(radioMessage, "The Igniter now has continuity");
    RadioSend();        
  }

  if(padStatus.igniterContinuity == 1 && tempRead == 1) {  // the igniter just lost continuity
    padStatus.igniterContinuity = 0;
    //send a full status
    statusSentence(); // set the status sentence
    if(serialDebug) {
      Serial.print(F("Igniter has lost continuity - reading: "));
      Serial.println(tempRead);
    }
    strcpy(radioHeader, "#FS");
    strcpy(radioMessage, working.statusSentenceString);
    RadioSend();  
    timers.fullStatusTimer = millis() + timers.fullStatusInterval;    
    //send a message
    strcpy(radioHeader, "#M");
    strcpy(radioMessage, "The Igniter has lost continuity");
    RadioSend();       
  }
}

void checkRocketConnect() {

  byte tempDetect;

  tempDetect = digitalRead(pin_rocketDetect);

  if(tempDetect == 0 && padStatus.rocketConnected == 0) {  //we just got connected
    padStatus.rocketConnected = 1;
    //send a full status
    statusSentence(); // set the status sentence
    if(serialDebug) Serial.println(F("Rocket now connected"));
    strcpy(radioHeader, "#FS");
    strcpy(radioMessage, working.statusSentenceString);
    RadioSend();  
    timers.fullStatusTimer = millis() + timers.fullStatusInterval;    
    //send a message
    strcpy(radioHeader, "#M");
    strcpy(radioMessage, "Rocket is now connected");
    RadioSend();  
  }

  if(tempDetect == 1 && padStatus.rocketConnected == 1) {  //we just got disconnected
    padStatus.rocketConnected = 0;
    //send a full status
    statusSentence(); // set the status sentence
    if(serialDebug) Serial.println(F("Rocket Disconnected"));
    strcpy(radioHeader, "#FS");
    strcpy(radioMessage, working.statusSentenceString);
    RadioSend();  
    timers.fullStatusTimer = millis() + timers.fullStatusInterval;    
    //send a message
    strcpy(radioHeader, "#M");
    strcpy(radioMessage, "Rocket is now disconnected");
    RadioSend();  
  }
}


void checkPressure() {

/*  All reading has been moved to the sample thread
 *  New Structure
      Values are no in: 
        padStatus.POX
        padStatus.PFL
        padStatus.PPS
        padStatus.PRS
      Enabled in:
        configuration.POXenabled (etc)
       
 */
 
  bool sendUpdate = false;

  if((padStatus.POX > (working.lastPOX + configuration.pressureMinChange) || padStatus.POX < (working.lastPOX - configuration.pressureMinChange))  && configuration.POXenabled == 1) sendUpdate = true;
  if((padStatus.PFL > (working.lastPFL + configuration.pressureMinChange) || padStatus.PFL < (working.lastPFL - configuration.pressureMinChange))  && configuration.PFLenabled == 1) sendUpdate = true;

  if(padStatus.PPS != working.lastPPS && configuration.PPSenabled == 1) sendUpdate = true;
  if(padStatus.PRS != working.lastPRS && configuration.PRSenabled == 1) sendUpdate = true;
  

  // only send updates every two seconds and if > 2 
  if(millis() > timers.pressureTimer && sendUpdate == true) {

      char pressureString[100];
      char comma[5] = ",";
      char str_int[16];
    
      strcpy(pressureString, "");  //zero out the string
      //------------ START ------------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.PPS);
      strcat(pressureString, str_int);strcat(pressureString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.POX);
      strcat(pressureString, str_int);strcat(pressureString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.PFL);
      strcat(pressureString, str_int);strcat(pressureString, comma);
      //-------------
      strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.PRS);
      strcat(pressureString, str_int);
      //-------------            

     if(serialDebug) {
      Serial.print(F("Sending new pressure data:  ")); Serial.println(pressureString);
     }
     strcpy(radioHeader, "#PRS");   
     strcpy(radioMessage, pressureString);
     RadioSend(); 
     timers.pressureTimer = millis() + configuration.pressureWaitTime;
     
     working.lastPOX = padStatus.POX;
     working.lastPFL = padStatus.PFL;
     working.lastPPS = padStatus.PPS;
     working.lastPRS = padStatus.PRS;

  }  
}


void statusSentence() {

  char comma[5] = ",";
  char str_int[16];

  strcpy(working.statusSentenceString, "");  //zero out the string
  //------------ START ------------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.tankConfig);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.rocketConnected);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.padHot);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------  
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.igniterArmed);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.valvesArmed);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.armedState);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.igniterContinuity);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%1.1f" , padStatus.mainVolts);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------  
  strcpy(str_int, ""); sprintf (str_int, "%1.1f" , padStatus.hazVolts);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
    strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.PPS);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.POX);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.PFL);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.PRS);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.tankTemperature);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.valveOne);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------    
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.valveTwo);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.valveThree);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.valveFour);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.valveFive);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.valveSix);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.valveSeven);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.valveEight);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.valveNine);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.mainValvesOn);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.mainValvesOff);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.mainValvesState);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.tankThreeActuatorOn);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.tankThreeActuatorOff);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.processC1);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.processF1);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.processF2);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.processO1);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.processO2);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.processAbort);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.processAC);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.process8);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------  
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.oxTankDisconnect);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------  
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.fuelTankDisconnect);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------  
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.pressureTankDisconnect);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------  
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.recoveryPower);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------  
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.armedA);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------  
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.armedB);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------    
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.CPUtemp);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------  
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.oxFill);
  strcat(working.statusSentenceString, str_int);strcat(working.statusSentenceString, comma);
  //-------------    
  strcpy(str_int, ""); sprintf (str_int, "%d" , padStatus.fuelFill);
  strcat(working.statusSentenceString, str_int);

  
  //-------------  END 

     working.lastPOX = padStatus.POX;
     working.lastPFL = padStatus.PFL;
     working.lastPPS = padStatus.PPS;
     working.lastPRS = padStatus.PRS;

}




//***********************************************************        RADIO LOGIC    ************************************************************
//***********************************************************        RADIO LOGIC     ************************************************************

void radioTest() {
  
    digitalWrite(pin_radioSet, HIGH); //turn radio set pin to low
    threads.delay(200);
    Serial1.begin(9600);
    char s[5];
    strcpy(s,"");
    s[0] = 0xaa;
    s[1] = 0xfa;
    s[2] = 0xaa; // aa = product model number 
    s[3] = 0x0d; //  /r termination
    s[4] = 0x0a; //  /n termination
    Serial1.println(s);

    radioTestRead();
    if(serialDebug) Serial.println(theWord);
    char *ptr = strstr(theWord, "LORA6100");
    if (ptr == NULL) {
      errors.radio = 1; 
      if(serialDebug) Serial.println("ERROR:  No Radio Found");
    } else {
      errors.radio = 0;
    }
    digitalWrite(pin_radioSet, LOW);
    threads.delay(200);
      
}

void radioTestRead() {  
  
   newWord = 0;
   char receivedChar;
   unsigned long testTime = millis() + 500;    
   strcpy(theWord, "");
   while (newWord != 1) {
      if(Serial1.available()) {
        receivedChar = Serial1.read();
        append(theWord, receivedChar);
      if(receivedChar == 0x0a) {
           newWord = 1;
           break;   
         }
       }
       if(millis() > testTime) {  // exit and evaluate if it takes too long
          newWord = 1;
          break;
       }
       threads.delay(1);
     }
}

void RadioSend() {  //New and improved
  //populate global radioHeader (e.g. @M) and radioMessage first
    strcpy(radioMessageS, "");
    strcat(radioMessageS, radioHeader);
    strcat(radioMessageS, ",");    
    strcat(radioMessageS, radioMessage);      
    strcat(radioMessageS, ",!");
    //done creating string
    //add it to the send queue n times
    rSendCount ++;
    rSendPos ++;
    if (rSendPos == 11) rSendPos = 1;
    strcpy(rSend[rSendPos], radioMessageS);  
}


void RadioSendQueue () {

  // this is designed to cut down on send congestion. Buffer packets 1000ms apart so they don't mangle together
  // Globals:  String rSend[10];  byte rSendCount = 0;  rSendLast = 0; rSendPos = 0; RadioSendQueueRate = 1000
   if(rSendCount > 0) {
     if (Serial1.available() > 0) { checkRadio(); } // don't send if inbound packets
     digitalWrite(pin_led1, HIGH); //blink onboard light
     rSendLast ++;
     if (rSendLast == 11) rSendLast = 1;
     Serial1.print(rSend[rSendLast]);
     Serial1.flush(); //waits for outgoing transmission to complete
     rSendCount = rSendCount - 1; 
     digitalWrite(pin_led1, LOW);
   }
}



void checkRadio() {  // **********  CHECK RADIO  *****************
   newWord = 0;
   char receivedChar;
   byte vtimeout = 0;
   int wCount = 0;
   byte rangeError = 0;
   byte lengthError = 0;
      
   if (Serial1.available() > 0) {
  //  Serial.println("Rx...");
    unsigned long testTime = millis() + 2000;
    strcpy(theWord, "");
    if(serialDebug) Serial.println("radio inbound detect");

    while (newWord != 1) {
       if(Serial1.available()) {

         receivedChar = Serial1.read();
         if(receivedChar == 33) {  // look for ! to end
          newWord = 1;
          append(theWord, receivedChar);
          //Serial1.read();  // read the extra end char ?
         } else {
           append(theWord, receivedChar);
           wCount++;
         }
         // Error checking
         if(wCount > 145) { // too long without exit
            lengthError = 1;
            newWord = 1; //abort
         }
         if(receivedChar < 32 || receivedChar > 126) {  //noise or garbage on the radio - empty and abort to not waste cycles
            rangeError = 1;
            newWord = 1;
         }
       }
       if(millis() > testTime) {  // exit and evaluate if it takes too long
          newWord =1; //force exit
          vtimeout = 1;
       }
       //threads.delay(1);
     } //end while 

     // Error processing
     if(rangeError == 1) {
       if(serialDebug) Serial.print("**** Radio RX out of range - noise ignored ****");
       while(Serial1.available()) {
              receivedChar = Serial1.read();
              if(millis() > testTime) break;
            }
     }
     if(lengthError == 1) {
       if(serialDebug) Serial.print("**** Radio RX length issue - purging ****");
       while(Serial1.available()) {
              receivedChar = Serial1.read();
              if(millis() > testTime) break;
            }
     }     
             
     if(vtimeout == 1) {
       if(serialDebug) Serial.println("timeout exit error - aborting...");
       if(serialDebug) Serial.println(theWord);
     } else {
       if(serialDebug) {Serial.print("Radio RX: ");Serial.println(theWord);}
       if(working.ftpMode) {
         ftpProcess();
       } else {
         ProcessRadio();
       }
     }
   } 
}


/*
void checkRadioOld() {  // **********  CHECK RADIO  *****************
   newWord = 0;
   char receivedChar;
   byte vtimeout = 0;
   int wCount = 0;
   byte rangeError = 0;
   byte lengthError = 0;
      
   if (Serial1.available() > 0) {
  //  Serial.println("Rx...");
    unsigned long testTime = millis() + 2000;
    strcpy(theWord, "");
    if(serialDebug) Serial.println("radio inbound detect");

    while (newWord != 1) {
       if(Serial1.available()) {

         receivedChar = Serial1.read();
         if(receivedChar == 33) {  // look for ! to end
          newWord = 1;
          append(theWord, receivedChar);
          Serial1.read();  // read the extra end char
         } else {
           append(theWord, receivedChar);
           wCount++;
         }
         // Error checking
         if(wCount > 145) { // too long without exit
            lengthError = 1;
            newWord = 1; //abort
         }
         if(receivedChar < 32 || receivedChar > 126) {  //noise or garbage on the radio - empty and abort to not waste cycles
            rangeError = 1;
            newWord = 1;
         }
       }
       if(millis() > testTime) {  // exit and evaluate if it takes too long
          newWord =1; //force exit
          vtimeout = 1;
       }
       //threads.delay(1);
     } //end while 

     // Error processing
     if(rangeError == 1) {
       if(serialDebug) Serial.print("**** Radio RX out of range - noise ignored ****");
       while(Serial.available()) {
              receivedChar = Serial1.read();
              if(millis() > testTime) break;
            }
     }
     if(lengthError == 1) {
       if(serialDebug) Serial.print("**** Radio RX length issue - purging ****");
       while(Serial.available()) {
              receivedChar = Serial1.read();
              if(millis() > testTime) break;
            }
     }     
             
     if(vtimeout == 1) {
       if(serialDebug) Serial.println("timeout exit error - aborting...");
       if(serialDebug) Serial.println(theWord);
     } else {
       if(serialDebug) {Serial.print("Radio RX: ");Serial.println(theWord);}
       if(working.ftpMode) {
         ftpProcess();
       } else {
         ProcessRadio();
       }
     }
   } 
}
*/

void ProcessRadio() {   // **********  PROCESS RADIO COMMAND  *****************

      int chkNew = 0;
      if(theWord[0] == 35 && theWord[strlen(theWord)-1] == 33) {   // proper sentence recieved from radio # !          
          chkNew = 1;

      } else {  // error
              newWord = 0;
              strcpy(theWord, "");
              if(serialDebug) {Serial.print("Bad RX Sentence: ");Serial.println(theWord);}
      }
      // **************  SUMMARY OF COMMANDS  ********************
     /*
      * #S  Send full status
        #O# Open valve #
        #C# Close valve #
        #ARM1/0  Arm Haz and Ignitor
        #AH1/0 Arm Haz only / zzz
        #PC1 - close all valves
        #PF1 - 
        #PF2 -
        #PO1 - 
        #PO2 - 
        #STOP Stop all process tasks
        #RON  Turn Recovery On
        #ROFF Turn Recovery Off
        #MON  Main Valves Open
        #MOFF Main Valves Closed
        #FIRE Ignitor Fire
        #ABORT  Abort
        #AC Abort Cancel
        #SCFG,lox fill, fuel fill, !  -  set ox and fuel fill pressure
      * 
      */
      
      if (chkNew == 1) {  // Real processing goes here            ********************************  RADIO PROCESSING  ******************************

          /*  1 - Fuel line vent
           *  2 - Fuel Pnuematic
           *  3 - Ox line vent
           *  4 - Fuel pressurization
           *  5 - Ox pressurization
           *  6 - Ox Pnuematic
           *  7 - Fuel Rocket Vent
           *  8 - Ox Rocket Vent
           */


          //-------------- STATUS  --------------------------------
          if (strncmp("#S,",theWord,3) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action STATUS"));
            strcpy(radioHeader, "#M");
            strcpy(radioMessage, "Hello from the Pad");
            RadioSend();  
            sendFullStatus(); 
            sendConfig();
            timers.healthTimer = millis(); // force health tx
            timers.fullStatusTimer = millis() + timers.fullStatusInterval;
         
          }
          //-------------- OX and FUEL FILL CONFIG  --------------------------------
          if (strncmp("#SCFG,",theWord,6) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action Fill Config"));
            char tempFill[20][20]={0x0};
            parseit(theWord,tempFill);
            int tempOx = 0;
            int tempFuel = 0;
            tempOx = atoi(tempFill[1]);
            tempFuel = atoi(tempFill[2]);            
            if((tempOx <= configuration.POXrange && tempOx >= 0) && (tempFuel <= configuration.PFLrange && tempFuel >= 0)) {
              // good values
            } else {
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Error: Fill values out of range");
              if(serialDebug) Serial.println(F("Fill values out of range"));
              RadioSend();   
              return;
            }
            // good values
            padStatus.oxFill = tempOx;
            padStatus.fuelFill = tempFuel;
            configuration.POXfill = tempOx;
            configuration.PFLfill = tempFuel;
            writeConfig();
            sendConfig();
            sendFullStatus(); //zzz remove this later
            strcpy(radioHeader, "#M");
            strcpy(radioMessage, "Ox and Fuel Fill Config Changed");
            RadioSend();           
          }

          //-------------- NEW FULL CONFIG  --------------------------------
          if (strncmp("#FCFG,",theWord,6) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action FULL Config"));
            char tempFill[35][20]={0x0};
            parseit(theWord,tempFill);
            // they are all int but only a few tests needed

            if(configTest(tempFill)) {
              setConfig(tempFill);
              writeConfig();
              
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Configuration Update Confirmed");
              RadioSend();  
              sendConfig();
              
            } else {
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "ERROR Bad Configuration Data from Pad");
              RadioSend();
            }         
          }
          //-------------- RESET or RECYCLE  --------------------------------
          if (strncmp("#RSET,",theWord,6) == 0) { 
            if(serialDebug) Serial.println(F("Reset Request from Remote"));
            resetAll();
            threads.yield();      
          }

          //-------------- DEFAULT ALL CONFIG  --------------------------------
          if (strncmp("#DEFAULT,",theWord,9) == 0) { 
            if(serialDebug) Serial.println(F("Default All Request from Remote"));
            defaultAll();
            threads.yield();  
            writeConfig();
            threads.yield();  
            sendConfig();
            threads.yield();  
            resetAll();
            threads.yield();      
          }
          

          //-------------- DAQ Start  --------------------------------
          if (strncmp("#DAQON,",theWord,7) == 0) { 
            char tempFill[20][20]={0x0};
            parseit(theWord,tempFill);
            strcpy(working.DAQfilename,tempFill[1]);
            working.DAQstart = 1;
            if(serialDebug) Serial.println(F("Got radio action DAQ Start"));
            strcpy(radioHeader, "#DON");
            strcpy(radioMessage, "DAQ Start");  
            RadioSend();      
            threads.yield(); 
          }
          //-------------- DAQ Stop  --------------------------------
          if (strncmp("#DAQOFF",theWord,7) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action DAQ Stop"));
            strcpy(radioHeader, "#DOFF");
            strcpy(radioMessage, "DAQ Stop");  
            working.DAQstart = 2;
            threads.yield();
            RadioSend();       
          }

          //-------------- Flash Format  --------------------------------
          if (strncmp("#FF,",theWord,4) == 0) { 
              if(serialDebug) Serial.println(F("Got radio action Flash Format"));
              working.flashFormat = 1;
              strcpy(radioHeader, "$DIR");
              strcpy(radioMessage, "0~0");
              FTPsend();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Flash Memory Erase Confirmed");
              RadioSend();   
              
              
          }

          //-------------- MCU Files Request  --------------------------------
          if (strncmp("#DIR,",theWord,5) == 0) { 
              if(serialDebug) Serial.println(F("Got radio action MCU Files Request"));
              MCUfiles();
          }

          //-------------- MCU File GET Request  --------------------------------
          if (strncmp("#GET,",theWord,5) == 0) { 
              char tempFill[20][20]={0x0};
              parseit(theWord,tempFill);
              strcpy(working.sendFile,tempFill[1]);
              if(serialDebug) Serial.print(F("Got radio action MCU File GET Request:  "));
              if(serialDebug) Serial.println(working.sendFile);
              MCUget();
          }
          
          //-------------- VALVES OPEN  --------------------------------
          if(strncmp("#O",theWord,2) == 0) {  //evaluate open valve command
            if(strncmp("#O1,",theWord,4) == 0) {
              valveOpen(1);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve open");
              RadioSend();              
            }
            if(strncmp("#O2,",theWord,4) == 0) {
              valveOpen(2);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve open");
              RadioSend();              
            }
            if(strncmp("#O3,",theWord,4) == 0) {
              valveOpen(3);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve open");
              RadioSend();              
            }
            if(strncmp("#O4,",theWord,4) == 0) {
              if(padStatus.valvesArmed == 1) {
                valveOpen(4);
                sendFullStatus();
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Confirmed valve open");
                RadioSend();   
              } else {
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Pad haz valves are not armed");
                RadioSend();                
              }
            }
            if(strncmp("#O5,",theWord,4) == 0) {
              if(padStatus.valvesArmed == 1) {
                valveOpen(5);
                sendFullStatus();
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Confirmed valve open");
                RadioSend();   
              } else {
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Pad haz valves are not armed");
                RadioSend();                
              }
            }
            if(strncmp("#O6,",theWord,4) == 0) {
              valveOpen(6);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve open");
              RadioSend();              
            }
            if(strncmp("#O7,",theWord,4) == 0) {
              valveOpen(7);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve open");
              RadioSend();              
            }
            if(strncmp("#O8,",theWord,4) == 0) {
              valveOpen(8);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve open");
              RadioSend();              
            }   
            if(strncmp("#O9,",theWord,4) == 0) {
              valveOpen(9);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve open");
              RadioSend();              
            }          
          }  // end valves open
         //-------------- VALVES CLOSE  --------------------------------
          if(strncmp("#C",theWord,2) == 0) {  //evaluate close valve command
            if(strncmp("#C1,",theWord,4) == 0) {
              valveClose(1);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve closed");
              RadioSend();              
            }
            if(strncmp("#C2,",theWord,4) == 0) {
              valveClose(2);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve closed");
              RadioSend();              
            }
            if(strncmp("#C3,",theWord,4) == 0) {
              valveClose(3);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve closed");
              RadioSend();              
            }
            if(strncmp("#C4,",theWord,4) == 0) {
                valveClose(4);
                if (padStatus.processF1 == 2) padStatus.processF1 = 0; // stop fill processes if running
                sendFullStatus();
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Confirmed valve closed");
                RadioSend();   
            }
            if(strncmp("#C5,",theWord,4) == 0) {
                valveClose(5);
                if (padStatus.processO1 == 2) padStatus.processO1 = 0; // stop fill processes if running
                sendFullStatus();
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Confirmed valve closed");
                RadioSend();   
            }
            if(strncmp("#C6,",theWord,4) == 0) {
              valveClose(6);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve closed");
              RadioSend();              
            }
            if(strncmp("#C7,",theWord,4) == 0) {
              valveClose(7);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve closed");
              RadioSend();              
            }
            if(strncmp("#C8,",theWord,4) == 0) {
              valveClose(8);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve closed");
              RadioSend();              
            }       
            if(strncmp("#C9,",theWord,4) == 0) {
              valveClose(9);
              sendFullStatus();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed valve closed");
              RadioSend();              
            } 

                 
          }  // end valves open
          //-------------- ARM ALL  --------------------------------
          if (strncmp("#ARM1,",theWord,6) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action ARM ALL"));
            if(padStatus.padHot == 1) {
              padStatus.valvesArmed = 1;
              padStatus.armedState = 1;
              sendFullStatus(); 
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed: Pad is Armed");
              RadioSend();       
            } else {
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Not Arming. Pad is not hot.");   
            }
          }
          if (strncmp("#ARM0,",theWord,6) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action DISARM ALL"));
            padStatus.valvesArmed = 0;
            padStatus.armedState = 0;
            //also the haz valves
            mainValveOff();
            valveClose(4);
            valveClose(5);
            padStatus.mainValvesState = 0;
            padStatus.mainValvesOn = 0;
            padStatus.mainValvesOff = 1;
            if (padStatus.processF1 == 2) padStatus.processF1 = 0; // stop fill processes if running
            if (padStatus.processO1 == 2) padStatus.processO1 = 0; // stop fill processes if running

            sendFullStatus(); 
            strcpy(radioHeader, "#M");
            strcpy(radioMessage, "Confirmed: Pad is Disarmed");
            RadioSend();           
          }
          //-------------- RECOVERY  --------------------------------
          // NOTE:  High is off
          if (strncmp("#RON,",theWord,5) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action recovery On"));
            digitalWrite(pin_recoveryOff, LOW);
            digitalWrite(pin_recoveryOn, HIGH);
            threads.delay(200);
            digitalWrite(pin_recoveryOn, LOW);
            padStatus.recoveryPower = 1;
            sendFullStatus(); 
            strcpy(radioHeader, "#M");
            strcpy(radioMessage, "Recovery Power is ON");
            RadioSend();           
          }
          if (strncmp("#ROFF,",theWord,6) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action recovery Off"));
            digitalWrite(pin_recoveryOn, LOW);
            digitalWrite(pin_recoveryOff, HIGH);
            threads.delay(200);
            digitalWrite(pin_recoveryOff, LOW);
            padStatus.recoveryPower = 0;
            sendFullStatus(); 
            strcpy(radioHeader, "#M");
            strcpy(radioMessage, "Recovery Power is OFF");
            RadioSend();           
          }
          //-------------- MAIN VALVES  --------------------------------
          if (strncmp("#MON,",theWord,5) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action Main Valves Open"));
              if(padStatus.valvesArmed == 1) {
                mainValveOn();
                padStatus.mainValvesState = 1;
                padStatus.mainValvesOn = 1;
                padStatus.mainValvesOff = 0;
                sendFullStatus(); 
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Confirmed: Main Valves OPEN");
                RadioSend();  
              } else {
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Pad is not armed");
                RadioSend();  
              }
          }
          if (strncmp("#MOFF,",theWord,6) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action Main Valves closed"));
                mainValveOff();
                padStatus.mainValvesState = 0;
                padStatus.mainValvesOn = 0;
                padStatus.mainValvesOff = 1;
                sendFullStatus(); 
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Confirmed: Main Valves CLOSED");
                RadioSend();         
          }
          //-------------- IGNITER --------------------------------
          if (strncmp("#FIRE,",theWord,6) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action FIRE"));
              if(padStatus.armedState == 1) {
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Confirmed: Igniter FIRE!");
                RadioSend();
                digitalWrite(pin_igniterFIRE, HIGH);
                timers.igniterTimer = millis() + 3000;
              } else {
                strcpy(radioHeader, "#M");
                //zzz check pad hot
                strcpy(radioMessage, "Igniter is not armed");
                RadioSend();  
              }
          }
          //-------------- ABORT --------------------------------
          if (strncmp("#ABORT,",theWord,7) == 0) { 
              // open rocket vent valves and close all other valves
              // zzz - disarm?
              if(serialDebug) Serial.println(F("Got radio action ABORT"));
              valveOpen(7);
              valveOpen(8);
              valveClose(1);
              valveClose(2);
              valveClose(3);
              valveClose(4);
              valveClose(5);
              valveClose(6);
              valveClose(9);
              padStatus.processF1 = 0; // stop fill processes if running
              padStatus.processO1 = 0; // stop fill processes if running
              mainValveOff();
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed: ABORT in process");
              RadioSend();               
              padStatus.mainValvesState = 0;
              padStatus.mainValvesOn = 0;
              padStatus.mainValvesOff = 1;
              padStatus.valvesArmed = 0;
              padStatus.armedState = 0;              
              sendFullStatus(); 
          }

          //-------------- PROCESS C1 --------------------------------
          if (strncmp("#PC1,",theWord,5) == 0) { 
              // open rocket vent valves and close all other valves
              // zzz - disarm?
              if(serialDebug) Serial.println(F("Got radio action PC1"));
              valveClose(1);
              valveClose(2);
              valveClose(3);
              valveClose(4);
              valveClose(5);
              valveClose(6);
              valveClose(7);
              valveClose(8);   
              valveClose(9);           
              mainValveOff();    
              padStatus.mainValvesState = 0;
              padStatus.mainValvesOn = 0;
              padStatus.mainValvesOff = 1;
              padStatus.processC1 = 1;
              sendFullStatus(); 
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Confirmed - All Valves Closed");
              RadioSend();        
          }

          //-------------- PROCESS F1 --------------------------------
          // Pressurize fuel tank
          if (strncmp("#PF1,",theWord,5) == 0) { 
              // fuel pressurization -- haz command
              if(serialDebug) Serial.println(F("Got radio action PF1"));
                if(padStatus.valvesArmed == 1) {
                  if(padStatus.PFL < padStatus.fuelFill) {  // check for fill limit exceeded
                    padStatus.processF1 = 2;
                    //zzz close all other valves
                    valveOpen(4);  // open the fill valve
                    sendFullStatus(); 
                    strcpy(radioHeader, "#M");
                    strcpy(radioMessage, "Fuel pressurization started");
                    RadioSend();                   
                  } else {
                    strcpy(radioHeader, "#M");
                    strcpy(radioMessage, "Denied - Fuel already over pressure");
                    RadioSend();  
                  }
              } else {
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Pad is not armed");
                RadioSend();              
              }
          }

          //-------------- STOP PROCESS F1 --------------------------------
          // Stop Pressurize fuel tank
          if (strncmp("#PF1S,",theWord,6) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action PF1S stop"));
              // fuel pressurization -- not haz command
              if(padStatus.processF1 == 2) {
                 valveClose(4);
                 padStatus.processF1 = 0;
                 sendFullStatus(); 
                 strcpy(radioHeader, "#M");
                 strcpy(radioMessage, "Fuel pressurization canceled");
                 RadioSend(); 
                  
              } else {
                 valveClose(4); 
                 strcpy(radioHeader, "#M");
                 strcpy(radioMessage, "Fuel pressurization not running");
                 RadioSend(); 
              }
          }          

           //-------------- PROCESS F2 --------------------------------
          //  fuel line vent and pneumatic disconnect
          if (strncmp("#PF2,",theWord,5) == 0) { 
              if(serialDebug) Serial.println(F("Got radio action PF2"));
              valveOpen(1);  // open the fuel line vent
              timers.fuelLineTimer = millis() + configuration.lineVentDelay;
              padStatus.processF2 = 2;
              padStatus.fuelTankDisconnect = 1;
              sendFullStatus(); 
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Fuel disconnect started");
              RadioSend();                   
          }         

          //-------------- PROCESS OX1 --------------------------------
          // Pressurize Ox tank
          
          if (strncmp("#PO1,",theWord,5) == 0) { 
              // ox pressurization -- haz command
              if(serialDebug) Serial.println(F("Got radio action PO1"));
                if(padStatus.valvesArmed == 1) {
                  if(padStatus.POX < padStatus.oxFill) {  // check for fill limit exceeded
                    padStatus.processO1 = 2;
                    //zzz close all other valves
                    valveOpen(5);  // open the fill valve
                    sendFullStatus(); 
                    strcpy(radioHeader, "#M");
                    strcpy(radioMessage, "Ox pressurization started");
                    RadioSend();                   
                  } else {
                    strcpy(radioHeader, "#M");
                    strcpy(radioMessage, "Denied - Ox already over pressure");
                    RadioSend();  
                  }
              } else {
                strcpy(radioHeader, "#M");
                strcpy(radioMessage, "Pad is not armed");
                RadioSend();              
              }
          }

          //-------------- STOP PROCESS O1 --------------------------------
          // Stop Pressurize ox tank
          if (strncmp("#PO1S,",theWord,6) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action PO1S stop"));
              // Ox pressurization -- not haz command
              if(padStatus.processO1 == 2) {
                 valveClose(5);
                 padStatus.processO1 = 0;
                 sendFullStatus(); 
                 strcpy(radioHeader, "#M");
                 strcpy(radioMessage, "Ox pressurization canceled");
                 RadioSend(); 
              } else {
                 valveClose(5); 
                 strcpy(radioHeader, "#M");
                 strcpy(radioMessage, "Ox pressurization not running");
                 RadioSend(); 
              }
          }   

          //-------------- PROCESS O2 --------------------------------
          //  Ox line vent and pneumatic disconnect
          if (strncmp("#PO2,",theWord,5) == 0) { 
              if(serialDebug) Serial.println(F("Got radio action PO2"));
              valveOpen(3);  // open the fuel line vent
              timers.oxLineTimer = millis() + configuration.lineVentDelay;
              padStatus.processO2 = 2;
              padStatus.oxTankDisconnect = 1;
              sendFullStatus(); 
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Ox disconnect started");
              RadioSend();                   
          }

          //-------------- ZERO PTs  --------------------------------
          if (strncmp("#ZPOX",theWord,5) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action POX zero"));
            working.POXzero = true;
            strcpy(radioHeader, "#M");
            strcpy(radioMessage, "Processing Ox PT zero");
            RadioSend();                   
          }
          if (strncmp("#ZPFL",theWord,5) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action PFL zero"));
            working.PFLzero = true;
            strcpy(radioHeader, "#M");
            strcpy(radioMessage, "Processing Fuel PT zero");
            RadioSend();                   
          }
          if (strncmp("#ZPPS",theWord,5) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action PPS zero"));
            working.PPSzero = true;
            strcpy(radioHeader, "#M");
            strcpy(radioMessage, "Processing Pad Spare PT zero");
            RadioSend();                   
          }
         if (strncmp("#ZPRS",theWord,5) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action PRS zero"));
            working.PRSzero = true;
            strcpy(radioHeader, "#M");
            strcpy(radioMessage, "Processing Rocket Spare PT zero");
            RadioSend();                   
          }
          

                      
          newWord = 0;
          strcpy(theWord,"");
      }
}

void resetAll() {
              
      strcpy(radioHeader, "#M");
      strcpy(radioMessage, "Process Reset Successful");
      RadioSend();  
      valveClose(1);
      valveClose(2);
      valveClose(3);
      valveClose(4);
      valveClose(5);
      valveClose(6);
      valveClose(7);
      valveClose(8);   
      valveClose(9);           
      mainValveOff();    
      padStatus.mainValvesState = 0;
      padStatus.mainValvesOn = 0;
      padStatus.mainValvesOff = 1;
      padStatus.valvesArmed = 0;
      padStatus.armedState = 0;
      padStatus.processC1 = 0;
      padStatus.processF1 = 0;
      padStatus.processF2 = 0;
      padStatus.processO1 = 0;
      padStatus.processO2 = 0;
      padStatus.processAbort = 0;
      padStatus.processAC = 0;
      padStatus.process8 = 0;
      padStatus.fuelTankDisconnect = 0;
      padStatus.oxTankDisconnect = 0;
      sendFullStatus(); 
      
}

void valveOpen(int theV) {
  if(theV == 1) { digitalWrite(pin_valve1, HIGH); padStatus.valveOne = 1;}
  if(theV == 2) { digitalWrite(pin_valve2, HIGH); padStatus.valveTwo = 1;}
  if(theV == 3) { digitalWrite(pin_valve3, HIGH); padStatus.valveThree = 1;}
  if(theV == 4) { digitalWrite(pin_valve4, HIGH); padStatus.valveFour = 1;}
  if(theV == 5) { digitalWrite(pin_valve5, HIGH); padStatus.valveFive = 1;}
  if(theV == 6) { digitalWrite(pin_valve6, HIGH); padStatus.valveSix = 1;}
  if(theV == 7) { digitalWrite(pin_valve7, HIGH); padStatus.valveSeven = 1;}
  if(theV == 8) { digitalWrite(pin_valve8, HIGH); padStatus.valveEight = 1;}  
  if(theV == 9) { digitalWrite(pin_valve9, HIGH); padStatus.valveNine = 1;}  
}

void valveClose(int theV) {
  if(theV == 1) { digitalWrite(pin_valve1, LOW); padStatus.valveOne = 0;}
  if(theV == 2) { digitalWrite(pin_valve2, LOW); padStatus.valveTwo = 0;}
  if(theV == 3) { digitalWrite(pin_valve3, LOW); padStatus.valveThree = 0;}
  if(theV == 4) { digitalWrite(pin_valve4, LOW); padStatus.valveFour = 0;}
  if(theV == 5) { digitalWrite(pin_valve5, LOW); padStatus.valveFive = 0;}
  if(theV == 6) { digitalWrite(pin_valve6, LOW); padStatus.valveSix = 0;}
  if(theV == 7) { digitalWrite(pin_valve7, LOW); padStatus.valveSeven = 0;}
  if(theV == 8) { digitalWrite(pin_valve8, LOW); padStatus.valveEight = 0;} 
  if(theV == 9) { digitalWrite(pin_valve9, LOW); padStatus.valveNine = 0;} 
}


void parseit(char *record, char arr[20][20]) {

  const byte segmentSize = 19 ;  // Segments longer than this will be skipped
  char scratchPad[segmentSize + 1]; // A buffer to pull the progmem bytes into for printing/processing
  int i = 0;
  // declare three pointers to the progmem string
  char * nextSegmentPtr = record; // points to first character of segment
  char * delimiterPtr = record;   // points to detected delimiter character, or eos NULL
  char * endOfData = record + strlen(record); // points to last character in string
  byte len; // number of characters in current segment
  while (1) {
    delimiterPtr = strchr_P(nextSegmentPtr, ','); // Locate target character in progmem string.
    len = delimiterPtr - nextSegmentPtr;
    if (delimiterPtr == nullptr) { // Hit end of string
      len = endOfData - nextSegmentPtr;
    }
    if (len <= segmentSize) {
      memcpy_P(scratchPad, nextSegmentPtr, len);
      scratchPad[len] = '\0'; // Append terminator to extracted characters.
      strcpy(arr[i],scratchPad); 
    }
    else {
      strcpy(arr[i],"overflow");       
    }
    if (delimiterPtr == nullptr) { // ----- Exit while loop here -----
      break;
    }
    i++;
    nextSegmentPtr = nextSegmentPtr + len + 1;
  } // end while  
}

void append(char* s, char c) {  //used to concat char to char array
        int len = strlen(s);
        s[len] = c;
        s[len+1] = '\0';
}

void mainValveOff() {

  // Turn MainValve on to low and energize main valve off. Leave HIGH if configured.

  digitalWrite(pin_mainValveOn, LOW);
  digitalWrite(pin_mainValveOff, HIGH);
  if(configuration.holdMainOff == 0) {
      padStatus.mainOff = 2;
      timers.mainOff = millis() + local.mainHold;
  }
}

void mainValveOn() {

  // Turn MainValve off to low and energize main valve on. Leave HIGH if configured.

  digitalWrite(pin_mainValveOff, LOW);
  digitalWrite(pin_mainValveOn, HIGH);
  if(configuration.holdMainOn == 0) {
      padStatus.mainOn = 2;
      timers.mainOn = millis() + local.mainHold;
  }
}











//***********************************************************        DATA LOGGING THREAD    ************************************************************
//***********************************************************        DATA LOGGING THREAD     ************************************************************
//***********************************************************        DATA LOGGING THREAD     ************************************************************


void DAQ_thread() {

  char theFilename[40];
  char fileLine[100];
  File logFile;
  File cfile;
  unsigned long sTimeout = 0;
  unsigned long nextTime = 0;
  unsigned long hardSave = 0;
  unsigned long pulseCheck = 0;
  elapsedMillis fileClock = 0;
  
  //Initialize the flash memory
  if(serialDebug) Serial.println(F("Initializing Flash Memory"));
  if (!myfs.begin()) {
        if(serialDebug) Serial.println(F("Flash Mount Failed"));
        errors.flashMem = 1;
  } else {
    if(serialDebug) Serial.println(F("Flash Mount Success"));
     errors.flashMem = 0; //healthy
  }
  
  threads.delay(1000); // wait for sampling to start
  if(serialDebug) Serial.println(">>> Starting Data Logging Loop...");

  // =======================================================================  LOGGING MAIN LOOP  =============== //
  while(1) { 

    //------------------- Update watchdog pulse to make sure thread not crashed -------------------------
    if(millis() > pulseCheck) {
      pulseCheck = millis() + 500;
      timers.loggingPulse = millis() + 2000;
    }

    //------------------- Recieved Erase Command from Remote -------------------------
    if(working.flashFormat == 1) {
      myfs.quickFormat();
      threads.delay(100);
      writeConfig(); // reset the config file after format
      working.flashFormat = 0;
    }

    //----------------------------  START IT ----------------------
    if(working.DAQstart == 1) {
      if(working.DAQrecording == 0) {
        // start new loggin session and file
        working.DAQrecording = 1;
        working.DAQstart = 0;
        strcpy(theFilename,"");
        strcpy(theFilename,"/");
        strcat(theFilename,working.DAQfilename);
        logFile = myfs.open(theFilename, FILE_WRITE);
        strcpy(fileLine, "Milliseconds,");
        // determine headers based on enabled PTs
        if(configuration.POXenabled == 1) {
          strcat(fileLine, "Ox");
          if(configuration.PFLenabled == 1 || configuration.PPSenabled == 1 || configuration.PRSenabled == 1) strcat(fileLine, ",");
        }
        if(configuration.PFLenabled == 1) {
          strcat(fileLine, "Fuel");
          if(configuration.PPSenabled == 1 || configuration.PRSenabled == 1) strcat(fileLine, ",");
        }
        if(configuration.PPSenabled == 1) {
          strcat(fileLine, "PSpare");
          if(configuration.PRSenabled == 1) strcat(fileLine, ",");
        }
        if(configuration.PRSenabled == 1) strcat(fileLine, "RSpare");
        logFile.println(fileLine); // write the header file
        sTimeout = millis() + local.DAQsuperTimeout;
        hardSave = millis() + local.DAQflush;
        fileClock = 0;
      } else {
        working.DAQstart = 0; // error call for start again
      }
    }
    
    //----------------------------  STOP IT ----------------------
    if(working.DAQstart == 2) {  //stop
      if(working.DAQrecording == 1) {
        // stop the presses
        working.DAQrecording = 0;
        working.DAQstart = 0;        
        logFile.close();
      } else {
        working.DAQstart = 0; // error call for stop again
      }
    }

     if(millis() > sTimeout && working.DAQrecording == 1) {
       working.DAQstart = 2; // stop
       strcpy(radioHeader, "#DOFF");
       strcpy(radioMessage, "DAQ Stop");  
       RadioSend();  
       strcpy(radioHeader, "#M");
       strcpy(radioMessage, "DAQ Timeout stop after six minutes");  
       RadioSend();        
     }

     //----------------------------  RECORD IT ----------------------
     if(working.DAQrecording == 1 && millis() >= nextTime) {
       nextTime = millis() + local.logRate;
       logFile.print(fileClock);logFile.print(",");
       if(configuration.POXenabled == 1) {
        logFile.print(padStatus.POX);
        if(configuration.PFLenabled == 1 || configuration.PPSenabled == 1 || configuration.PRSenabled == 1) logFile.print(",");
        }
       if(configuration.PFLenabled == 1) {
        logFile.print(padStatus.PFL);
        if(configuration.PPSenabled == 1 || configuration.PRSenabled == 1) logFile.print(",");
        }
       if(configuration.PPSenabled == 1) {
         logFile.print(padStatus.PPS);
         if(configuration.PRSenabled == 1) logFile.print(",");
        }
       if(configuration.PRSenabled == 1) logFile.print(padStatus.PRS);   
       logFile.println(" "); 
     }
     threads.yield();

     //----------------------------  HARD SAVE IT ----------------------
     if(working.DAQrecording == 1 && millis() >= hardSave) {
       hardSave = millis() + local.DAQflush;
       logFile.close();
       logFile= myfs.open(theFilename, FILE_WRITE);
       // a logFile.flush() probably does the same thing, but not sure.
     }    

     //-------------------------------------------------------------------  FTP FILE SEND FLASH PROCESSES -------------------------------------

     //----------------------------  DIR FILES REQUEST ----------------------
     if(working.requestDir) {

         // ----------- Put Directory contents into a temp file 
         File theDir = myfs.open("/");
         myfs.remove("/tempdir.tmp");
         File tempDir = myfs.open("/tempdir.tmp", FILE_WRITE);
         int fcount = 0;
         uint64_t fSize = 0;
         
          while (true) {
            File entry = theDir.openNextFile();
            if (!entry) {
              // no more files
              tempDir.print("!");
              if(serialDebug) {Serial.print("No more files to read. Total is: ");Serial.println(fcount);}
              entry.close();
              break;
            }
            if(serialDebug) Serial.println(entry.name());
            
            char *ptr = strstr(entry.name(), "config"); // skip config file
            char *ptr2 = strstr(entry.name(), "tmp"); // skip tmp files
            if (!ptr && !ptr2) {      
              fcount++;
              fSize += entry.size();
              tempDir.print(entry.name());
              tempDir.print(",");
              tempDir.print(entry.size(), DEC);
              tempDir.print(",");
            } else {
              if(serialDebug) Serial.println("Skipping Config or Temp");
            }
            entry.close();
          } // end files evaluate loop
          
        tempDir.close();
        theDir.close();
        
        // now read the file to get the total packets 
        if(fcount > 0) {
         working.dirFileCount = fcount;
         char tmpName[30];
         strcpy(tmpName,"/tempdir.tmp");
         int theBytes = fileLength(tmpName);
         if(theBytes > 0) {
           float tFloat = (float) theBytes / (float) local.FTPsize;  // packet length
           working.totalPackets = (int) tFloat + 1;            
         } else {
           working.totalPackets = 0;
         }
        }
        working.requestDir = false;
        threads.yield();
    }

    //----------------------------  START FILE REQUEST ----------------------
     if(working.getFile) {        
        // read the file to get the total packets 
         int theBytes = fileLength(working.sendFile);
         if(theBytes > 0) {
           float tFloat = (float) theBytes / (float) local.FTPsize;  // packet length 
           working.totalPackets = (int) tFloat + 1;            
         } else {
           working.totalPackets = 0;
         }
        working.getFile = false;
        threads.yield();
    }

    //----------------------------  Send a chunk of file back to the remote iPad  ----------------------

    if(working.sendFlag) {
      // populates working.sendChunk with part of a file
      // each chunk/packet is local.FTPsize bytes
      int bufCount = 0;
      cfile.close(); // just incase
      cfile = myfs.open(working.sendFile, FILE_READ);
      if (cfile) {
        if(serialDebug) Serial.println("Reading file chunk to send");
      } else {
        if(serialDebug) Serial.println("ERROR OPENING FILE");
        //zzz figure out error handling
        return;
      }
      int seekStart = ((working.currentPacket - 1) * local.FTPsize);
      int seekEnd = working.currentPacket * local.FTPsize;
      strcpy(working.sendChunk, "");
      char fbyte;
      int posCount = 0;
      while(cfile.available()) {
       fbyte = cfile.read();
       if(bufCount >= seekStart && bufCount < seekEnd) {
          working.sendChunk[posCount] = fbyte;
          working.sendChunk[posCount + 1] = '\0';
          posCount++;
       }
       bufCount++;
       if(bufCount > seekEnd) break;
      }
      cfile.close();
      working.sendFlag = false;
    }
    
     
   threads.yield();
  }

} // end DAQ Logging Thread   ================================================================================



int fileLength(char cname[30]) {
  
    int bufCount = 0;
    File file = myfs.open(cname, FILE_READ);
    if (file) {
      if(serialDebug) Serial.println("Determining file length");
    } else {
      if(serialDebug) Serial.println("ERROR OPENING FILE");
      return 0;
    }

    while(file.available()) {
     bufCount++;
     file.read();
    }
    file.close();
    if(serialDebug) {Serial.print("Length is: ");Serial.println(bufCount);}
    return bufCount;
    threads.yield();
}



//***********************************************************        SAMPLE THREAD    ************************************************************
//***********************************************************        SAMPLE THREAD     ************************************************************
//***********************************************************        SAMPLE THREAD     ************************************************************

/* Dedicated thread for sampling the ADCs
 *  
 ADC A:
   0 = Main Voltage through opAmp (/ 10)
   1 = Haz Voltage through opAmp (/ 10)
   2 = N/A
   3 = Pad Spare PT (3 pin JST)
 ADC B:
   0 = Rocket Fuel Pressure (P3)
   1 = Rocket Ox Pressure (P2)
   2 = N/A
   3 = Rocket Spare PT (#4)
 */


void sample_thread() {

  if (serialDebug) Serial.println("Init Sampling Thread...");
  threads.delay(10);
  uint8_t tmpHealth = 0;
  errors.ADC = 0;

  // ADS1115 
  Wire.begin();
  threads.delay(1);
  ads1115a.reset();
  threads.delay(100);
  ads1115b.reset();
  threads.delay(100);
  ads1115a.setDeviceMode(ADS1115_MODE_SINGLE);
  ads1115a.setDataRate(ADS1115_DR_860_SPS);
  ads1115a.setPga(ADS1115_PGA_6_144);
  tmpHealth = ads1115a.healthTest(); // reads config
  if(tmpHealth != 227) errors.ADC = 1; 
  threads.delay(100);
  ads1115b.setDeviceMode(ADS1115_MODE_SINGLE);
  ads1115b.setDataRate(ADS1115_DR_860_SPS);
  ads1115b.setPga(ADS1115_PGA_6_144);
  tmpHealth = ads1115b.healthTest(); // reads config
  if(tmpHealth != 227) errors.ADC = 1; 
  threads.delay(100); 
  Wire.setClock(400000); 
  threads.delay(100);  

  unsigned long mainVoltageTimer = 0;
  unsigned long hazVoltageTimer = 0;
  unsigned long padSparePTtimer = 0;
  unsigned long oxPTtimer = 0;
  unsigned long fuelPTtimer = 0;
  unsigned long rocketSparePTtimer = 0;
  unsigned long pulseCheck = 0;
  elapsedMicros adcTimer = 0;
  float valueADC = 0.0;
  
  while(1) {  //***********************  MASTER SAMPLE LOOP ********************************************

    //------------------- Update watchdog pulse to make sure thread not crashed -------------------------
    if(millis() > pulseCheck) {
      pulseCheck = millis() + 500;
      timers.samplePulse = millis() + 2000;
    }

    // ----------------------------------  Check Main Voltage  ------------------------
    if(millis() > mainVoltageTimer) {
      ads1115a.setMultiplexer(ADS1115_MUX_AIN0_GND);
      ads1115a.startSingleConvertion();
      adcTimer = 0;
      while(adcTimer < 1000) {}; //wait 1000 micros for read - Q ver of ADS1115
      valueADC = ads1115a.readConvertedValue();
      padStatus.mainVolts = valueADC / 100; // opAmp voltage divider x10
      mainVoltageTimer = millis() + ADCvalue.mainVoltageInt;
    }


    // ----------------------------------  Check Haz Voltage  ------------------------
    if(millis() > hazVoltageTimer) {
      ads1115a.setMultiplexer(ADS1115_MUX_AIN1_GND);
      ads1115a.startSingleConvertion();
      adcTimer = 0;
      while(adcTimer < 1000) {}; //wait 1000 micros for read - Q ver of ADS1115
      valueADC = ads1115a.readConvertedValue();
      padStatus.hazVolts = valueADC / 100; // opAmp voltage divider x10
      hazVoltageTimer = millis() + ADCvalue.hazVoltageInt;
    }

    // ----------------------------------  Check Rocket Fuel Pressure  ------------------------
    if(millis() > fuelPTtimer && configuration.PFLenabled == 1) {
      ads1115b.setMultiplexer(ADS1115_MUX_AIN0_GND);
      ads1115b.startSingleConvertion();
      adcTimer = 0;
      while(adcTimer < 1000) {}; //wait 1000 micros for read - Q ver of ADS1115
      valueADC = ads1115b.readConvertedValue(); // v x 1000    
      if(valueADC < (float) 200) {
        padStatus.PFL = 99999; // error state below .2v
      } else {
        valueADC = valueADC / (float) 1000.0;
        if(working.PFLzero) { configuration.PFLzero = valueADC; working.PFLzero = false;} // zero command
        valueADC = valueADC - configuration.PFLzero; // remove .5v base
        valueADC = valueADC / (float) 4.0 * (float) configuration.PFLrange;
        padStatus.PFL = (int) valueADC; // the PSI
      }
      
      fuelPTtimer = millis() + ADCvalue.fuelPTint;
    }

    // ----------------------------------  Check Rocket Ox Pressure  ------------------------
    if(millis() > oxPTtimer && configuration.POXenabled == 1) {
      ads1115b.setMultiplexer(ADS1115_MUX_AIN1_GND);
      ads1115b.startSingleConvertion();
      adcTimer = 0;
      while(adcTimer < 1000) {}; //wait 1000 micros for read - Q ver of ADS1115
      valueADC = ads1115b.readConvertedValue(); // v x 1000
      if(valueADC < 200) {
        padStatus.POX = 99999; // error state below .2v
      } else {
        valueADC = valueADC / (float) 1000.0;
        if(working.POXzero) { configuration.POXzero = valueADC; working.POXzero = false;} // zero command
        valueADC = valueADC - configuration.POXzero; // remove .5v base
        valueADC = valueADC / (float) 4.0 * (float) configuration.POXrange;
        padStatus.POX = (int) valueADC; 
      }
      oxPTtimer = millis() + ADCvalue.oxPTint;
    }

    // ----------------------------------  Check Pad Spare Pressure  ------------------------
    if(millis() > padSparePTtimer && configuration.PPSenabled == 1) {
      ads1115a.setMultiplexer(ADS1115_MUX_AIN3_GND);
      ads1115a.startSingleConvertion();
      adcTimer = 0;
      while(adcTimer < 1000) {}; //wait 1000 micros for read - Q ver of ADS1115
      valueADC = ads1115a.readConvertedValue(); // v x 1000
      if(valueADC < 200) {
        padStatus.PPS = 99999; // error state below .2v
      } else {
        valueADC = valueADC / (float) 1000.0;
        if(working.PPSzero) { configuration.PPSzero = valueADC; working.PPSzero = false;} // zero command
        valueADC = valueADC - configuration.PPSzero; // remove .5v base
        valueADC = valueADC / (float) 4.0 * (float) configuration.PPSrange;
        padStatus.PPS = (int) valueADC; 
      }
      oxPTtimer = millis() + ADCvalue.padSparePTint;
    }    

    // ----------------------------------  Check Rocket Spare Pressure  ------------------------
    if(millis() > rocketSparePTtimer && configuration.PRSenabled == 1) {
      ads1115b.setMultiplexer(ADS1115_MUX_AIN3_GND);
      ads1115b.startSingleConvertion();
      adcTimer = 0;
      while(adcTimer < 1000) {}; //wait 1000 micros for read - Q ver of ADS1115
      valueADC = ads1115b.readConvertedValue(); // v x 1000
      if(valueADC < 200) {
        padStatus.PRS = 99999; // error state below .2v
      } else {
        valueADC = valueADC / (float) 1000.0;
        if(working.PRSzero) { configuration.PRSzero = valueADC; working.PRSzero = false;} // zero command
        valueADC = valueADC - configuration.PRSzero; // remove .5v base
        valueADC = valueADC / (float) 4.0 * (float) configuration.PRSrange;
        padStatus.PRS = (int) valueADC; 
      }
      oxPTtimer = millis() + ADCvalue.rocketSparePTint;
    }



   threads.yield(); 
  
  } // end while

} // end sample thread



// zzz delete this test data routine


void rValue(int sens) {
  
  long rn;
  if(sens == 1) { // fuel

     rn = random(20);
     if(rn == 3) { 
      if((padStatus.PFL + 1) < 500) padStatus.PFL = padStatus.PFL + 1;
     }
     if(rn == 9) {
      if((padStatus.PFL - 1) >= 0 ) padStatus.PFL = padStatus.PFL - 1;
     }
     if(rn == 12 && padStatus.PFL > 400) {
        padStatus.PFL = padStatus.PFL - 1;
     }
     if(rn == 15 && padStatus.PFL < 350) {
        padStatus.PFL = padStatus.PFL + 1;
     }
  }

  if(sens == 2) { // fuel

     rn = random(20);
     if(rn == 3) { 
      if((padStatus.POX + 1) < 500) padStatus.POX = padStatus.POX + 2;
     }
     if(rn == 9) {
      if((padStatus.POX - 1) >= 0 ) padStatus.POX = padStatus.POX - 1;
     }
     if(rn == 12 && padStatus.POX > 400) {
        padStatus.POX = padStatus.POX - 1;
     }
     if(rn == 15 && padStatus.POX < 350) {
        padStatus.POX = padStatus.POX + 3;
     }
  }

  if(sens == 3) { // fuel

     rn = random(20);
     if(rn == 3) { 
      if((padStatus.PPS + 1) < 500) padStatus.PPS = padStatus.PPS + 1;
     }
     if(rn == 9) {
      if((padStatus.PPS - 1) >= 0 ) padStatus.PPS = padStatus.PPS - 1;
     }
     if(rn == 12 && padStatus.PPS > 400) {
        padStatus.PPS = padStatus.PPS - 1;
     }
     if(rn == 15 && padStatus.PPS < 350) {
        padStatus.PPS = padStatus.PPS + 1;
     }
  }

  if(sens == 4) { // fuel

     rn = random(20);
     if(rn == 3) { 
      if((padStatus.PRS + 1) < 500) padStatus.PRS = padStatus.PRS + 1;
     }
     if(rn == 9) {
      if((padStatus.PRS - 1) >= 0 ) padStatus.PRS = padStatus.PRS - 1;
     }
     if(rn == 12 && padStatus.PRS > 400) {
        padStatus.PRS = padStatus.PRS - 1;
     }
     if(rn == 15 && padStatus.PRS < 350) {
        padStatus.PRS = padStatus.PRS + 1;
     }
  }
  
}


//***********************************************************        CONFIG FILE    ************************************************************
//***********************************************************        CONFIG FILE     ************************************************************

void sendConfig() {
    configSentence(); // set the Config sentence
    if(serialDebug) Serial.println(F("Sending Config"));
    strcpy(radioHeader, "#CFG");
    strcpy(radioMessage, working.configString);
    RadioSend();  
}



void readConfig() {

    char result[25];
    if (!myfs.begin()) {
      if(serialDebug) Serial.println("LittleFS Flash:  *** ERROR starting Little FS ***");
      errors.flashMem = 1;
      return;
    } else {
      if(serialDebug) Serial.println("LittleFS Flash:  Successfully started");
      errors.flashMem = 0;
      unsigned long bFree = 0;
      bFree = myfs.totalSize() - myfs.usedSize();
      if(bFree < 1000000) {
        errors.flashFull = 1;
        if(serialDebug) Serial.println("Flash Logging Memory is Full");
      } else {
        errors.flashFull = 0;
        if(serialDebug) {Serial.print("Flash Memory Available: ");Serial.println(bFree);}
      }
    }
    
    char fname[32] = "/config.txt";
    //myfs.remove(fname); //uncomment to force a new file
    if(!myfs.exists(fname)) {
      if(serialDebug) Serial.println("Config file does not exist");
      writeConfig();
    } else {
      if(serialDebug) Serial.println("Config file found");
      configFile = myfs.open(fname, FILE_READ);

      strcpy(result,"");readLine(result);configuration.POXenabled = atoi(result);
      strcpy(result,"");readLine(result);configuration.POXrange = atoi(result);
      strcpy(result,"");readLine(result);configuration.POXzero = atof(result);
      strcpy(result,"");readLine(result);configuration.POXfill = atoi(result);
      strcpy(result,"");readLine(result);configuration.POXalarm = atoi(result);
      
      strcpy(result,"");readLine(result);configuration.PFLenabled = atoi(result);
      strcpy(result,"");readLine(result);configuration.PFLrange = atoi(result);
      strcpy(result,"");readLine(result);configuration.PFLzero = atof(result);
      strcpy(result,"");readLine(result);configuration.PFLfill = atoi(result);
      strcpy(result,"");readLine(result);configuration.PFLalarm = atoi(result);

      strcpy(result,"");readLine(result);configuration.PPSenabled = atoi(result);
      strcpy(result,"");readLine(result);configuration.PPSrange = atoi(result);
      strcpy(result,"");readLine(result);configuration.PPSzero = atof(result);
      strcpy(result,"");readLine(result);configuration.PPSalarm = atoi(result);

      strcpy(result,"");readLine(result);configuration.PRSenabled = atoi(result);
      strcpy(result,"");readLine(result);configuration.PRSrange = atoi(result);
      strcpy(result,"");readLine(result);configuration.PRSzero = atof(result);
      strcpy(result,"");readLine(result);configuration.PRSalarm = atoi(result);

      strcpy(result,"");readLine(result);configuration.pressureWaitTime = atoi(result);
      strcpy(result,"");readLine(result);configuration.lineVentDelay = atoi(result);
      strcpy(result,"");readLine(result);configuration.mainVoltage2412 = atoi(result);
      strcpy(result,"");readLine(result);configuration.hazVoltage2412 = atoi(result);
      strcpy(result,"");readLine(result);configuration.recoveryArmA = atoi(result);
      strcpy(result,"");readLine(result);configuration.recoveryArmB = atoi(result);
      strcpy(result,"");readLine(result);configuration.valve9 = atoi(result);
      strcpy(result,"");readLine(result);configuration.holdMainOn = atoi(result);
      strcpy(result,"");readLine(result);configuration.holdMainOff = atoi(result);
      strcpy(result,"");readLine(result);configuration.padRadioPower = atoi(result);
      strcpy(result,"");readLine(result);configuration.remoteRadioPower = atoi(result);
      strcpy(result,"");readLine(result);configuration.CPUtempAlarm = atoi(result);

      configFile.close(); 
      padStatus.oxFill = configuration.POXfill;
      padStatus.fuelFill = configuration.PFLfill;
  
    }
    printConfig();
    
    //configFile = myfs.open(fname1, FILE_READ);
}

void defaultAll() {


      configuration.POXenabled = 1;
      configuration.POXrange = 500;
      configuration.POXzero = 0.5;
      configuration.POXfill = 400;
      configuration.POXalarm = 450;
      
      configuration.PFLenabled = 1;
      configuration.PFLrange = 500;
      configuration.PFLzero = 0.5;
      configuration.PFLfill = 400;
      configuration.PFLalarm = 450;

      configuration.PPSenabled = 0;
      configuration.PPSrange = 500;
      configuration.PPSzero = 0.50;
      configuration.PPSalarm = 450;

      configuration.PRSenabled = 0;
      configuration.PRSrange = 500;
      configuration.PRSzero = 0.50;
      configuration.PRSalarm = 450;

      configuration.pressureWaitTime = 2000;
      configuration.lineVentDelay = 5000;
      configuration.mainVoltage2412 = 24;
      configuration.hazVoltage2412 = 24;
      configuration.recoveryArmA = 0;
      configuration.recoveryArmB = 1;
      configuration.valve9 = 0;
      configuration.holdMainOn = 1;
      configuration.holdMainOff = 0;
      configuration.padRadioPower = 1;
      configuration.remoteRadioPower = 1;
      configuration.CPUtempAlarm = 165;

}





void readLine(char* res) {
   char cr;
   int i = 0;
   while(true){
    cr = configFile.read();
    res[i] = cr;
    i++;
    if(cr == '\n' || i == 23){
      return;
    }
   }
}

void writeConfig() {
  // write the config using the current live or default start-up settings
  
  char fname[32] = "/config.txt";
  myfs.remove(fname);
  File configFile;
  configFile = myfs.open(fname, FILE_WRITE);
  threads.delay(10);
  
  configFile.println(configuration.POXenabled);
  configFile.println(configuration.POXrange);
  configFile.println(configuration.POXzero);
  configFile.println(configuration.POXfill);
  configFile.println(configuration.POXalarm);
  configFile.println(configuration.PFLenabled);
  configFile.println(configuration.PFLrange);
  configFile.println(configuration.PFLzero);
  configFile.println(configuration.PFLfill);
  configFile.println(configuration.PFLalarm);
  configFile.println(configuration.PPSenabled);
  configFile.println(configuration.PPSrange);
  configFile.println(configuration.PPSzero);
  configFile.println(configuration.PPSalarm);
  configFile.println(configuration.PRSenabled);
  configFile.println(configuration.PRSrange);
  configFile.println(configuration.PRSzero);
  configFile.println(configuration.PRSalarm);
  configFile.println(configuration.pressureWaitTime);
  configFile.println(configuration.lineVentDelay);
  configFile.println(configuration.mainVoltage2412);
  configFile.println(configuration.hazVoltage2412);
  configFile.println(configuration.recoveryArmA);
  configFile.println(configuration.recoveryArmB);
  configFile.println(configuration.valve9);
  configFile.println(configuration.holdMainOn);
  configFile.println(configuration.holdMainOff);
  configFile.println(configuration.padRadioPower);
  configFile.println(configuration.remoteRadioPower);
  configFile.println(configuration.CPUtempAlarm);
  configFile.close();
  if(serialDebug) Serial.println("Wrote new config.txt file");
  
}

void printConfig() {
  if(serialDebug) {
    Serial.println("CONFIGURATION: ");
    Serial.print("       POXenabled: ");Serial.println(configuration.POXenabled);
    Serial.print("         POXrange: ");Serial.println(configuration.POXrange);
    Serial.print("          POXzero: ");Serial.println(configuration.POXzero);
    Serial.print("          POXfill: ");Serial.println(configuration.POXfill);
    Serial.print("         POXalarm: ");Serial.println(configuration.POXalarm);
    
    Serial.print("       PFLenabled: ");Serial.println(configuration.PFLenabled);
    Serial.print("         PFLrange: ");Serial.println(configuration.PFLrange);
    Serial.print("          PFLzero: ");Serial.println(configuration.PFLzero);
    Serial.print("          PFLfill: ");Serial.println(configuration.PFLfill);
    Serial.print("         PFLalarm: ");Serial.println(configuration.PFLalarm);

    Serial.print("       PPSenabled: ");Serial.println(configuration.PPSenabled);
    Serial.print("         PPSrange: ");Serial.println(configuration.PPSrange);
    Serial.print("          PPSzero: ");Serial.println(configuration.PPSzero);
    Serial.print("         PPSalarm: ");Serial.println(configuration.PPSalarm);

    Serial.print("       PRSenabled: ");Serial.println(configuration.PRSenabled);
    Serial.print("         PRSrange: ");Serial.println(configuration.PRSrange);
    Serial.print("          PRSzero: ");Serial.println(configuration.PRSzero);
    Serial.print("         PRSalarm: ");Serial.println(configuration.PRSalarm);

    Serial.print("Pres TX wait time: ");Serial.println(configuration.pressureWaitTime);
    Serial.print("  Line vent delay: ");Serial.println(configuration.lineVentDelay);
    Serial.print(" Main volts 12/24: ");Serial.println(configuration.mainVoltage2412);
    Serial.print("  Haz volts 12/24: ");Serial.println(configuration.hazVoltage2412);
    Serial.print("   Recovery Arm A: ");Serial.println(configuration.recoveryArmA);
    Serial.print("   Recovery Arm B: ");Serial.println(configuration.recoveryArmB);
    Serial.print("          Valve 9: ");Serial.println(configuration.valve9);
    Serial.print(" Hold Main Vlv On: ");Serial.println(configuration.holdMainOn);
    Serial.print("Hold Main Vlv Off: ");Serial.println(configuration.holdMainOff);
    Serial.print("  Pad Radio Power: ");Serial.println(configuration.padRadioPower);
    Serial.print(" Remote Radio Pwr: ");Serial.println(configuration.remoteRadioPower);
    Serial.print(" CPU Temp Alarm-F: ");Serial.println(configuration.CPUtempAlarm);
    Serial.println("  ");
  }
}



void configSentence() {
  // NOTE: do not send PT zero values. they stay local.

  char comma[5] = ",";
  char str_int[16];

  strcpy(working.configString, "");  //zero out the string
  //------------ START ------------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.POXenabled);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.POXrange);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.POXfill);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.POXalarm);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.PFLenabled);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.PFLrange);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.PFLfill);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.PFLalarm);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.PPSenabled);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.PPSrange);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.PPSalarm);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.PRSenabled);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.PRSrange);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.PRSalarm);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.pressureWaitTime); //15
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.lineVentDelay);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.mainVoltage2412); //17
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.hazVoltage2412); //18
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.recoveryArmA); //19
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.recoveryArmB);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.valve9);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.holdMainOn);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.holdMainOff); //23
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.padRadioPower);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.remoteRadioPower);
  strcat(working.configString, str_int);strcat(working.configString, comma);
  //-------------
  strcpy(str_int, ""); sprintf (str_int, "%d" , configuration.CPUtempAlarm); //26
  strcat(working.configString, str_int);
  //-------------

}

void setConfig(char tF[35][20]) {

  configuration.POXenabled = atoi(tF[1]);
  configuration.POXrange = atoi(tF[2]);
  configuration.POXfill = atoi(tF[3]);
  configuration.POXalarm = atoi(tF[4]);
  configuration.PFLenabled = atoi(tF[5]);
  configuration.PFLrange = atoi(tF[6]);
  configuration.PFLfill = atoi(tF[7]);
  configuration.PFLalarm = atoi(tF[8]);
  configuration.PPSenabled = atoi(tF[9]);
  configuration.PPSrange = atoi(tF[10]);
  configuration.PPSalarm = atoi(tF[11]);
  configuration.PRSenabled = atoi(tF[12]);
  configuration.PRSrange = atoi(tF[13]);
  configuration.PRSalarm = atoi(tF[14]);
  configuration.pressureWaitTime = atoi(tF[15]);
  configuration.lineVentDelay = atoi(tF[16]);
  configuration.mainVoltage2412 = atoi(tF[17]);
  configuration.hazVoltage2412 = atoi(tF[18]);
  configuration.recoveryArmA = atoi(tF[19]);
  configuration.recoveryArmB = atoi(tF[20]);
  configuration.valve9 = atoi(tF[21]);
  configuration.holdMainOn = atoi(tF[22]);
  configuration.holdMainOff = atoi(tF[23]);
  configuration.padRadioPower = atoi(tF[24]);
  configuration.remoteRadioPower = atoi(tF[25]);
  configuration.CPUtempAlarm = atoi(tF[26]);

  padStatus.oxFill = configuration.POXfill;
  padStatus.fuelFill = configuration.PFLfill;

  //zzz need to invoke the config:  radio power, in progress processes, etc.

}


//Config data integrity testing
  
bool configTest(char tF[35][20]) {
  // test for valid config data
  if(zeroOne(atoi(tF[1]))) return false;
  if(PTrange(atoi(tF[2]))) return false;
  if(PTfill(atoi(tF[3]))) return false;
  if(PTfill(atoi(tF[4]))) return false;
  if(zeroOne(atoi(tF[5]))) return false;
  if(PTrange(atoi(tF[6]))) return false;
  
  if(PTfill(atoi(tF[7]))) return false;
  if(PTfill(atoi(tF[8]))) return false;  
  if(zeroOne(atoi(tF[9]))) return false;
  if(PTrange(atoi(tF[10]))) return false;
  if(PTfill(atoi(tF[11]))) return false;
  if(zeroOne(atoi(tF[12]))) return false;
  if(PTrange(atoi(tF[13]))) return false;
  if(PTfill(atoi(tF[14]))) return false;
  
  if(chkTime(atoi(tF[15]))) return false; //pressure wait time
  if(chkTime(atoi(tF[16]))) return false;
  if(chkV(atoi(tF[17]))) return false;
  if(chkV(atoi(tF[18]))) return false;
  if(zeroOne(atoi(tF[19]))) return false;
  if(zeroOne(atoi(tF[20]))) return false;
  if(zeroOne(atoi(tF[21]))) return false;
  if(zeroOne(atoi(tF[22]))) return false;
  if(zeroOne(atoi(tF[23]))) return false;
  if(oneTen(atoi(tF[24]))) return false;
  if(oneTen(atoi(tF[25]))) return false;
  if(oneThreeh(atoi(tF[26]))) return false;

  int tempOx = atoi(tF[3]);
  int tempFuel = atoi(tF[7]);
  int oxRange = atoi(tF[2]);
  int flRange = atoi(tF[6]);
  
  if((tempOx <= oxRange && tempOx > 0) && (tempFuel <= flRange && tempFuel > 0)) {
    // good values
  } else {
    return false;
  }
  return true;
}

bool zeroOne(int theT) {
  if(theT == 0 || theT == 1) {
    return false;
  } else {
    return true;
  }
}

bool oneTen(int theT) {
  if(theT > 0 && theT < 11) {
    return false;
  } else {
    return true;
  }
}

bool oneThreeh(int theT) {
  if(theT > 0 && theT < 301) {
    return false;
  } else {
    return true;
  }
}

bool PTrange(int theT) {
  if(theT > 20 && theT < 6001) {
    return false;
  } else {
    return true;
  }
}

bool PTfill(int theT) {
  if(theT > 0 && theT < 6001) {
    return false;
  } else {
    return true;
  }
}

bool chkTime(int theT) {
  if(theT > 100 && theT < 60000) {
    return false;
  } else {
    return true;
  }
}
bool chkV(int theT) {
  if(theT == 12 || theT == 24) {
    return false;
  } else {
    return true;
  }
}


//***********************************************************        FILE TRANSFER PROTOCOL    ************************************************************
//***********************************************************        FILE TRANSFER PROTOCOL     ************************************************************

// This is used to transfer files to the iPad over serial radio. It chunks data into 60 byte sentences and uses NMEA checksums 




//----------------------------------------------------------   MCU FILES AND DIRECTORY TRANSFER   ------------------------------------------------------------

void MCUfiles() { // directory of files

  char theSentence[101];
  char str_int[20];

  // send over a list of MCU files. Tricky hand-off between two threads, since logging owns the flash thread
  if(working.DAQrecording == 1) {
    strcpy(radioHeader, "#M");
    strcpy(radioMessage, "Files Denied - DAQ is Recording");
    RadioSend();
    return;
  }
  
  working.requestDir = true;  // signal logging thread
  elapsedMillis theTimeout = 0;
  while(working.requestDir == true && theTimeout < 5000) {
    // wait for a response from logging thread
    threads.yield();
  }
  if(theTimeout > 5000) {
    strcpy(radioHeader, "#M");
    strcpy(radioMessage, "Flash Read Timeout");
    RadioSend();
    return;
  }

  // send back file count and packet count and start high speed comms
  if(serialDebug) {Serial.print(F("Total Files:"));Serial.println(working.dirFileCount);}
  if(serialDebug) {Serial.print(F("Total Packets:"));Serial.println(working.totalPackets);}

  strcpy(radioHeader, "$DIR");
  strcpy(theSentence,"");
  strcpy(str_int, ""); sprintf (str_int, "%d" , working.dirFileCount);
  strcat(theSentence, str_int);strcat(theSentence, "~");
  strcpy(str_int, ""); sprintf (str_int, "%d" , working.totalPackets);
  strcat(theSentence, str_int);
  strcpy(radioMessage, theSentence); 
  FTPsend();
  working.superTimeout = 0;  // 30 second super timeout
  bool wip = true;
  working.ftpMode = true;
  // --------------------- master communication loop
  while(wip) {
    checkRadio(); // note:  redirects to ftpProcess sentences since working.ftpMode is turned on.
    if(working.superTimeout > 10000) {
      if(serialDebug) Serial.print(F("FTP Send Super Timeout Exceeded - Aborting!!"));
      strcpy(radioHeader, "#M");
      strcpy(radioMessage, "Flash Read Timeout");
      working.ftpMode = false;
      RadioSend();
      wip = false;
    }
    if(working.ftpMode == false) wip = false;
  }
  working.ftpMode = false;

  
}


void MCUget() { // directory of files

  char theSentence[61];
  char str_int[20];

  // send over a list of MCU files. Tricky hand-off between two threads, since logging owns the flash thread
  if(working.DAQrecording == 1) {
    strcpy(radioHeader, "#M");
    strcpy(radioMessage, "Transfer Denied - DAQ is Recording");
    RadioSend();
    return;
  }
  
  working.getFile = true;  // signal logging thread
  elapsedMillis theTimeout = 0;
  while(working.getFile == true && theTimeout < 10000) {
    // wait for a response from logging thread
    threads.yield();
  }
  if(theTimeout > 10000) {
    strcpy(radioHeader, "#M");
    strcpy(radioMessage, "Flash Read Timeout");
    RadioSend();
    return;
  }

  // send back file count and packet count and start high speed comms
  if(serialDebug) {Serial.print(F("Total Packets:"));Serial.println(working.totalPackets);}

  strcpy(radioHeader, "$START");
  strcpy(theSentence,"");
  strcat(theSentence, "1~"); // indicates valid file
  strcpy(str_int, ""); sprintf (str_int, "%d" , working.totalPackets);
  strcat(theSentence, str_int);
  strcpy(radioMessage, theSentence); 
  FTPsend();
  bool wip = true;
  working.ftpMode = true;
  working.superTimeout = 0;
  // --------------------- master communication loop
  while(wip) {
    checkRadio(); // note:  redirects to ftpProcess sentences since working.ftpMode is turned on.
    if(working.superTimeout > 10000) {
      if(serialDebug) Serial.print(F("FTP Send Super Timeout Exceeded - Aborting!!"));
      strcpy(radioHeader, "#M");
      strcpy(radioMessage, "Flash Read Timeout");
      working.ftpMode = false;
      RadioSend();
      wip = false;
    }
    if(working.ftpMode == false) wip = false;
  }
  working.ftpMode = false;

  
}


void FTPsend() {  
  //immediate send for high speed FTP transfer
    strcpy(radioMessageS, "");
    strcat(radioMessageS, radioHeader);
    strcat(radioMessageS, "~");    
    strcat(radioMessageS, radioMessage);      
    strcat(radioMessageS, "~!");

    digitalWrite(pin_led1, HIGH); //blink onboard light
    Serial1.print(radioMessageS);
    Serial1.flush(); //waits for outgoing transmission to complete
    digitalWrite(pin_led1, LOW);
    if(serialDebug) {Serial.print(F("FTP Radio Sent:"));Serial.println(radioMessageS);}
}


void ftpProcess() {

      int chkNew = 0;
      char theSentence[70];
      char str_int[20];
      
      if(theWord[0] == 36 && theWord[strlen(theWord)-1] == 33) {   // proper sentence recieved from radio $ ! for FTP          
          chkNew = 1;
          working.superTimeout = 0;
      } else {  // error
              newWord = 0;
              strcpy(theWord, "");
              if(serialDebug) {Serial.print("Bad RX Sentence in FTP: ");Serial.println(theWord);}
      }
      if (chkNew == 1) {  // ********************************  FTP RADIO PROCESSING  ******************************

          //-------------- RECEIVED A SEND REQUEST  --------------------------------
          if (strncmp("$SEND,",theWord,6) == 0) { 
            char tempFill[20][20]={0x0};
            parseit(theWord,tempFill);

            strcpy(working.sendFile, "/");
            strcat(working.sendFile, tempFill[1]);
            working.currentPacket = atoi(tempFill[2]);   
            elapsedMillis theTimeout = 0;
            threads.yield();
            working.sendFlag = true; 
            while(working.sendFlag && theTimeout < 5000) {
              threads.yield();
              // wait for a response from logging thread
            }
            if(theTimeout > 5000) {
              strcpy(radioHeader, "#M");
              strcpy(radioMessage, "Flash Read Timeout - Abort");
              RadioSend();
              working.ftpMode = false;
              return;
            }
            if(serialDebug) Serial.println("Sending");
            strcpy(radioHeader, "$FILE");
            strcpy(theSentence,"");
            strcpy(str_int, ""); sprintf (str_int, "%d" , working.currentPacket);
            strcat(theSentence, str_int);strcat(theSentence, "~");
            strcat(theSentence, working.sendChunk);
            strcpy(radioMessage, theSentence); 
            FTPsend();
          } // end $SEND
          
          //-------------- RECEIVED A DONE REQUEST  --------------------------------
          if (strncmp("$DONE,",theWord,6) == 0) { 
            if(serialDebug) Serial.print(F("FTP Done!"));
            working.ftpMode = false;
          }

          
      }
      newWord = 0;
      strcpy(theWord,"");
}

void testRead() { // zzz unused
  
            elapsedMillis theTimeout = 0;
            working.currentPacket = 3;
            working.sendFlag = true; 
            
            while(working.sendFlag && theTimeout < 5000) {
              threads.yield();
            }
            Serial.print("Got1:  "); Serial.println(working.sendChunk);
            if(theTimeout > 5000) Serial.println("Timeout A");
            
            theTimeout = 0;
            working.currentPacket = 2;
            working.sendFlag = true; 
            while(working.sendFlag && theTimeout < 5000) {
              threads.yield();
            }
            Serial.print("Got2:  "); Serial.println(working.sendChunk);
            if(theTimeout > 5000) Serial.println("Timeout B");

            theTimeout = 0;
            working.currentPacket = 3;
            working.sendFlag = true; 
            while(working.sendFlag && theTimeout < 5000) {
              threads.yield();
            }
            Serial.print("Got3:  "); Serial.println(working.sendChunk);
            if(theTimeout > 5000) Serial.println("Timeout C");

  
}



void example() { // zzz unused

  const char* sentence = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
  
  // Calculate the checksum
  uint8_t checksum = calculateNMEAChecksum(sentence);
  
  // Print the checksum in hexadecimal format
  if(serialDebug) Serial.print("Checksum: 0x");
  if (checksum < 0x10) {
    if(serialDebug) Serial.print("0");
  }
  if(serialDebug) Serial.println(checksum, HEX);

  
}




uint8_t calculateNMEAChecksum(const char* sentence) {  // zzz unused
  // Exclude the initial '$' character from the checksum calculation
  sentence++;

  // Initialize the XOR checksum
  uint8_t checksum = 0;

  // Calculate the XOR checksum for each character in the sentence
  while (*sentence && *sentence != '*') {
    checksum ^= *sentence;
    sentence++;
  }

  return checksum;
}



