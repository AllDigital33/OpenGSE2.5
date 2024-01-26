
/* GSE Mini Analog Remote 
 *  
 *  Production Code v1
 *  Current controller hardware version is 2.5pro
 *  
 *  Uses MCP23017 library by Bertrand Lemasle
 *  
 *  Note:  There is a hardware design error (no darlington array) on the three small status LEDs.
 *  Control them by pulling low on the GPIO to ground them. A hack but it works.
 *  
 *  
 *  valve codes:
 *  
 *  FRV = Fuel Rocket Vent
 *  FPR = Fuel Pressurize
 *  FLV = Fuel Line Vent
 *  FQD = Fuel Pnuematic Disconnect
 *  ORV = Ox Rocket Vent
 *  OPR = Ox Pressurize
 *  OLV = Ox Line Vent
 *  OQD = Ox Pnuematic Disconnect
 *  MON = Main Valves On
 *  MOF = Main valves Off
 *  
 *  other codes:
 *  
 *  REC = recovery
 *  IGN = Igniter
 *  ABT = Abort
 *  
 *  
 *  
 */

  bool serialDebug = false;  // set to true to get Serial output for debugging
  bool debugMode = false;
  bool mute = false;  // silence the buzzer for testing

  
 // Pin configuration (MCP1 = U7, MCP2 = U2)
  #define RXD1 0
  #define TXD1 1
  #define pin_arm 2
  #define pin_2FA 3
  #define pin_internalLED 13
  #define pin_voltage A14
  #define pin_radioSet 1 //M1
  #define pin_buzzer 2 //M1

  #define LED_armR 24
  #define LED_armG 12  
  #define LED_status 29   //L2
  #define LED_radio 30    //L3
  #define LED_error 31    //L4
  #define LED_BU_FP 25
  #define LED_BU_FQD 26
  #define LED_BU_LP 27
  #define LED_BU_OQD 28
  #define LED_FRV 2 //M2
  #define LED_FPR 5 //M2
  #define LED_FLV 3 //M1
  #define LED_FQD 7 //M1
  #define LED_ORV 1 //M2
  #define LED_OPR 4 //M2
  #define LED_OLV 4 //M1
  #define LED_OQD 6 //M1
  #define LED_REC 0 //M2
  #define LED_IGN 3 //M2
  #define LED_MON 5 //M1

  #define POT_FU A9
  #define POT_OX A7
  #define SW_POT_FU 32
  #define SW_POT_OX 22

  #define SW_FRV_o 41
  #define SW_FRV_c 14
  #define SW_FPR_o  6 //M2
  #define SW_FPR_c 14 //M2
  #define SW_FLV_o 13 //M1
  #define SW_FLV_c 14 //M1
  #define SW_FQD_o 12 //M2
  #define SW_FQD_c 13 //M2
  #define SW_ORV_o 39
  #define SW_ORV_c 40
  #define SW_OPR_o 34
  #define SW_OPR_c 35
  #define SW_OLV_o 11 //M1
  #define SW_OLV_c 12 //M1
  #define SW_OQD_o 10 //M2
  #define SW_OQD_c 11 //M2
  #define SW_REC_on 36
  #define SW_REC_off 37
  #define SW_MON 9 //M2
  #define SW_MOF 8 //M2
  #define BU_FQD 17
  #define BU_FP 20
  #define BU_OQD 15
  #define BU_OP 16
  #define BU_IGN 33
  #define BU_ABORT 10 //M1


// GPIO extender
  #include <Adafruit_MCP23X17.h>
  Adafruit_MCP23X17 mcp1; //U7
  Adafruit_MCP23X17 mcp2; //U2

//TM1637 Display 
  #include <TM1637TinyDisplay.h>
  #define CLK1 4
  #define DIO1 5
  #define CLK2 8
  #define DIO2 9
  #define CLK3 7
  #define DIO3 6
  #define CLK4 11
  #define DIO4 10    
//Arduino libraries
  #include "Arduino.h"
  #include "Wire.h"

// For the flash memory
  //Little FS flash
  #include <LittleFS.h>
  char buf[512] = "";
  char fname1[32] = "/testfile.txt";
  //LittleFS_QSPIFlash myfs; // RT5.1
  LittleFS_QPINAND myfs; // used for fully loaded T4.1
  File file, file1, file3;
//  #include <LittleFS_NAND.h>
  #include <stdarg.h>
  uint64_t fTot, totSize1;  
  #include <TimeLib.h>  // for logging time

// Temperature from CPU
  extern float tempmonGetTemp(void);  

// just for SD
  #include "SD.h"
  #include <SPI.h> 
  File tempFile;

  // Set up displays
  TM1637TinyDisplay display1(CLK1, DIO1);
  TM1637TinyDisplay display2(CLK2, DIO2);
  TM1637TinyDisplay display3(CLK3, DIO3);
  TM1637TinyDisplay display4(CLK4, DIO4);

  
// *** Radio Setup
  char radioHeader[7];
  char radioMessage[150];
  char radioMessageS[200];
  int newWord = 0;
  char theWord[180];

//added for send queue
  char rSend[15][300];
  byte rSendCount = 0;
  byte rSendLast = 0;
  byte rSendPos = 0;
  char vFullMessageR[300];


  struct settingsStruct {  // working variables

    int voltageAlarm = 10;
    int CPUtempAlarm = 150;

  };
  settingsStruct settings;

  struct workingStruct {  // working variables

    int voltage = 0;
    char stuck[5];
    int CPUtemp = 0;
    bool errorLED = false;
    bool errorLEDstate = false;
    bool sendArm = false;
    bool sendDisarm = false;
    bool recFlash = false;

  };
  workingStruct working;

  struct errorsStruct {
    bool count = false;
    bool radio = false;
    bool batt = false;
    bool stuck = false; // stuck button
    bool flash = false;
    bool CPUtemp = false;
    bool I2C = false;
    bool MCUcrash = false;
    bool radioTimeout = false; //Rx timeout 30 sec
    bool recFlash = false;
  };
    errorsStruct errors;

struct padConfigStruct {  // master configuration
    
    int POXenabled = 1;
    int POXrange = 500;
    int POXfill = 100;
    int POXalarm = 425;
    int PFLenabled = 1;
    int PFLrange = 500;
    int PFLfill = 100;
    int PFLalarm = 425;
    int PPSenabled = 0;
    int PPSrange = 500;
    int PPSalarm = 425;
    int PRSenabled = 0;
    int PRSrange = 500;
    int PRSalarm = 425;
    int lineVentDelay = 5000;
    int pressureWaitTime = 2000;
    int mainVoltage2412 = 24;
    int hazVoltage2412 = 24;
    int recoveryArmA = 0;
    int recoveryArmB = 1;
    int valve9 = 0;
    int holdMainOn = 1;
    int holdMainOff = 1;
    int padRadioPower = 1;
    int remoteRadioPower = 1;
    int CPUtempAlarm = 1;
    int DAQauto = 1;
    int DAQtimeout = 30000;
    bool updated = false;
};
  padConfigStruct padConfig;

struct padStatusStruct {  // Main variables sent over radio to indicate pad status
    
    int tankConfig = 2; // valid is 1, 2, 3
    bool rocketConnected = false;
    bool padHot = false; // pad safety plug in/out (haz 24v)
    bool igniterArmed = false;
    bool valvesArmed = false;
    bool padArmed = false;
    bool igniterContinuity = false;
    float mainVolts = 0.0;
    float hazVolts = 0.0;
    int pressureOne = 0;  // Ox Tank
    int pressureTwo = 0;  // Fuel Tank
    int pressureThree = 0;  // Pressure ground tank
    int pressureFour = 0; // Pressure vehicle tank
    int tankTemperature = 0; // Pressure tank temp
    bool valveOne = false; // closed = 0, open = 1
    bool valveTwo = false;
    bool valveThree = false;
    bool valveFour = false;
    bool valveFive = false;
    bool valveSix = false;
    bool valveSeven = false;
    bool valveEight = false;
    bool valveNine = false;
    bool mainValvesOn = false;
    bool mainValvesOff = false;
    bool mainValvesState = false; // 1 = open , 0 = closed
    bool tankThreeActuatorOn = false;
    bool tankThreeActuatorOff = false;
    int processC1 = 0;  // can be 0- not run, 1 - complete, 2 - running, 3-stopped
    int processF1 = 0;
    int processF2 = 0;
    int processO1 = 0;
    int processO2 = 0;
    int processAbort = 0;
    int process7 = 0;
    int process8 = 0;
    bool oxTankDisconnect = false;
    bool fuelTankDisconnect = false;
    bool pressureTankDisconnect = false;
    bool recoveryPower = false;
    bool altimeterAarmed = false;
    bool altimeterBarmed = false;
    int CPUtemp = 0;
    int oxFill = 400;
    int fuelFill = 400;
}; 
  padStatusStruct padStatus;
  char* tempArr[50]; // temp array for incoming values

struct ErrorsStruct {
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
    int BLEerror = 2;
    int BLEbatt = 2;
    int localRadio = 2;
    int iPadBatt = 2;
};
  ErrorsStruct padErrors;



  //Timers
  elapsedMillis errorTimer;
  elapsedMillis radioRxTimer;
  elapsedMillis radioTxTimer;
  elapsedMillis blinkTimer;
  elapsedMillis radioTimeout;
  elapsedMillis configTimer;  
  elapsedMillis switchTimer; // anti-bounce
  elapsedMillis armTimer;
  elapsedMillis recFlashTimer;

  

//************************************************************************************************************************************************************  SETUP
//************************************************************************************************************************************************************  SETUP

 
void setup() {


// Core Pin setup
  pinMode(pin_arm, INPUT_PULLUP);   
  pinMode(pin_2FA, INPUT_PULLUP);  
  pinMode(pin_internalLED, OUTPUT); digitalWrite(pin_internalLED, LOW);  
  pinMode(pin_voltage, INPUT);
  pinMode(POT_FU, INPUT);
  pinMode(POT_OX, INPUT);

  pinMode(LED_armR, OUTPUT); digitalWrite(LED_armR, LOW);
  pinMode(LED_armG , OUTPUT); digitalWrite(LED_armG , LOW);

/* Don't use pinmode - only when using. hardware issue.
 * pinMode(LED_status , OUTPUT); digitalWrite(LED_status , HIGH); //hw issue
  pinMode(LED_radio , OUTPUT); digitalWrite(LED_radio , HIGH); //hw issue
  pinMode(LED_error , OUTPUT); digitalWrite(LED_error , HIGH); //hw issue
*/  
  pinMode(LED_BU_FP , OUTPUT); digitalWrite(LED_BU_FP , LOW);
  pinMode(LED_BU_FQD , OUTPUT); digitalWrite(LED_BU_FQD , LOW);
  pinMode(LED_BU_LP , OUTPUT); digitalWrite(LED_BU_LP , LOW);
  pinMode(LED_BU_OQD , OUTPUT); digitalWrite(LED_BU_OQD , LOW);

  pinMode(SW_POT_FU, INPUT_PULLUP); 
  pinMode(SW_POT_OX, INPUT_PULLUP); 
  pinMode(SW_FRV_o, INPUT_PULLUP); 
  pinMode(SW_FRV_c, INPUT_PULLUP); 
  pinMode(SW_ORV_o, INPUT_PULLUP); 
  pinMode(SW_ORV_c, INPUT_PULLUP); 
  pinMode(SW_OPR_o, INPUT_PULLUP); 
  pinMode(SW_OPR_c, INPUT_PULLUP); 
  pinMode(SW_REC_on, INPUT_PULLUP); 
  pinMode(SW_REC_off, INPUT_PULLUP); 
  pinMode(BU_FQD, INPUT_PULLUP); 
  pinMode(BU_FP, INPUT_PULLUP); 
  pinMode(BU_OQD, INPUT_PULLUP); 
  pinMode(BU_OP, INPUT_PULLUP); 
  pinMode(BU_IGN, INPUT_PULLUP); 

  delay(500);
  Serial.begin(115200);
  if(CrashReport) { // capture if the MCU core dumps
    if(serialDebug) Serial.print(CrashReport);
    errors.MCUcrash = true;
  } else {
    errors.MCUcrash = false;
  }
  delay(100);
  if(serialDebug) Serial.println("GSE Mini Analog Remote Startup...");

  Wire.begin(400000);
  if (!mcp1.begin_I2C(0x27)) {
     if(serialDebug) Serial.println("Error starting MCP1");
     errors.count = true;
     errors.I2C = true;
  }
  if (!mcp2.begin_I2C(0x20)) {
     if(serialDebug) Serial.println("Error starting MCP2");
     errors.count = true;
     errors.I2C = true;
  }

// MCP1 Pin setup
  mcp1.pinMode(pin_radioSet, OUTPUT); mcp1.digitalWrite(pin_radioSet, LOW);
  mcp1.pinMode(pin_buzzer, OUTPUT); mcp1.digitalWrite(pin_buzzer, LOW);
  mcp1.pinMode(LED_FLV, OUTPUT); mcp1.digitalWrite(LED_FLV, LOW);
  mcp1.pinMode(LED_FQD, OUTPUT); mcp1.digitalWrite(LED_FQD, LOW);
  mcp1.pinMode(LED_OLV, OUTPUT); mcp1.digitalWrite(LED_OLV, LOW);
  mcp1.pinMode(LED_OQD, OUTPUT); mcp1.digitalWrite(LED_OQD, LOW);
  mcp1.pinMode(LED_MON, OUTPUT); mcp1.digitalWrite(LED_MON, LOW);
  mcp1.pinMode(SW_OLV_o, INPUT_PULLUP);
  mcp1.pinMode(SW_OLV_c, INPUT_PULLUP);
  mcp1.pinMode(SW_FLV_o, INPUT_PULLUP);
  mcp1.pinMode(SW_FLV_c, INPUT_PULLUP);
  mcp1.pinMode(BU_ABORT, INPUT_PULLUP);

// MCP2 Pin setup
  mcp2.pinMode(LED_FRV, OUTPUT); mcp2.digitalWrite(LED_FRV, LOW);
  mcp2.pinMode(LED_FPR, OUTPUT); mcp2.digitalWrite(LED_FPR, LOW);
  mcp2.pinMode(LED_ORV, OUTPUT); mcp2.digitalWrite(LED_ORV, LOW);
  mcp2.pinMode(LED_OPR, OUTPUT); mcp2.digitalWrite(LED_OPR, LOW);
  mcp2.pinMode(LED_REC, OUTPUT); mcp2.digitalWrite(LED_REC, LOW);
  mcp2.pinMode(LED_IGN, OUTPUT); mcp2.digitalWrite(LED_IGN, LOW);
  mcp2.pinMode(SW_FPR_o, INPUT_PULLUP);
  mcp2.pinMode(SW_FPR_c, INPUT_PULLUP);
  mcp2.pinMode(SW_OQD_o, INPUT_PULLUP);
  mcp2.pinMode(SW_OQD_c, INPUT_PULLUP);
  mcp2.pinMode(SW_MON, INPUT_PULLUP);
  mcp2.pinMode(SW_MOF, INPUT_PULLUP);
  mcp2.pinMode(SW_FQD_o, INPUT_PULLUP);
  mcp2.pinMode(SW_FQD_c, INPUT_PULLUP);

  // Do Diagnostics if 2FA
  if(digitalRead(pin_2FA) == LOW) {  // enter diagnostics on startup if 2FA is held down on power up
    if(serialDebug) Serial.println("Entering Diagnostics Mode");
    doDiags();
  }

  // Start-up Sequence
  getVoltage();
  startUp();
  Serial1.begin(19200, SERIAL_8N1);
  configTimer = 10000; // get config in five seconds

}

//************************************************************************************************************************************************************  MAIN LOOP
//************************************************************************************************************************************************************  MAIN LOOP

void loop() {

  // Housekeeping Loops
  if(errorTimer > 30000) checkErrors();  // check battery, CPU temp

  if(blinkTimer > 500) blinkIt();  // blink error light

  if(radioRxTimer > 0) {checkRadio(); radioRxTimer = 0;} // check radio inbound messages

  if(!padConfig.updated && configTimer > 15000) requestConfig();

  processActions(); // check for button press

  if(working.sendArm && armTimer > 15000) sendArm(); // command arm 
  if(working.sendDisarm && armTimer > 15000) sendDisarm(); // command disarm 

  if(working.recFlash && recFlashTimer > 500) flashRec();
  
  
}

//************************************************************************************************************************************************************  MAIN LOOP
//************************************************************************************************************************************************************  MAIN LOOP

void processActions() {

  /* Send requires 2Fa for everything
   * 
   * Haz requires armed -- 
   * use switchTimer for anti-bounce  
   *  
   */

  

   if(switchTimer > 500) {
    // Valve Switches
    if(digitalRead(SW_FRV_o) == LOW) if(chk2FA()) {dispIt("FRV","OPN"); aSend("#O7,33,!");switchTimer = 0;}
    if(digitalRead(SW_FRV_c) == LOW) if(chk2FA()) {dispIt("FRV","CLS"); aSend("#C7,33,!");switchTimer = 0;}
    if(mcp1.digitalRead(SW_FLV_o) == LOW) if(chk2FA()) {dispIt("FLV","OPN"); aSend("#O1,33,!");switchTimer = 0;}
    if(mcp1.digitalRead(SW_FLV_c) == LOW) if(chk2FA()) {dispIt("FLV","CLS"); aSend("#C1,33,!");switchTimer = 0;}
    if(mcp2.digitalRead(SW_FQD_o) == LOW) if(chk2FA()) {dispIt("FQD","OPN"); aSend("#O2,33,!");switchTimer = 0;}
    if(mcp2.digitalRead(SW_FQD_c) == LOW) if(chk2FA()) {dispIt("FQD","CLS"); aSend("#C2,33,!");switchTimer = 0;}
    if(digitalRead(SW_ORV_o) == LOW) if(chk2FA()) {dispIt("ORV","OPN"); aSend("#O8,33,!");switchTimer = 0;}
    if(digitalRead(SW_ORV_c) == LOW) if(chk2FA()) {dispIt("ORV","CLS"); aSend("#C8,33,!");switchTimer = 0;}
    if(mcp1.digitalRead(SW_OLV_o) == LOW) if(chk2FA()) {dispIt("OLV","OPN"); aSend("#O3,33,!");switchTimer = 0;}
    if(mcp1.digitalRead(SW_OLV_c) == LOW) if(chk2FA()) {dispIt("OLV","CLS"); aSend("#C3,33,!");switchTimer = 0;}
    if(mcp2.digitalRead(SW_OQD_o) == LOW) if(chk2FA()) {dispIt("OQD","OPN"); aSend("#O6,33,!");switchTimer = 0;}
    if(mcp2.digitalRead(SW_OQD_c) == LOW) if(chk2FA()) {dispIt("OQD","CLS"); aSend("#C6,33,!");switchTimer = 0;}

    //HAZ    
    if(mcp2.digitalRead(SW_FPR_o) == LOW) if(chk2FA()) {if(chkArmed()) {dispIt("FPR","OPN"); aSend("#O4,33,!");switchTimer = 0;}}
    if(mcp2.digitalRead(SW_FPR_c) == LOW) if(chk2FA()) {dispIt("FPR","CLS"); aSend("#C4,33,!");switchTimer = 0;} // no haz close
    if(digitalRead(SW_OPR_o) == LOW) if(chk2FA()) {if(chkArmed()) {dispIt("OPR","OPN"); aSend("#O5,33,!");switchTimer = 0;}}
    if(digitalRead(SW_OPR_c) == LOW) if(chk2FA()) {dispIt("OPR","CLS"); aSend("#C5,33,!");switchTimer = 0;} // no haz close
    if(mcp2.digitalRead(SW_MON) == LOW) if(chk2FA()) {if(chkArmed()) {dispIt("MAN","OPN"); aSend("#MON,33,!");switchTimer = 0;}}
    if(mcp2.digitalRead(SW_MOF) == LOW) if(chk2FA()) {dispIt("MAN","CLS"); aSend("#MOFF,33,!");switchTimer = 0;} // no haz close

    // REC
    if(digitalRead(SW_REC_on) == LOW) if(chk2FA()) {dispIt("REC","ON "); aSend("#RON,33,!");switchTimer = 0;}
    if(digitalRead(SW_REC_off) == LOW) if(chk2FA()) {dispIt("REC","OFF"); aSend("#ROFF,33,!");switchTimer = 0;}

    // PROCESS BUTTONS
    if(digitalRead(BU_FP) == LOW) if(chk2FA()) {if(chkArmed()) {
      if(padStatus.processF1 == 2) { // already on
         dispIt("FPR","CAN"); aSend("#PF1S,033,!");
      } else {
         dispIt("FPR","GO"); aSend("#PF1,033,!");
      }
      switchTimer = 0;}
    }
    if(digitalRead(BU_FQD) == LOW) if(chk2FA()) {if(chkArmed()) {
      if(padStatus.processF2 == 2) { // already on
         dispIt("FQD","CAN"); aSend("#PF2S,033,!");
      } else {
         dispIt("FQD","GO"); aSend("#PF2,033,!");
      }
      switchTimer = 0;}
    }   
    if(digitalRead(BU_OP) == LOW) if(chk2FA()) {if(chkArmed()) {
      if(padStatus.processO1 == 2) { // already on
         dispIt("OPR","CAN"); aSend("#PO1S,033,!");
      } else {
         dispIt("OPR","GO"); aSend("#PO1,033,!");
      }
      switchTimer = 0;}
    }  
    if(digitalRead(BU_OQD) == LOW) if(chk2FA()) {if(chkArmed()) {
      if(padStatus.processO2 == 2) { // already on
         dispIt("OQD","CAN"); aSend("#PO2S,033,!");
      } else {
         dispIt("OQD","GO"); aSend("#PO2,033,!");
      }
      switchTimer = 0;}
    }  

    //Igniter
    if(digitalRead(BU_IGN) == LOW) if(chk2FA()) {if(chkArmed()) {dispIt("IGN","FIR"); aSend("#FIRE,33,!");switchTimer = 0;}}
    //Abort
    if(mcp1.digitalRead(BU_ABORT) == LOW) if(chk2FA()) {dispIt("ABT","ABT"); aSend("#ABORT,33,!");switchTimer = 0;}

    // ARM switch active
    if (digitalRead(pin_arm) == LOW && !padStatus.padArmed) {
      if(!working.sendArm) armTimer = 15001; // force send first time
      working.sendArm = true;
    } else {
      working.sendArm = false;
    }
    // ARM switch deactivate
    if (digitalRead(pin_arm) == HIGH && padStatus.padArmed) {
      if(!working.sendDisarm) armTimer = 15001; // force send first time
      working.sendDisarm = true;
    } else {
      working.sendDisarm = false;
    }

    if(digitalRead(SW_POT_FU) == LOW) setFuelTarget();
    if(digitalRead(SW_POT_OX) == LOW) setOxTarget();
        
   } // switch timer anti-bounce
  
}

void setFuelTarget() {
  //suspend the state and all activity while adjusting
  //20 second timeout
  int pot1 = 0;
  bool finished = false;
  if(!padConfig.updated) { // abort no config rec
    pinMode(LED_error , OUTPUT);digitalWrite(LED_error, LOW); 
    displayStr("NO",1);displayStr("CFG",2);delay(2000);
    digitalWrite(LED_error, HIGH); pinMode(LED_error , INPUT);
    switchTimer = 0;
    return;
  }
  elapsedMillis sTimeout;
  displayStr("SET",1);
  float i = 0.0;
  int i2 = 0;
  while(!finished) {
    if(sTimeout > 20000) finished = true; 
    pot1 = analogRead(POT_FU);
    i= (float)((float) pot1 / 1023.0) * (float) padConfig.PFLrange;
    if((int) i != i2) {
      i2 = (int) i;
      displayInt((int) i,2);
    }
    if(digitalRead(SW_POT_FU) == LOW && sTimeout > 1000) { // confirm w anti bounce
       finished = true;
    }
  }
  if(sTimeout > 20000) { // ended due to timeout
    dispIt("TIM","OUT");
  } else { // all good
    displayStr("SND",1);
    char sendStr[15];
    strcpy(sendStr,"");
    strcpy(sendStr,"#SCFG,");
    char myTmp[12];
    itoa(padStatus.oxFill, myTmp, 10);
    strcat(sendStr,myTmp);strcat(sendStr,",");
    itoa((int) i, myTmp, 10);
    strcat(sendStr,myTmp);strcat(sendStr,"!");
    aSend(sendStr);
    switchTimer = 0;
  }
}

void setOxTarget() {

  //suspend the state and all activity while adjusting
  //20 second timeout
  int pot1 = 0;
  bool finished = false;
  if(!padConfig.updated) { // abort no config rec
    pinMode(LED_error , OUTPUT);digitalWrite(LED_error, LOW); 
    displayStr("NO",1);displayStr("CFG",2);delay(2000);
    digitalWrite(LED_error, HIGH); pinMode(LED_error , INPUT);
    switchTimer = 0;
    return;
  }
  elapsedMillis sTimeout;
  displayStr("SET",3);
  float i = 0.0;
  int i2 = 0;
  while(!finished) {
    if(sTimeout > 20000) finished = true; 
    pot1 = analogRead(POT_OX);
    i= (float)((float) pot1 / 1023.0) * (float) padConfig.PFLrange;
    if((int) i != i2) {
      i2 = (int) i;
      displayInt((int) i,4);
    }
    if(digitalRead(SW_POT_OX) == LOW && sTimeout > 1000) { // confirm w anti bounce
       finished = true;
    }
  }
  if(sTimeout > 20000) { // ended due to timeout
    displayStr("TIM",3);displayStr("OUT",4);
  } else { // all good
    displayStr("SND",3);
    char sendStr[15];
    strcpy(sendStr,"");
    strcpy(sendStr,"#SCFG,");
    char myTmp[12];
    itoa((int) i, myTmp, 10);
    strcat(sendStr,myTmp);strcat(sendStr,",");
    itoa(padStatus.fuelFill, myTmp, 10);
    strcat(sendStr,myTmp);strcat(sendStr,"!");
    aSend(sendStr);
    switchTimer = 0;
  }

  
}

bool chk2FA() {
  if(digitalRead(pin_2FA) == LOW) {
    return true;
  } else {
    pinMode(LED_error , OUTPUT);digitalWrite(LED_error, LOW); 
    displayStr("NO",1);displayStr("2FA",2);delay(500);
    digitalWrite(LED_error, HIGH); pinMode(LED_error , INPUT);
    return false;
  }  
}

bool chkArmed() {
  if(padStatus.padArmed) {
    return true;
  } 
  if(!padStatus.padArmed) {
    pinMode(LED_error , OUTPUT);digitalWrite(LED_error, LOW); 
    displayStr("NO",1);displayStr("ARM",2);delay(500);
    digitalWrite(LED_error, HIGH); pinMode(LED_error , INPUT);
    return false;
  }    
}

void dispIt(char one[5], char two[5]) {
  displayStr(one,1);
  displayStr(two,2);
}

void flashRec() {  // flash recover LED while altimeter is arming
  if(mcp2.digitalRead(LED_REC) == HIGH) {
    mcp2.digitalWrite(LED_REC, LOW);
  } else {
    mcp2.digitalWrite(LED_REC, HIGH);
  }
  recFlashTimer = 0;
}


void sendArm() {
  if(padStatus.padHot) {
    displayStr("ARM",1);displayStr("REQ",2);
    aSend("#ARM1,033,!");
  } else {
    displayStr("ARM",1);displayStr("ERR",2);
    displayStr("NOT",3);displayStr("HOT",4);
  }
  armTimer = 0;
}

void sendDisarm() {
    displayStr("ARM",1);displayStr("OFF",2);
    aSend("#ARM0,033,!");
    armTimer = 0;
}

void aSend(char rString[50]) {
     if (Serial1.available() > 0) { checkRadio(); } // don't send if inbound packets
     pinMode(LED_radio , OUTPUT);digitalWrite(LED_radio, LOW);  //blink onboard light
     Serial1.print(rString);
     Serial1.flush(); //waits for outgoing transmission to complete
     digitalWrite(LED_radio, HIGH); pinMode(LED_radio , INPUT);
}

void requestConfig() {
     if (Serial1.available() > 0) { checkRadio(); } // don't send if inbound packets
     pinMode(LED_radio , OUTPUT);digitalWrite(LED_radio, LOW);  //blink onboard light
     Serial1.print("#S,033,!"); //send all
     Serial1.flush(); //waits for outgoing transmission to complete
     digitalWrite(LED_radio, HIGH); pinMode(LED_radio , INPUT);
     configTimer = 0;
}

void blinkIt() {
    if((errors.count || padErrors.errorCount > 0) && !working.errorLED) { // turn it on
      working.errorLED = true;
      working.errorLEDstate = false;
    }
    if((!errors.count && padErrors.errorCount == 0) && working.errorLED) { // turn it off
      working.errorLED = false;
      working.errorLEDstate = false;
      pinMode(LED_error , INPUT); 
    }    
    if(errors.count || padErrors.errorCount > 0) {
      if(working.errorLEDstate) {
        working.errorLEDstate = false;
        digitalWrite(LED_error, HIGH); pinMode(LED_error , INPUT);
      } else {
        working.errorLEDstate = true;
        pinMode(LED_error , OUTPUT); digitalWrite(LED_error, LOW); 
      }
    }
    blinkTimer = 0;
}

void checkErrors() {
  if(serialDebug) {Serial.flush(); Serial.println("Checking for Errors");}
  getVoltage();
  if(working.voltage < settings.voltageAlarm) {
    errors.batt = true;
  } else {
    errors.batt = false;
  }
  
  CPUtemp();
  if(working.CPUtemp > settings.CPUtempAlarm) {
    errors.CPUtemp = true;
  } else {
    errors.CPUtemp = false;
  }
  if(errors.flash) {
    if(serialDebug) Serial.println(F("Retry Initializing Flash Memory"));
    if (!myfs.begin()) {
          if(serialDebug) Serial.println(F("Flash Mount Failed"));
          errors.flash = true;
    } else {
      if(serialDebug) Serial.println(F("Flash Mount Success"));
       errors.flash = false; //healthy
    }
    delay(200);
  }
  if(radioTimeout > 30000) {
    errors.radioTimeout = true;
  } else {
    errors.radioTimeout = false;
  }

  
  if(errors.batt || errors.flash || errors.CPUtemp || errors.radioTimeout) {

    errors.count = true;
    if(errors.batt) {
       displayStr("ERR",1);displayStr("LCL",2); 
       displayStr("BAT",3);displayInt(working.voltage,4); 
       delay(2000);
    }
    if(errors.flash) {
       displayStr("ERR",1);displayStr("LCL",2); 
       displayStr("FLS",3);displayStr("MEM",4); 
       delay(2000);
    }    
    if(errors.CPUtemp) {
       displayStr("ERR",1);displayStr("LCL",2); 
       displayStr("TMP",3);displayInt(working.CPUtemp,4); 
       delay(2000);
    }
    if(errors.radioTimeout) {
       displayStr("ERR",1);displayStr("LCL",2); 
       displayStr("NO",3);displayStr("REC",4);
       delay(2000);
    }
  } else {
    errors.count = false;
  }
  errorTimer = 0;
}

void startUp() {

  displayStr("HI",1);displayStr("GSE",2); // Hello GSE
  displayStr("BAT",3);displayInt(working.voltage,4); // show battery voltage
  delay(2000);
  if(checkStuck()) {  // a switch or button is stuck
    delay(1000);
    displayStr("ERR",1);displayStr("LCL",2); 
    displayStr("STK",3);displayStr(working.stuck,4); 
     pinMode(LED_error , OUTPUT); 
    while(checkStuck()) {
      digitalWrite(LED_error, LOW);
      delay(250);
      digitalWrite(LED_error, HIGH);
      delay(250);
      checkStuck();
    }
    // recovered
   pinMode(LED_error , INPUT);
   displayStr("HI",1);displayStr("GSE",2); // Hello GSE
   displayStr("BAT",3);displayInt(working.voltage,4); // show battery voltage   
  } // end stuck
  // start and check radio
  radioStart();
  if(errors.radio) {  // radio not reporting in
    delay(500);
    displayStr("ERR",1);displayStr("LCL",2); 
    displayStr("RAD",3);displayStr("IO",4); 
     pinMode(LED_error , OUTPUT); 
    while(1) {
      digitalWrite(LED_error, LOW);
      delay(250);
      digitalWrite(LED_error, HIGH);
      delay(250);
    }
  }
  if(errors.I2C) {  // I2C gpio failure
    delay(500);
    displayStr("ERR",1);displayStr("LCL",2); 
    displayStr("I2C",3);displayStr("ERR",4); 
     pinMode(LED_error , OUTPUT); 
    while(1) { // fatal
      digitalWrite(LED_error, LOW);
      delay(250);
      digitalWrite(LED_error, HIGH);
      delay(250);
    }
  }

  //Initialize the flash memory
  if(serialDebug) Serial.println(F("Initializing Flash Memory"));
  if (!myfs.begin()) {
        if(serialDebug) Serial.println(F("Flash Mount Failed"));
        errors.flash = true;
  } else {
    if(serialDebug) Serial.println(F("Flash Mount Success"));
     errors.flash = false; //healthy
  }
  delay(200);

 //Check CPUtemp
 CPUtemp(); 

  
}

void radioStart() {
  
    if(serialDebug) Serial.println("Initiating Radio Test...");
    mcp1.digitalWrite(pin_radioSet, HIGH); //turn radio set pin to low
    delay(50);
    Serial1.begin(9600);
    char s[5];
    strcpy(s,"");
    unsigned long testtime = millis() + 3500;
    s[0] = 0xaa;
    s[1] = 0xfa;
    s[2] = 0xaa; // aa = product model number 
    s[3] = 0x0d; //  /r termination
    s[4] = 0x0a; //  /n termination
    Serial1.println(s);
    int x = 0;
    while(x < 1) {
      radioTestRead();
      if(newWord == 1) {
        if(serialDebug) Serial.print("Radio Rx:  ");
        if(serialDebug) Serial.println(theWord);
        char *ptr = strstr(theWord, "LORA611");
        if (ptr != NULL) {
          if(serialDebug) Serial.println("Radio Set Comms Test Successful!"); 
        } else {
          errors.count = true;
          errors.radio = true;
        }
        mcp1.digitalWrite(pin_radioSet, LOW); //turn radio set pin to low
        Serial1.begin(19200);
        x = 1;
      }
      if(millis() > testtime) {
        if(serialDebug) Serial.println("Radio Timeout");
        mcp1.digitalWrite(pin_radioSet, LOW); //turn radio set pin to low
        delay(200);
        Serial1.begin(19200);
        x = 2;
        errors.count = true;
        errors.radio = true;       
      }
    }
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
       delay(1);
     }
}

void CPUtemp() {

      float CPUtemp = 0.0;
      CPUtemp  = (tempmonGetTemp() * 9.0f / 5.0f) + 32.0f;
      working.CPUtemp = (int)CPUtemp;
      if(serialDebug) Serial.print("CPU temp = ");
      if(serialDebug) Serial.println(working.CPUtemp);
      if(working.CPUtemp > settings.CPUtempAlarm) {
        errors.CPUtemp = true;
      } else {
        errors.CPUtemp = false;
      }
}

bool checkStuck() {

    if(digitalRead(SW_POT_FU) == LOW) {strcpy(working.stuck, "PFU"); return true;}
    if(digitalRead(SW_POT_OX) == LOW) {strcpy(working.stuck, "POX"); return true;}
    if(digitalRead(BU_FP) == LOW) {strcpy(working.stuck, "BFP"); return true;}
    if(digitalRead(BU_FQD) == LOW) {strcpy(working.stuck, "BFQ"); return true;}
    if(digitalRead(BU_OP) == LOW) {strcpy(working.stuck, "BOP"); return true;}
    if(digitalRead(BU_OQD) == LOW) {strcpy(working.stuck, "BOQ"); return true;}
    if(digitalRead(SW_FRV_o) == LOW) {strcpy(working.stuck, "FRV"); return true;}
    if(digitalRead(SW_FRV_c) == LOW) {strcpy(working.stuck, "FRV"); return true;}
    if(mcp2.digitalRead(SW_FPR_o) == LOW) {strcpy(working.stuck, "FPV"); return true;}
    if(mcp2.digitalRead(SW_FPR_c) == LOW) {strcpy(working.stuck, "FPV"); return true;}
    
    if(mcp1.digitalRead(SW_FLV_o) == LOW) {strcpy(working.stuck, "FLV"); return true;}
    if(mcp1.digitalRead(SW_FLV_c) == LOW) {strcpy(working.stuck, "FLV"); return true;}
    if(mcp2.digitalRead(SW_FQD_o) == LOW) {strcpy(working.stuck, "FQD"); return true;}
    if(mcp2.digitalRead(SW_FQD_c) == LOW) {strcpy(working.stuck, "FQD"); return true;}
    if(digitalRead(SW_ORV_o) == LOW) {strcpy(working.stuck, "ORV"); return true;}
    if(digitalRead(SW_ORV_c) == LOW) {strcpy(working.stuck, "ORV"); return true;}
    if(digitalRead(SW_OPR_o) == LOW) {strcpy(working.stuck, "OPR"); return true;}
    if(digitalRead(SW_OPR_c) == LOW) {strcpy(working.stuck, "OPR"); return true;}
    if(mcp1.digitalRead(SW_OLV_o) == LOW) {strcpy(working.stuck, "OLV"); return true;}
    if(mcp1.digitalRead(SW_OLV_c) == LOW) {strcpy(working.stuck, "OLV"); return true;}
    if(mcp2.digitalRead(SW_OQD_o) == LOW) {strcpy(working.stuck, "OQD"); return true;}
    if(mcp2.digitalRead(SW_OQD_c) == LOW) {strcpy(working.stuck, "OQD"); return true;}
    if(digitalRead(SW_REC_on) == LOW) {strcpy(working.stuck, "REC"); return true;}
    if(digitalRead(SW_REC_off) == LOW) {strcpy(working.stuck, "REC"); return true;}
    if(mcp2.digitalRead(SW_MON) == LOW) {strcpy(working.stuck, "MAN"); return true;}
    if(mcp2.digitalRead(SW_MOF) == LOW) {strcpy(working.stuck, "MAN"); return true;}
    if(digitalRead(BU_IGN) == LOW) {strcpy(working.stuck, "IGN"); return true;}
    if(mcp1.digitalRead(BU_ABORT) == LOW) {strcpy(working.stuck, "IGN"); return true;}
    return false;


}

void getVoltage() {
  int i = 0;
  i = analogRead(pin_voltage);
  i = i + analogRead(pin_voltage);
  i = i + analogRead(pin_voltage);
  i = i / 3;
  int v = 0;
  int t = 0;
  t = 595 - i;
  if(t <= 0) v = 100;
  if(t > 115) v = 0;
  if(t <= 115 && t > 0) {
    float c = 0.0;
    c = (float) (115 - t) / (float) 115.00 * (float) 100.00;
    v = (int) c;
  }
  working.voltage = v;
  if(serialDebug) Serial.print("Batt % = ");
  if(serialDebug) Serial.println(working.voltage);
    
}


void doDiags() {

  // This is an endless loop to test everything on power up when 2FA is held down
  displayStr("TST",1);
  displayStr("MOD",2);
  displayStr("123",3);
  displayStr("456",4);
  LEDtestAll();
  displayStr("OK",3);
  displayStr("TST",4);
  int pot1;
  int pot1m;
  int pot2;
  int pot2m;
  int theRange = 500;

  pot1m = analogRead(POT_FU);
  pot2m = analogRead(POT_OX);

  while(1) {

    if(digitalRead(SW_POT_FU) == LOW) {displayStr("FUL",3);displayStr("POT",4);delay(500);}
    if(digitalRead(SW_POT_OX) == LOW) {displayStr("LOX",3);displayStr("POT",4);delay(500);}
    if(digitalRead(BU_FP) == LOW) {displayStr("FPR",3);displayStr("BUT",4);delay(500);}
    if(digitalRead(BU_FQD) == LOW) {displayStr("FQD",3);displayStr("BUT",4);delay(500);}
    if(digitalRead(BU_OP) == LOW) {displayStr("OPR",3);displayStr("BUT",4);delay(500);}
    if(digitalRead(BU_OQD) == LOW) {displayStr("OQD",3);displayStr("BUT",4);delay(500);}
    if(digitalRead(SW_FRV_o) == LOW) {displayStr("FRV",3);displayStr("OPN",4);delay(500);}
    if(digitalRead(SW_FRV_c) == LOW) {displayStr("FRV",3);displayStr("CLS",4);delay(500);}
    if(mcp2.digitalRead(SW_FPR_o) == LOW) {displayStr("FPR",3);displayStr("OPN",4);delay(500);}
    if(mcp2.digitalRead(SW_FPR_c) == LOW) {displayStr("FPR",3);displayStr("CLS",4);delay(500);}
    
    if(mcp1.digitalRead(SW_FLV_o) == LOW) {displayStr("FLV",3);displayStr("OPN",4);delay(500);}
    if(mcp1.digitalRead(SW_FLV_c) == LOW) {displayStr("FLV",3);displayStr("CLS",4);delay(500);}
    if(mcp2.digitalRead(SW_FQD_o) == LOW) {displayStr("FQD",3);displayStr("OPN",4);delay(500);}
    if(mcp2.digitalRead(SW_FQD_c) == LOW) {displayStr("FQD",3);displayStr("CLS",4);delay(500);}
    if(digitalRead(SW_ORV_o) == LOW) {displayStr("ORV",3);displayStr("OPN",4);delay(500);}
    if(digitalRead(SW_ORV_c) == LOW) {displayStr("ORV",3);displayStr("CLS",4);delay(500);}
    if(digitalRead(SW_OPR_o) == LOW) {displayStr("OPV",3);displayStr("OPN",4);delay(500);}
    if(digitalRead(SW_OPR_c) == LOW) {displayStr("OPV",3);displayStr("CLS",4);delay(500);}
    if(mcp1.digitalRead(SW_OLV_o) == LOW) {displayStr("OLV",3);displayStr("OPN",4);delay(500);}
    if(mcp1.digitalRead(SW_OLV_c) == LOW) {displayStr("OLV",3);displayStr("CLS",4);delay(500);}
    if(mcp2.digitalRead(SW_OQD_o) == LOW) {displayStr("OQD",3);displayStr("OPN",4);delay(500);}
    if(mcp2.digitalRead(SW_OQD_c) == LOW) {displayStr("OQD",3);displayStr("CLS",4);delay(500);}
    if(digitalRead(SW_REC_on) == LOW) {displayStr("REC",3);displayStr("ON",4);delay(500);}
    if(digitalRead(SW_REC_off) == LOW) {displayStr("REC",3);displayStr("OFF",4);delay(500);}
    if(mcp2.digitalRead(SW_MON) == LOW) {displayStr("MAN",3);displayStr("OPN",4);delay(500);}
    if(mcp2.digitalRead(SW_MOF) == LOW) {displayStr("MAN",3);displayStr("CLS",4);delay(500);}
    if(digitalRead(BU_IGN) == LOW) {displayStr("IGN",3);displayStr("IGN",4);delay(500);}
    if(mcp1.digitalRead(BU_ABORT) == LOW) {displayStr("ABT",3);displayStr("ABT",4);delay(500);}

    if(digitalRead(pin_arm) == LOW) {displayStr("ARM",3);displayStr("ARM",4);delay(500);}
    if(digitalRead(pin_2FA) == LOW) {displayStr("2FA",3);displayStr("2FA",4);delay(500);}

 
    pot1 = analogRead(POT_FU);
    if (pot1 > (pot1m + 2) || pot1 < (pot1m - 2)) {
      pot1m = pot1;
      float i= (float)((float) pot1 / 1023.0) * (float) theRange;
      display2.showNumber((int) i, false, 3, 0);
    }

    pot2 = analogRead(POT_OX);
    if (pot2 > (pot2m + 2) || pot2 < (pot2m - 2)) {
      pot2m = pot2;
      float i= (float)((float) pot2 / 1023.0) * (float) theRange;
      display4.showNumber((int) i, false, 3, 0);
    }



    
  }
}

void LEDtestAll() {

  digitalWrite(LED_armR, HIGH);delay(250);digitalWrite(LED_armR, LOW);
  digitalWrite(LED_armG, HIGH);delay(250);digitalWrite(LED_armG, LOW);
  pinMode(LED_status , OUTPUT); digitalWrite(LED_status, LOW);delay(250);pinMode(LED_status , INPUT);
  pinMode(LED_radio , OUTPUT); digitalWrite(LED_radio, LOW);delay(250);pinMode(LED_radio , INPUT);
  pinMode(LED_error , OUTPUT); digitalWrite(LED_error, LOW);delay(250);pinMode(LED_error , INPUT);
  
 /* digitalWrite(LED_status, LOW);delay(1000);digitalWrite(LED_status, HIGH);
  digitalWrite(LED_radio, LOW);delay(1000);digitalWrite(LED_radio, HIGH);
  digitalWrite(LED_error, LOW);delay(1000);digitalWrite(LED_error, HIGH);
 */ digitalWrite(LED_BU_FP, HIGH);delay(250);digitalWrite(LED_BU_FP, LOW);
  digitalWrite(LED_BU_FQD, HIGH);delay(250);digitalWrite(LED_BU_FQD, LOW);
  digitalWrite(LED_BU_LP, HIGH);delay(250);digitalWrite(LED_BU_LP, LOW);
  digitalWrite(LED_BU_OQD, HIGH);delay(250);digitalWrite(LED_BU_OQD, LOW);
  mcp2.digitalWrite(LED_FRV, HIGH);delay(250);mcp2.digitalWrite(LED_FRV, LOW);
  mcp2.digitalWrite(LED_FPR, HIGH);delay(250);mcp2.digitalWrite(LED_FPR, LOW);
  mcp1.digitalWrite(LED_FLV, HIGH);delay(250);mcp1.digitalWrite(LED_FLV, LOW);
  mcp1.digitalWrite(LED_FQD, HIGH);delay(250);mcp1.digitalWrite(LED_FQD, LOW);
  mcp2.digitalWrite(LED_ORV, HIGH);delay(250);mcp2.digitalWrite(LED_ORV, LOW);
  mcp2.digitalWrite(LED_OPR, HIGH);delay(250);mcp2.digitalWrite(LED_OPR, LOW);
  mcp1.digitalWrite(LED_OLV, HIGH);delay(250);mcp1.digitalWrite(LED_OLV, LOW);
  mcp1.digitalWrite(LED_OQD, HIGH);delay(250);mcp1.digitalWrite(LED_OQD, LOW);
  mcp2.digitalWrite(LED_REC, HIGH);delay(250);mcp2.digitalWrite(LED_REC, LOW);
  mcp2.digitalWrite(LED_IGN, HIGH);delay(250);mcp2.digitalWrite(LED_IGN, LOW);
  mcp1.digitalWrite(LED_MON, HIGH);delay(250);mcp1.digitalWrite(LED_MON, LOW);
  
}

void displayStr(char theStr[5], int theD) {

  if(theD == 1) {
    display1.setBrightness(BRIGHT_7);
    display1.clear();
    display1.showString(theStr);
  }
  
  if(theD == 2) {
    display2.setBrightness(BRIGHT_7);
    display2.clear();
    display2.showString(theStr);
  }
  if(theD == 3) {
    display3.setBrightness(BRIGHT_7);
    display3.clear();
    display3.showString(theStr);
  }
  if(theD == 4) {
    display4.setBrightness(BRIGHT_7);
    display4.clear();
    display4.showString(theStr);
  }
  
}

void displayInt(int msg, int theD) {

  if(theD == 1) {
    display1.setBrightness(BRIGHT_7);
    display1.clear();
    display1.showNumber(msg, false, 3, 0);
  }
  
  if(theD == 2) {
    display2.setBrightness(BRIGHT_7);
    display2.clear();
    display2.showNumber(msg, false, 3, 0);
  }
  if(theD == 3) {
    display3.setBrightness(BRIGHT_7);
    display3.clear();
    display3.showNumber(msg, false, 3, 0);
  }
  if(theD == 4) {
    display4.setBrightness(BRIGHT_7);
    display4.clear();
    display4.showNumber(msg, false, 3, 0);
  }
  
}




//***********************************************************************  RADIO  *************************************************************************************  
//***********************************************************************  RADIO  *************************************************************************************  

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

/*
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
*/



void checkRadio() {  // **********  CHECK RADIO  *****************
   newWord = 0;
   int vtimeout = 0;
   char receivedChar;
   byte wError = 0;
   int wCount = 0;
   if (Serial1.available() > 0) {
    pinMode(LED_radio , OUTPUT);digitalWrite(LED_radio, LOW); 
    strcpy(theWord, "");
    //if(serialDebug) Serial.println("radio inbound detect");
    unsigned long theStart = millis();
    unsigned long testTime = millis() + 500; 
    while (newWord != 1) {
       if(Serial1.available()) {
         receivedChar = Serial1.read();
         append(theWord, receivedChar);
         wCount++;
         if(receivedChar == 33) {  // look for ! to end
          newWord = 1;
         }
         // Error checking
         if(wCount > 160) { // too long without exit
            wError = 1;
            newWord = 1; //abort
         }
         if(receivedChar < 32 || receivedChar > 126) {  //noise or garbage on the radio - empty and abort to not waste cycles
            wError = 1;
            newWord = 1;
         }
       }
       if(millis() > testTime) {  // exit and evaluate if it takes too long
          if(serialDebug) {Serial.println("*TIMEOUT*: ");delay(1);}
          newWord =1;
          vtimeout = 1;
       }
    }
    if(serialDebug) {
       int m = millis() - theStart;
       Serial.print("T: ");delay(1);Serial.print(m);delay(1);Serial.print(" L:");delay(1);
       Serial.print(strlen(theWord));delay(1);Serial.print("  W:"); delay(1);
       Serial.println(theWord);delay(1);
    }
     if(serialDebug) {Serial.flush();delay(1);}
     digitalWrite(LED_radio, HIGH); pinMode(LED_radio , INPUT);
     if (vtimeout == 0 && wError == 0) {
      radioTimeout = 0; // reset Rx error timer with good sentance
      ProcessRadio();
     }
   }
}




void ProcessRadio() {   // **********  PROCESS RADIO COMMAND  *****************

      int chkNew = 0;
      if(theWord[0] == 35 && theWord[strlen(theWord)-1] == 33) {   // proper sentence recieved from radio # !          
          chkNew = 1;

      } else {  // error
              newWord = 0;
              strcpy(theWord, "");
              if(serialDebug) {Serial.print("Bad RX Sentence: ");Serial.println(theWord);delay(1);}
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

          //-------------- PAD STATUS STRING PROCESSING  --------------------------------
          if (strncmp("#FS,",theWord,4) == 0) { 
            if(serialDebug) {Serial.println(F("Got radio action STATUS"));delay(1);}
            splitString(theWord, ',', tempArr, 50);
            ingestStatus();
            processStatus();

          }

          //-------------- PAD ERRORS PROCESSING  --------------------------------
          if (strncmp("#HER,",theWord,5) == 0) { 
            if(serialDebug) {Serial.println(F("Got radio health and errors"));delay(1);}
            splitString(theWord, ',', tempArr, 50);
            padErrorProcessing();           
          }
          
          //-------------- PRESSURE UPDATE  --------------------------------
          if (strncmp("#PRS,",theWord,5) == 0) { 
            //if(serialDebug) {Serial.println(F("Got radio pressure update"));delay(1);}
            splitString(theWord, ',', tempArr, 50);
            pressureUpdate();           
          }
          //-------------- CONFIG UPDATE  --------------------------------
          if (strncmp("#CFG,",theWord,5) == 0) { 
            splitString(theWord, ',', tempArr, 50);
            processConfig();        
            if(serialDebug) {Serial.println(F("Config is updated"));delay(1);}   
          }


      }
}


// Split a character array into an array of strings using a delimiter
void splitString(char* input, char delimiter, char** outputArray, int maxElements) {
  int index = 0;
  char* token = strtok(input, ",");
  
  while (token != NULL && index < maxElements) {
    outputArray[index++] = token;
    token = strtok(NULL, ",");
  }
}

void processConfig() {
    padConfig.POXenabled = atoi(tempArr[1]);
    padConfig.POXrange = atoi(tempArr[2]);
    padConfig.POXfill = atoi(tempArr[3]);
    padStatus.oxFill = padConfig.POXfill;
    padConfig.POXalarm = atoi(tempArr[4]);
    padConfig.PFLenabled = atoi(tempArr[5]);
    padConfig.PFLrange = atoi(tempArr[6]);
    padConfig.PFLfill = atoi(tempArr[7]);
    padStatus.fuelFill = padConfig.PFLfill;
    padConfig.PFLalarm = atoi(tempArr[8]);
    padConfig.PPSenabled = atoi(tempArr[9]);
    padConfig.PPSrange = atoi(tempArr[10]);
    padConfig.PPSalarm = atoi(tempArr[11]);
    padConfig.PRSenabled = atoi(tempArr[12]);
    padConfig.PRSrange = atoi(tempArr[13]);
    padConfig.PRSalarm = atoi(tempArr[14]);
    padConfig.pressureWaitTime = atoi(tempArr[15]);
    padConfig.lineVentDelay = atoi(tempArr[16]);
    padConfig.mainVoltage2412 = atoi(tempArr[17]);
    padConfig.hazVoltage2412 = atoi(tempArr[18]);
    padConfig.recoveryArmA = atoi(tempArr[19]);
    padConfig.recoveryArmB = atoi(tempArr[20]);
    padConfig.valve9 = atoi(tempArr[21]);
    padConfig.holdMainOn = atoi(tempArr[22]);
    padConfig.holdMainOff = atoi(tempArr[23]);
    padConfig.padRadioPower = atoi(tempArr[24]);
    padConfig.remoteRadioPower = atoi(tempArr[25]);
    padConfig.CPUtempAlarm = atoi(tempArr[26]);
    padConfig.updated = true;
}




void ingestStatus() {
   //tempArr is already loaded

    padStatus.tankConfig = atoi(tempArr[1]);
    padStatus.rocketConnected = stringToBool(tempArr[2]);
    padStatus.padHot = stringToBool(tempArr[3]);
    padStatus.igniterArmed = stringToBool(tempArr[4]);  
    padStatus.valvesArmed = stringToBool(tempArr[5]);

    if (stringToBool(tempArr[6]) && !padStatus.padArmed) {
        //speakNow("Pad Controller Armed");
    }
    if (!stringToBool(tempArr[6]) && padStatus.padArmed) {
        //speakNow("Pad Controller Disarmed");
    }
    padStatus.padArmed = stringToBool(tempArr[6]);
    padStatus.igniterContinuity = stringToBool(tempArr[7]);
    padStatus.mainVolts = atof(tempArr[8]);
    padStatus.hazVolts = atof(tempArr[9]);
    if (padStatus.hazVolts < 2.0) {
        padStatus.hazVolts = 0.0;
    }
    padStatus.pressureOne = atoi(tempArr[11]); // Ox Tank
    padStatus.pressureTwo = atoi(tempArr[12]); // Fuel Tank
    padStatus.pressureThree = atoi(tempArr[10]);  // Pressure ground tank
    padStatus.pressureFour = atoi(tempArr[13]); // Pressure vehicle tank
    padStatus.tankTemperature = atoi(tempArr[14]); // Pressure tank temp
    padStatus.valveOne = stringToBool(tempArr[15]);
    padStatus.valveTwo = stringToBool(tempArr[16]);
    padStatus.valveThree = stringToBool(tempArr[17]);
    padStatus.valveFour = stringToBool(tempArr[18]);
    padStatus.valveFive = stringToBool(tempArr[19]);
    padStatus.valveSix = stringToBool(tempArr[20]);
    padStatus.valveSeven = stringToBool(tempArr[21]);
    padStatus.valveEight = stringToBool(tempArr[22]);
    padStatus.valveNine = stringToBool(tempArr[23]);
    padStatus.mainValvesOn = stringToBool(tempArr[24]);
    padStatus.mainValvesOff = stringToBool(tempArr[25]);
    padStatus.mainValvesState = stringToBool(tempArr[26]);
    padStatus.tankThreeActuatorOn = stringToBool(tempArr[27]);
    padStatus.tankThreeActuatorOff = stringToBool(tempArr[28]);
    padStatus.processC1 = atoi(tempArr[29]);  // can be 0- not run, 1 - complete, 2 - running, 3-stopped
    padStatus.processF1 = atoi(tempArr[30]);
    padStatus.processF2 = atoi(tempArr[31]);
    padStatus.processO1 = atoi(tempArr[32]);
    padStatus.processO2 = atoi(tempArr[33]);
    padStatus.processAbort = atoi(tempArr[34]);
    padStatus.process7 = atoi(tempArr[35]);
    padStatus.process8 = atoi(tempArr[36]);
    padStatus.oxTankDisconnect = stringToBool(tempArr[37]);
    padStatus.fuelTankDisconnect = stringToBool(tempArr[38]);
    padStatus.pressureTankDisconnect = stringToBool(tempArr[39]);
    padStatus.recoveryPower = stringToBool(tempArr[40]);
    padStatus.altimeterAarmed = stringToBool(tempArr[41]);
    padStatus.altimeterBarmed = stringToBool(tempArr[42]);
    padStatus.CPUtemp = atoi(tempArr[43]);
    padStatus.oxFill = atoi(tempArr[44]);
    padStatus.fuelFill = atoi(tempArr[45]);
}

void processStatus() {
  // after getting new status do this

  // Displays
  displayInt(padStatus.pressureTwo,1);
  displayInt(padStatus.pressureOne,3);
  displayInt(padStatus.fuelFill,2);
  displayInt(padStatus.oxFill,4);
  // LED states - process lights
  if(padStatus.padArmed) {
    digitalWrite(LED_armR, HIGH);
    digitalWrite(LED_armG, LOW);
  } else {
    digitalWrite(LED_armR, LOW);
    digitalWrite(LED_armG, HIGH);
  }
  if(padStatus.processF1 == 2) {
    digitalWrite(LED_BU_FP, HIGH);
  } else {
    digitalWrite(LED_BU_FP, LOW);
  }
  if(padStatus.processF2 == 2) {
    digitalWrite(LED_BU_FQD, HIGH);
  } else {
    digitalWrite(LED_BU_FQD, LOW);
  }  
  if(padStatus.processO1 == 2) {
    digitalWrite(LED_BU_LP, HIGH);
  } else {
    digitalWrite(LED_BU_LP, LOW);
  }  
  if(padStatus.processO2 == 2) {
    digitalWrite(LED_BU_OQD, HIGH);
  } else {
    digitalWrite(LED_BU_OQD, LOW);
  }    
  // Valve LEDS
  if(padStatus.valveOne) {
    mcp1.digitalWrite(LED_FLV, HIGH);
  } else {
    mcp1.digitalWrite(LED_FLV, LOW);
  }   
  if(padStatus.valveFour) {
    mcp2.digitalWrite(LED_FPR, HIGH);
  } else {
    mcp2.digitalWrite(LED_FPR, LOW);
  }  
  if(padStatus.valveTwo) {
    mcp1.digitalWrite(LED_FQD, HIGH);
  } else {
    mcp1.digitalWrite(LED_FQD, LOW);
  }  
  if(padStatus.valveSeven) {
    mcp2.digitalWrite(LED_FRV, HIGH);
  } else {
    mcp2.digitalWrite(LED_FRV, LOW);
  }  
  if(padStatus.valveThree) {
    mcp1.digitalWrite(LED_OLV, HIGH);
  } else {
    mcp1.digitalWrite(LED_OLV, LOW);
  }  
  if(padStatus.valveFive) {
    mcp2.digitalWrite(LED_OPR, HIGH);
  } else {
    mcp2.digitalWrite(LED_OPR, LOW);
  }  
  if(padStatus.valveSix) {
    mcp1.digitalWrite(LED_OQD, HIGH);
  } else {
    mcp1.digitalWrite(LED_OQD, LOW);
  }   
  if(padStatus.valveEight) {
    mcp2.digitalWrite(LED_ORV, HIGH);
  } else {
    mcp2.digitalWrite(LED_ORV, LOW);
  }  
  if(padStatus.mainValvesState) {
    mcp1.digitalWrite(LED_MON, HIGH);
  } else {
    mcp1.digitalWrite(LED_MON, LOW);
  }  
  // LED other  
  if(padStatus.igniterContinuity) {
    mcp2.digitalWrite(LED_IGN, HIGH);
  } else {
    mcp2.digitalWrite(LED_IGN, LOW);
  }  
  if(padStatus.recoveryPower) { 
    if(padStatus.altimeterBarmed) { // got both
       mcp2.digitalWrite(LED_REC, HIGH);
       working.recFlash = false;
    } else {
      // have power but not armed
      working.recFlash = true;
    }
  } else {
    mcp2.digitalWrite(LED_REC, LOW);
    working.recFlash = false;
  }  

}

void pressureUpdate() {
    padStatus.pressureOne = atoi(tempArr[2]); // Ox Tank
    padStatus.pressureTwo = atoi(tempArr[3]); // Fuel Tank
    padStatus.pressureThree = atoi(tempArr[1]);  // Pressure ground tank
    padStatus.pressureFour = atoi(tempArr[4]); // Pressure vehicle tank
    displayInt(padStatus.pressureTwo,1);
    displayInt(padStatus.pressureOne,3);
    displayInt(padStatus.fuelFill,2);
    displayInt(padStatus.oxFill,4);
}

void padErrorProcessing() {

    padErrors.errorCount = atoi(tempArr[1]);
    padErrors.mainBatt = atoi(tempArr[2]);
    padErrors.hazBatt = atoi(tempArr[3]);
    padErrors.MCUtemp = atoi(tempArr[4]);
    padErrors.PT = atoi(tempArr[5]);
    padErrors.radio = atoi(tempArr[6]);
    padErrors.flashMem = atoi(tempArr[7]);
    padErrors.flashFull = atoi(tempArr[8]);
    padErrors.ADC = atoi(tempArr[9]);
    padErrors.MCUcrash = atoi(tempArr[10]);

    if(padErrors.errorCount > 0) {
      if(padErrors.mainBatt == 1) {
         displayStr("ERR",1);displayStr("PAD",2); 
         displayStr("BAT",3);displayInt((int)padStatus.mainVolts,4); 
         delay(2000);
      }
      if(padErrors.hazBatt == 1) {
         displayStr("ERR",1);displayStr("PAD",2); 
         displayStr("HAZ",3);displayInt((int)padStatus.hazVolts,4); 
         delay(2000);
      }
      if(padErrors.MCUtemp == 1) {
         displayStr("ERR",1);displayStr("PAD",2); 
         displayStr("TMP",3);displayInt(padStatus.CPUtemp,4); 
         delay(2000);
      }
      if(padErrors.PT == 1) {
         displayStr("ERR",1);displayStr("PAD",2); 
         displayStr("PT",3);displayStr(" ",4);  
         delay(2000);
      }
      if(padErrors.radio == 1) {
         displayStr("ERR",1);displayStr("PAD",2); 
         displayStr("RAD",3);displayStr("IO",4);  
         delay(2000);
      }
      if(padErrors.flashMem == 1) {
         displayStr("ERR",1);displayStr("PAD",2); 
         displayStr("FLS",3);displayStr("MEM",4);  
         delay(2000);
      }
      if(padErrors.flashFull == 1) {
         displayStr("ERR",1);displayStr("PAD",2); 
         displayStr("FLS",3);displayStr("FUL",4);  
         delay(2000);
      }
      if(padErrors.ADC == 1) {
         displayStr("ERR",1);displayStr("PAD",2); 
         displayStr("ADC",3);displayStr(" ",4);  
         delay(2000);
      }
      if(padErrors.MCUcrash == 1) {
         displayStr("ERR",1);displayStr("PAD",2); 
         displayStr("MCU",3);displayStr("CSH",4);  
         delay(2000);
      }
    }
  
}






// Define a function to convert a string to a boolean
bool stringToBool(const char* str) {
    return strcmp(str, "1") == 0;
}




void append(char* s, char c) {  //used to concat char to char array
        int len = strlen(s);
        s[len] = c;
        s[len+1] = '\0';
}
