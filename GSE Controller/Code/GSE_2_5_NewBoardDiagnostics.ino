


/* FAR GSE New Board Diagnostic v2.5
 *  
 *  Used to test all sensors, inputs and outputs on a new GSE Controller board
 *  Current hardware version is 2.5
 *  
 *  January 2024
 *  
 *  Compile at 396 Mhz for best performance/heat management
 *  
 *  TO DO:
 *    - none
 *   
 *  
 */

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


  #include "Arduino.h"
  #include "ADS1115-Driver.h"
  ADS1115 ads1115a = ADS1115(0x49);
  ADS1115 ads1115b = ADS1115(0x48);

 //Radio
 char theWord[100];
 int newWord = 0;

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

// enabling ADC channels

  bool aEnabled[8] = {1,1,1,1,1,1,1,1};
  

//************************************************************************************************************************************************************  SETUP
//************************************************************************************************************************************************************  SETUP

 
void setup() {

// Pin setup
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
  pinMode(pin_ext5v, OUTPUT);digitalWrite(pin_ext5v, LOW);
  pinMode(pin_strobes, OUTPUT);digitalWrite(pin_strobes, LOW);
  pinMode(pin_fan, OUTPUT);digitalWrite(pin_fan, LOW);
  
  delay(4000);
  Serial.begin(115200);
  delay(100);
  Serial.println("GSE 2.5 New Board Diagnostics and Health Check");
  Serial.println(" ");Serial.println(" ");

  printCommands();

}

//************************************************************************************************************************************************************  MAIN LOOP
//************************************************************************************************************************************************************  MAIN LOOP

void loop() {

  char chIn = 255;
  if ( Serial.available() ) {
    do {
     if ( chIn != '0' && chIn != '1' && chIn != '2' && chIn != '$' && chIn != '#' && chIn != '3' && chIn != '4' && chIn != '5' && chIn != '6' && chIn != '7' && chIn != '8' && chIn != '9' && chIn != 'a' && chIn != 'b' && chIn != 'c' && chIn != 'd' && chIn != 'e' && chIn != 'f' && chIn != 'g' && chIn != 'h' && chIn != 'i' && chIn != 'j' && chIn != 'k' && chIn != 'l' && chIn != 'm' && chIn != 'n' && chIn != 'o' && chIn != 'p' && chIn != 'q' && chIn != 'r' && chIn != 's' && chIn != 't'&& chIn != 'u'&& chIn != 'v'&& chIn != 'w'&& chIn != 'x'&& chIn != 'y'&& chIn != 'z' && chIn != '?' && chIn != '!' && chIn != '&' && chIn != '*' && chIn != '+'  )
        chIn = Serial.read();
      else
        Serial.read();
    }
    while ( Serial.available() );
  }

  switch(chIn) {
    case '1':
      toggleLed1(); printCommands(); break;
    case '2':
      toggleLed2(); printCommands(); break;
    case '3':
      //toggleLed3(); printCommands(); break;                
    case '4':
      toggleLB(); printCommands(); break;
    case '?':
      toggleBuzzer(); printCommands(); break;
    case '6':
      continuitySwitch(); printCommands(); break;
    case '7':
      checkCont();printCommands(); break;
    case '8':
      toggleIgniter(); printCommands(); break;
    case '9':
      RadioTXtest();printCommands(); break;
    case '0':
      toggleRset(); printCommands(); break;

      
    case 'a':
      toggleRon();  printCommands(); break;
    case 'b':
      toggleRoff(); printCommands(); break;
    case 'c':
      armStatus(); printCommands(); break;
    case 'd':
      rocketConnect(); printCommands(); break;
    case 'e':
      toggleV7(); printCommands(); break;
    case 'f':
      toggleV8(); printCommands(); break;
    case 'g':
      toggleV9(); printCommands(); break;
    case 'h':
      toggleMainOn(); printCommands(); break;
    case 'i':
      toggleMainOff(); printCommands(); break;
    case 'j':
      toggleV1(); printCommands(); break;
    case 'k':
      toggleV2(); printCommands(); break;
    case 'l':
      toggleV3(); printCommands(); break;
    case 'm':
      toggleV4(); printCommands(); break;
    case 'n':
      toggleV5(); printCommands(); break;
    case 'o':
      toggleV6(); printCommands(); break;
    case 'p':
      formatFlash(); printCommands(); break;
    case 'q':
      eraseFlash(); printCommands(); break;
    case 'r':
      printFlashDirectory(); printCommands(); break;
    case 's':
      copy2SD(); printCommands(); break;
    case 't':
      flashLogging(); printCommands(); break;
    case 'u':
      //toggleValve("VPP"); printCommands(); break;
    case 'v':
      checkTemp(); printCommands(); break;
    case 'w':
      //toggleValve("VBO"); printCommands(); break;
    case 'x':
      I2Cscan(); printCommands(); break;
    case 'y':
      PTloop(); printCommands(); break;
    case 'z':
      //radioRead();printCommands(); break; //Configuration report
      radioTest();printCommands(); break;
    case '$':
      readConfig();printCommands(); break;
    case '#':
      readFile();printCommands(); break;
    case '5':
      toggle5v();printCommands(); break;
    case '!':
      toggleStrobe();printCommands(); break;
    case '&':
      toggleFan();printCommands(); break;
    case '*':
      fuseTest();printCommands(); break;
    case '+':
      checkVoltage();printCommands(); break;
  }
  
}

void printCommands() {
  Serial.println("  ");
  Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
  Serial.println("COMMANDS:");
  Serial.println("      Local:  1-LED1, 2-LED2, 4-LED BLTIN, ?-Buzzer, 5-5vExt, !-Strobes, &-Fan ");
  Serial.println("    Sensing:  v-CPU Temp, 6-Cont Switch, *-Fuses, +-Voltage ");
  Serial.println("   Recovery:  a-Pulse on, b-Pulse off, c-Arm Status ");
  Serial.println("     Ignite:  7-Ign Continuity, 8-Igniter Fire(HAZ), ACTUATORS: h-Main on(HAZ), i-Main off  ");
  Serial.println("     Rocket:  d-Connect, VALVES: e-RFV/v7, f=ROV/v8, g-RSP/vSpare  ACTUATORS: h-Main on(HAZ), i-Main off");
  Serial.println("        Pad:  VALVES: j-FLV/v1, k-FQD/v2, l-OLV/v3, m-FPR/v4(HAZ), n-OPR/v5(HAZ), o-OQD/v6");    
  Serial.println("        PTs:  x-I2C Scan, y-PT values");
  Serial.println("      Radio:  z-Radio Test, 0-radio set, 9-Radio TX");
  Serial.println("      FLASH:  p-Low Format, q-Quick Erase, r-Directory, s-copy to SD, t-Flash Logging Test, $-read config file, #-read file");  
  Serial.println("-----------------------------------------------------------------------------------------------------------------------------------------------------------");  
  Serial.println("  ");
}


void checkVoltage() {

  Serial.println("Checking voltage using the ADC... ");
  Serial.println(" ");
  // ADS1115 
  Wire.begin();
  delay(1);
  ads1115a.reset();
  delay(100);
  ads1115b.reset();
  delay(100);
  ads1115a.setDeviceMode(ADS1115_MODE_SINGLE);
  ads1115a.setDataRate(ADS1115_DR_860_SPS);
  ads1115a.setPga(ADS1115_PGA_6_144);
  delay(100);
  ads1115b.setDeviceMode(ADS1115_MODE_SINGLE);
  ads1115b.setDataRate(ADS1115_DR_860_SPS);
  ads1115b.setPga(ADS1115_PGA_6_144);
  delay(100); 
  Wire.setClock(400000); 


// ----------------------------------  Check Main Voltage  ------------------------
      elapsedMicros adcTimer = 0;
      float valueADC = 0.0;
      ads1115a.setMultiplexer(ADS1115_MUX_AIN0_GND);
      ads1115a.startSingleConvertion();
      adcTimer = 0;
      while(adcTimer < 1000) {}; //wait 1000 micros for read - Q ver of ADS1115
      valueADC = ads1115a.readConvertedValue();
      float mainVolts = valueADC / 100; // opAmp voltage divider x10
      Serial.print("Main Voltage = ");
      Serial.println(mainVolts);


    // ----------------------------------  Check Haz Voltage  ------------------------
    
      ads1115a.setMultiplexer(ADS1115_MUX_AIN1_GND);
      ads1115a.startSingleConvertion();
      adcTimer = 0;
      while(adcTimer < 1000) {}; //wait 1000 micros for read - Q ver of ADS1115
      valueADC = ads1115a.readConvertedValue();
      
      float hazVolts = valueADC / 100; // opAmp voltage divider x10
      Serial.print("Haz Voltage = ");
      Serial.println(hazVolts);
    



}


void toggle5v() {
  Serial.print("5v External Toggle is now ");
  if(digitalRead(pin_ext5v)) {
    Serial.println("OFF");
    digitalWrite(pin_ext5v, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_ext5v, HIGH);
  }
 
}

void toggleStrobe() {
  Serial.print("Strobe Toggle is now ");
  if(digitalRead(pin_strobes)) {
    Serial.println("OFF");
    digitalWrite(pin_strobes, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_strobes, HIGH);
  }
 
}

void toggleFan() {
  Serial.print("Fan Toggle is now ");
  if(digitalRead(pin_fan)) {
    Serial.println("OFF");
    digitalWrite(pin_fan, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_fan, HIGH);
  }
 
}

void fuseTest() {
      Serial.print("Fuse Detect Status:  ");
      // If voltage is not on then this will report incorrectly

      if(digitalRead(pin_fuse5v) == LOW) {
        Serial.println("5v Fuse Good/Closed");
      } else {
        Serial.println("5v Fuse Open/Bad");
      }
      if(digitalRead(pin_fuseMain) == LOW) {
        Serial.println("Main Fuse Good/Closed");
      } else {
        Serial.println("Main Fuse Open/Bad");
        //zzz check the voltage 
      }
      if(digitalRead(pin_fuseHaz) == LOW) {
        Serial.println("Haz Fuse Good/Closed");
      } else {
        Serial.println("Haz Fuse Open/Bad");
        //zzz check the voltage 
      }

      
}

void checkCont() {
      Serial.print("Igniter Continuity = ");
      if(digitalRead(pin_igniterContinuity) == LOW) {
        Serial.println("Closed");
      } else {
        Serial.println("Open");
      }
}

void checkTemp(){
      float CPUtemp = 0.0;
      Serial.println("Checking CPU Temperature...");Serial.println(" ");
      Serial.print("CPU Temp (f) = ");
      CPUtemp  = (tempmonGetTemp() * 9.0f / 5.0f) + 32.0f;
      Serial.println(CPUtemp);
      
}

void toggleRon() {
  Serial.print("Recovery ON Toggle is now ");
  if(digitalRead(pin_recoveryOn)) {
    Serial.println("OFF");
    digitalWrite(pin_recoveryOn, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_recoveryOn, HIGH);
  }
 
}

void toggleRoff() {
  Serial.print("Recovery OFF Toggle is now ");
  if(digitalRead(pin_recoveryOff)) {
    Serial.println("OFF");
    digitalWrite(pin_recoveryOff, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_recoveryOff, HIGH);
  }
 
}

void toggleIgniter(){
  Serial.print("Igniter Toggle is now ");
  if(digitalRead(pin_igniterFIRE)) {
    Serial.println("OFF");
    digitalWrite(pin_igniterFIRE, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_igniterFIRE, HIGH);
  }
}

void toggleMainOn(){
  Serial.print("Main On Toggle is now ");
  if(digitalRead(pin_mainValveOn)) {
    Serial.println("OFF");
    digitalWrite(pin_mainValveOn, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_mainValveOn, HIGH);
  }
}

void toggleMainOff(){
  Serial.print("Main Off Toggle is now ");
  if(digitalRead(pin_mainValveOff)) {
    Serial.println("OFF");
    digitalWrite(pin_mainValveOff, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_mainValveOff, HIGH);
  }
}

void toggleV1(){
  Serial.print("Valve 1 Toggle is now ");
  if(digitalRead(pin_valve1)) {
    Serial.println("OFF");
    digitalWrite(pin_valve1, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_valve1, HIGH);
  }
}
void toggleV2(){
  Serial.print("Valve 2 Toggle is now ");
  if(digitalRead(pin_valve2)) {
    Serial.println("OFF");
    digitalWrite(pin_valve2, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_valve2, HIGH);
  }
}
void toggleV3(){
  Serial.print("Valve 3 Toggle is now ");
  if(digitalRead(pin_valve3)) {
    Serial.println("OFF");
    digitalWrite(pin_valve3, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_valve3, HIGH);
  }
}
void toggleV4(){
  Serial.print("Valve 4 Toggle is now ");
  if(digitalRead(pin_valve4)) {
    Serial.println("OFF");
    digitalWrite(pin_valve4, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_valve4, HIGH);
  }
}
void toggleV5(){
  Serial.print("Valve 5 Toggle is now ");
  if(digitalRead(pin_valve5)) {
    Serial.println("OFF");
    digitalWrite(pin_valve5, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_valve5, HIGH);
  }
}
void toggleV6(){
  Serial.print("Valve 6 Toggle is now ");
  if(digitalRead(pin_valve6)) {
    Serial.println("OFF");
    digitalWrite(pin_valve6, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_valve6, HIGH);
  }
}
void toggleV7(){
  Serial.print("Valve 7 Toggle is now ");
  if(digitalRead(pin_valve7)) {
    Serial.println("OFF");
    digitalWrite(pin_valve7, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_valve7, HIGH);
  }
}
void toggleV8(){
  Serial.print("Valve 8 Toggle is now ");
  if(digitalRead(pin_valve8)) {
    Serial.println("OFF");
    digitalWrite(pin_valve8, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_valve8, HIGH);
  }
}
void toggleV9(){
  Serial.print("Valve 9 Toggle is now ");
  if(digitalRead(pin_valve9)) {
    Serial.println("OFF");
    digitalWrite(pin_valve9, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_valve9, HIGH);
  }
}


void I2Cscan() {

 byte error, address;
  int nDevices;
  Serial.println("Scanning I2C Bus...");
  Wire.begin();
  delay(1000);
  
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  Wire.end(); 

  
}

//*****************************************************************************  ADC STUFF ******************************

void PTloop() {

  // ADS1115 
  Wire.begin();
  delay(1);
  ads1115a.reset();
  delay(100);
  ads1115b.reset();
  delay(100);
  ads1115a.setDeviceMode(ADS1115_MODE_SINGLE);
  ads1115a.setDataRate(ADS1115_DR_860_SPS);
  ads1115a.setPga(ADS1115_PGA_6_144);
  delay(100);
  ads1115b.setDeviceMode(ADS1115_MODE_SINGLE);
  ads1115b.setDataRate(ADS1115_DR_860_SPS);
  ads1115b.setPga(ADS1115_PGA_6_144);
  delay(100); 
  Wire.setClock(400000); 
  delay(100);  

  uint16_t valueA = 0;
  uint16_t valueB = 0;

  float value0f = 0.0;
  float value1f = 0.0;
  float value2f = 0.0;
  float value3f = 0.0;
  float value0fb = 0.0;
  float value1fb = 0.0;
  float value2fb = 0.0;
  float value3fb = 0.0;



  unsigned long aMicros = 900;
  //int aMicros = 100000;
  unsigned long  bMicros = 900;
  int aRotate = 0;
  int bRotate = 0;
  int aCounter[5] = {0,0,0,0};
  int bCounter[5] = {0,0,0,0};
  aRotate = nextA(aRotate);
  bRotate = nextB(bRotate);

  // Do the first one A
  if(aRotate == 1) ads1115a.setMultiplexer(ADS1115_MUX_AIN0_GND);
  if(aRotate == 2) ads1115a.setMultiplexer(ADS1115_MUX_AIN1_GND);
  if(aRotate == 3) ads1115a.setMultiplexer(ADS1115_MUX_AIN2_GND);
  if(aRotate == 4) ads1115a.setMultiplexer(ADS1115_MUX_AIN3_GND);
  ads1115a.startSingleConvertion();

  // Do the first one B
  if(bRotate == 1) ads1115b.setMultiplexer(ADS1115_MUX_AIN0_GND);
  if(bRotate == 2) ads1115b.setMultiplexer(ADS1115_MUX_AIN1_GND);
  if(bRotate == 3) ads1115b.setMultiplexer(ADS1115_MUX_AIN2_GND);
  if(bRotate == 4) ads1115b.setMultiplexer(ADS1115_MUX_AIN3_GND);
  ads1115b.startSingleConvertion();


  elapsedMillis timeIt = 0;
  elapsedMicros aTimer = 0;
  elapsedMicros bTimer = 0;

  Serial.println("Starting Five Second Sampling Test...");
  Serial.println("    ");
  
  
  while(timeIt < 5000) {

    if(aTimer >= aMicros) {
      //Serial.print("Starting    ");
      valueA = ads1115a.readConvertedValue();
      //Serial.print(valueA);
      //Serial.print("   ");
      //Serial.print(aRotate);
      //Serial.print("   ");      
      if(aRotate == 1) value0f += (float) valueA;
      if(aRotate == 2) value1f += (float) valueA;
      if(aRotate == 3) value2f += (float) valueA;
      if(aRotate == 4) value3f += (float) valueA;
      aCounter[aRotate]++;
      //Serial.print(aCounter[aRotate]);
      //Serial.print("    Next:");
      aRotate = nextA(aRotate);
      //Serial.print(aRotate);
      //Serial.print("   ");
      if(aRotate == 1) ads1115a.setMultiplexer(ADS1115_MUX_AIN0_GND);
      if(aRotate == 2) ads1115a.setMultiplexer(ADS1115_MUX_AIN1_GND);
      if(aRotate == 3) ads1115a.setMultiplexer(ADS1115_MUX_AIN2_GND);
      if(aRotate == 4) ads1115a.setMultiplexer(ADS1115_MUX_AIN3_GND);
      ads1115a.startSingleConvertion();
      aTimer = 0;
      //Serial.println("DONE!   ");
    }

    if(bTimer >= bMicros) {
      //Serial.print("Starting    ");
      valueB = ads1115b.readConvertedValue();
      //Serial.print(valueB);
      //Serial.print("   ");
      //Serial.print(bRotate);
      //Serial.print("   ");      
      if(bRotate == 1) value0fb += (float) valueB;
      if(bRotate == 2) value1fb += (float) valueB;
      if(bRotate == 3) value2fb += (float) valueB;
      if(bRotate == 4) value3fb += (float) valueB;
      bCounter[bRotate]++;
      //Serial.print(bCounter[bRotate]);
      //Serial.print("    Next:");
      bRotate = nextB(bRotate);
      //Serial.print(bRotate);
      //Serial.print("   ");
      if(bRotate == 1) ads1115b.setMultiplexer(ADS1115_MUX_AIN0_GND);
      if(bRotate == 2) ads1115b.setMultiplexer(ADS1115_MUX_AIN1_GND);
      if(bRotate == 3) ads1115b.setMultiplexer(ADS1115_MUX_AIN2_GND);
      if(bRotate == 4) ads1115b.setMultiplexer(ADS1115_MUX_AIN3_GND);
      ads1115b.startSingleConvertion();
      bTimer = 0;
      //Serial.println("DONE!   ");
    }
    
  } // end five second while





  Serial.println("ACD1 Values:     ");
  if(aEnabled[0] == 1) value0f = value0f / (float) aCounter[1];
  if(aEnabled[1] == 1) value1f = value1f / (float) aCounter[2];
  if(aEnabled[2] == 1) value2f = value2f / (float) aCounter[3];
  if(aEnabled[3] == 1) value3f = value3f / (float) aCounter[4];
  
  Serial.print(" (VOL) IN 0: ");
  Serial.print(value0f,0);
  Serial.print("     count: ");
  Serial.println(aCounter[1]);
  Serial.print(" (HAZ) IN 1: ");
  Serial.print(value1f,0);
  Serial.print("     count: ");
  Serial.println(aCounter[2]);
  Serial.print(" (N/A) IN 2: ");
  Serial.print(value2f,0);
  Serial.print("     count: ");
  Serial.println(aCounter[3]);
  Serial.print(" (PSP) IN 3: ");
  Serial.print(value3f,0);
  Serial.print("     count: ");
  Serial.println(aCounter[4]);
  Serial.println("  ");

  Serial.println("ACD2 Values:     ");
  if(aEnabled[4] == 1) value0fb = value0fb / (float) bCounter[1];
  if(aEnabled[5] == 1) value1fb = value1fb / (float) bCounter[2];
  if(aEnabled[6] == 1) value2fb = value2fb / (float) bCounter[3];
  if(aEnabled[7] == 1) value3fb = value3fb / (float) bCounter[4];
  
  Serial.print(" (PFL) IN 0: ");
  Serial.print(value0fb,0);
  Serial.print("     count: ");
  Serial.println(bCounter[1]);
  Serial.print(" (POX) IN 1: ");
  Serial.print(value1fb,0);
  Serial.print("     count: ");
  Serial.println(bCounter[2]);
  Serial.print(" (N/A) IN 2: ");
  Serial.print(value2fb,0);
  Serial.print("     count: ");
  Serial.println(bCounter[3]);
  Serial.print(" (RP4) IN 3: ");
  Serial.print(value3fb,0);
  Serial.print("     count: ");
  Serial.println(bCounter[4]);
  Serial.println("  ");

}



int nextA(int curVal) {
 int iNext = curVal +1;
 if(iNext > 4) iNext = 1;
 int iNext2 = 0;
 for(int i=0;i < 4;i++) {
  iNext2 = iNext++;
  if(iNext2 == 5) iNext2 = 1;
  if(aEnabled[iNext2-1] == 1) return iNext2;
 }
 return 0; // nothing enabled
}

int nextB(int curVal) {
 int iNext = curVal +1;
 if(iNext > 4) iNext = 1;
 int iNext2 = 0;
 for(int i=0;i < 4;i++) {
  iNext2 = iNext++;
  if(iNext2 == 5) iNext2 = 1;
  if(aEnabled[iNext2 + 3] == 1) return iNext2;
 }
 return 0; // nothing enabled
}






void toggleLB() {
  Serial.print("LED Built-in Toggle is now ");
  if(digitalRead(pin_internalLED)) {
    Serial.println("OFF");
    digitalWrite(pin_internalLED, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_internalLED, HIGH);
  }
 
}

void toggleLed1() {
  Serial.print("LED1 Toggle is now ");
  if(digitalRead(pin_led1)) {
    Serial.println("OFF");
    digitalWrite(pin_led1, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_led1, HIGH);
  }
 
}

void toggleLed2() {
  Serial.print("LED2 Toggle is now ");
  if(digitalRead(pin_led2)) {
    Serial.println("OFF");
    digitalWrite(pin_led2, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_led2, HIGH);
  }
 
}


void toggleBuzzer() {
  Serial.print("Buzzer Toggle is now ");
  if(digitalRead(pin_buzzer)) {
    Serial.println("OFF");
    digitalWrite(pin_buzzer, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_buzzer, HIGH);
  }
}

void togglePoff() {
  Serial.print("Recovery Pulse Off Toggle is now ");
  if(digitalRead(pin_recoveryOff)) {
    Serial.println("OFF");
    digitalWrite(pin_recoveryOff, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_recoveryOff, HIGH);
  }
 
}

void togglePon() {
  Serial.print("Recovery Pulse On Toggle is now ");
  if(digitalRead(pin_recoveryOn)) {
    Serial.println("OFF");
    digitalWrite(pin_recoveryOn, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_recoveryOn, HIGH);
  }
 
}

void continuitySwitch() {
  Serial.print("Continuity switch is ");
  if(digitalRead(pin_continuitySwitch) == HIGH) {
    Serial.println("OPEN  (not pressed)");
  } else {
    Serial.println("CLOSED  (pressed)");
  }
}

void armStatus() {
  Serial.print("Recovery Armed A is ");
  if(digitalRead(pin_altAarmed) == HIGH) {
    Serial.println("OPEN  (not armed)");
  } else {
    Serial.println("CLOSED  (armed)");
  }
  Serial.print("Recovery Armed B is ");
  if(digitalRead(pin_altBarmed) == HIGH) {
    Serial.println("OPEN  (not armed)");
  } else {
    Serial.println("CLOSED  (armed)");
  }
}

void rocketConnect() {
  

  Serial.print("The Rocket umbilical is  ");
  if(digitalRead(pin_rocketDetect) == HIGH) {
    Serial.println("NOT CONNECTED  (open)");
  } else {
    Serial.println("CONNECTED  (closed)");
  }
}


//***************************************************  RADIO STUFF ********************************************


void RadioTXtest(){

    Serial.println("Initiating Radio Transmit Test...");
    Serial.println(" ");
    Serial1.begin(19200);
    Serial.println("Transmitting 10 packets....");
    for(int i=0; i < 10; i++) {
      Serial1.print("@M,33,Hello World hello hello hello hello hello hello hello hello hello hello hello hello hello hello hello ,!");
    }
    Serial.println("Done.");

}


void radioTest() {
  
    Serial.println("Initiating Radio Test...");
    Serial.println(" ");
    digitalWrite(pin_radioSet, HIGH); //turn radio set pin to low
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
      checkRadio();
      if(newWord == 1) {
        Serial.print("Radio Rx:  ");
        Serial.println(theWord);
        char *ptr = strstr(theWord, "LORA6100");
        if (ptr != NULL) Serial.println("Radio Set Comms Test Successful!"); 
        digitalWrite(pin_radioSet, LOW); //turn radio set pin to low
        delay(2000);
        Serial1.begin(19200);
        Serial1.print("@M,33,Hello World,!");
        Serial1.flush();
        Serial.println(" "); 
        Serial.println("Message Sent"); 
        
        x = 1;
        
      }
      if(millis() > testtime) {
        Serial.println("Radio Timeout");
        digitalWrite(pin_radioSet, LOW); //turn radio set pin to low
        delay(200);
        Serial1.begin(19200);
        x = 2;
      }
       
    }
}

void checkRadio() {  // **********  CHECK RADIO  *****************
   newWord = 0;
   char receivedChar;
   if (Serial1.available() > 0) {
  
    unsigned long testTime = millis() + 100; //100
    
    strcpy(theWord, "");
    //Serial.println("radio inbound detect");

    while (newWord != 1) {
       if(Serial1.available()) {

         receivedChar = Serial1.read();
         //Serial.print(receivedChar);
         append(theWord, receivedChar);
         if(receivedChar == 0x0a) {
            newWord = 1;
            break;   
         }
       }
       if(millis() > testTime) {  // exit and evaluate if it takes too long
          newWord =1;
          Serial.println("RX timeout exit error");
          Serial.println(theWord);
          break;
       }
       delay(1);
     }
     Serial.println(" ");
     //ProcessRadio();
   }
}

void append(char* s, char c) {  //used to concat char to char array
        int len = strlen(s);
        s[len] = c;
        s[len+1] = '\0';
}

void radioRead() {
  
    Serial.println("Reading Radio Configuration...");
    Serial.println(" ");
    digitalWrite(pin_radioSet, HIGH); //turn radio set pin to low
    delay(200);
    Serial1.begin(9600);
    char s[5];
    strcpy(s,"");
    unsigned long testtime = millis() + 5000;
    s[0] = 0xaa;
    s[1] = 0xfa;
    s[2] = 0x01; // 01 = read configuration 
    s[3] = 0x0d; //  /r termination
    s[4] = 0x0a; //  /n termination
    Serial.print("Results:  ");
    Serial1.println(s);
    unsigned long testTime = millis() + 5000; //100
    while(1) {
      
      if (Serial1.available() > 0) {
        Serial.print(Serial1.read());
      }
      if(millis() > testTime) break;
    }
    Serial.println(" ");
    Serial.println("Done.");
    digitalWrite(pin_radioSet, LOW); 

}

void toggleRset() {
  Serial.print("RadioSet Toggle is now ");
  if(digitalRead(pin_radioSet)) {
    Serial.println("OFF");
    digitalWrite(pin_radioSet, LOW);
  } else {
    Serial.println("ON");
    digitalWrite(pin_radioSet, HIGH);
  }
 
}


//***************************************************  FLASH STUFF ********************************************



void formatFlash() {

    if (!myfs.begin()) {
      Serial.println("LittleFS Flash:  *** ERROR starting Little FS ***");
      return;
    } else {
      Serial.println("LittleFS Flash:  Successfully started");
    }
    Serial.println(" ");
    Serial.println("Initiated a low-level format of flash. This will takes about 16.5 minutes...");
    myfs.lowLevelFormat('.');
    Serial.println("Done formating flash memory");

}


void printFlashDirectory() {



  Serial.println("Printing Flash Directory: ");
  if (!myfs.begin()) {
    Serial.println("Error starting spidisk");
  }

  
  printDirectory(myfs.open("/"), 0);
  Serial.println();
  freeSpaces();

}

  void freeSpaces() {

  unsigned long bFree = 0;
  bFree = myfs.totalSize() - myfs.usedSize();
  Serial.printf("Bytes Used: %llu, Bytes Total:%llu", myfs.usedSize(), myfs.totalSize());
  Serial.print("    Bytes Free: ");
  Serial.println(bFree);
  Serial.println();
}

void printDirectory(File dir, int numTabs) {
  //dir.whoami();
  uint64_t fSize = 0;
  uint32_t dCnt = 0, fCnt = 0;
  if ( 0 == dir ) {
    Serial.printf( "\t>>>\t>>>>> No Dir\n" );
    return;
  }
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      Serial.printf("\n %u dirs with %u files of Size %u Bytes\n", dCnt, fCnt, fSize);
      fTot += fCnt;
      totSize1 += fSize;
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }

    if (entry.isDirectory()) {
      Serial.print("DIR\t");
      dCnt++;
    } else {
      Serial.print("FILE\t");
      fCnt++;
      fSize += entry.size();
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println(" / ");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
    //Serial.flush();
  }
}


void copy2SD() {
    Serial.println("Copying Flash to SD card... ");
    // Initializing the SD Card
    if(!SD.begin(BUILTIN_SDCARD)){
        Serial.println(F("SD Card Mount Failed"));
        return;
    } else {
        Serial.println(F("Good SD Card mounted"));
    }
  if (!myfs.begin()) {
    Serial.println("Error starting Flash Disk");
    return;
  }
  delay(500);
    
   uint64_t fSize = 0;
  // Cycle through directory
      File mdir;
      mdir = myfs.open("/");
    while (true) {  
      File entry =  mdir.openNextFile();
      if (! entry) {
        // no more files
        Serial.println("No more flash files to copy ");
        totSize1 += fSize;
        break;
      }
      
      if (entry.isDirectory()) {
        Serial.print("Skipping Directory: ");
        Serial.println(entry.name());
      } else {
        Serial.print("Copying File ");
        Serial.print(entry.name());
        Serial.print("   Size = ");
        fSize = entry.size();      
        Serial.println(fSize);
        // Copy it to SD
        // Delete and open from SD
        char sName[30];
        strcpy(sName,"/");  
        strcat(sName,entry.name());
        SD.remove(sName);
        tempFile = SD.open(sName, FILE_WRITE);
        // Open for reading and writing from flash
        char buf2[1];
        file = myfs.open(entry.name(), FILE_READ);
       
        while(file.available()) {         
         file.read(buf2, 1);
         tempFile.print(buf2[0]);
        }
        file.close();
        tempFile.close();        
      }
      entry.close();
    }
    
}


void eraseFlash() {
  Serial.println("Quick formatting flash...");
  myfs.quickFormat();
  //myfs.lowLevelFormat('.');
  Serial.println("Done formating flash memory");
  Serial.println(" ");

}


void flashLogging() {


    if (!myfs.begin()) {
      Serial.println("LittleFS Flash:  *** ERROR starting Little FS ***");
      return;
    } else {
      Serial.println("LittleFS Flash:  Successfully started");
    }
    Serial.println("Process:  Deleting File");
    myfs.remove(fname1);
    Serial.println("Process:  Writing File");
    strcpy(buf,"Testing");
    file = myfs.open(fname1, FILE_WRITE);
    delay(10);
    file.println(buf);
    file.close();   
    Serial.println("Process:  Reading File");
    char buf2[1];
    char buf3[100];
    file = myfs.open(fname1, FILE_READ);
    bool fLetter = false;
    bool fCorrect = false;
    int bufCount = 0;
    while(file.available()) {
     file.read(buf2, 1);
     buf3[bufCount] = buf2[0];
     bufCount++;
    }
    file.close();
    char *ptr = strstr(buf3, "Test");
    if (ptr != NULL) { /* Substring found */
      Serial.println("Process:  Successful File Read");
    } else {
      Serial.println("Process:  File Read Failed");
      return;
    }
    Serial.println("Process:  Conducting performance test...");
    unsigned long theStop;
    int theCounter = 0;
    strcpy(buf,"Line 1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,Line 1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,Line 1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10");
    myfs.remove(fname1);
    delay(200);
    theStop = millis() + 1000;
    while(millis() < theStop) {
     file = myfs.open(fname1, FILE_WRITE);
     file.println(buf);
     file.close(); 
     theCounter++;
    }
    Serial.print("Flash Logging Speed Result = ");
    Serial.print(theCounter);
    Serial.println("  should be at least 13 (98 after low level format)");   
    
}

void readConfig() {


    if (!myfs.begin()) {
      Serial.println("LittleFS Flash:  *** ERROR starting Little FS ***");
      return;
    } else {
      Serial.println("LittleFS Flash:  Successfully started");
    }
    Serial.println("Reading Config File....");
    Serial.println("  ");
    Serial.println("  ");

    char fname[30];
    char chIn = 255;
    strcpy(fname,"/config.txt");

    file = myfs.open(fname, FILE_READ);
    if (file) {
      Serial.println("File open successful. Here are the contents:");
    } else {
      Serial.println("ERROR OPENING FILE");
      return;
    }

    while(file.available()) {
     //file.read(buf2, 1);
     //buf3[bufCount] = buf2[0];
     //bufCount++;
     char buf = file.read();
     Serial.print(buf);
    }
    file.close();

  

}

void readFile() {


    if (!myfs.begin()) {
      Serial.println("LittleFS Flash:  *** ERROR starting Little FS ***");
      return;
    } else {
      Serial.println("LittleFS Flash:  Successfully started");
    }
    Serial.println("Reading File....");
    Serial.println("  ");
    Serial.println("  ");

    char fname[30];
    char chIn = 255;
    strcpy(fname,"tempdir.tmp");

    file = myfs.open(fname, FILE_READ);
    if (file) {
      Serial.println("File open successful. Here are the contents:");
    } else {
      Serial.println("ERROR OPENING FILE");
      return;
    }

    while(file.available()) {
     //file.read(buf2, 1);
     //buf3[bufCount] = buf2[0];
     //bufCount++;
     char buf = file.read();
     Serial.print(buf);
    }
    file.close();

  

}
