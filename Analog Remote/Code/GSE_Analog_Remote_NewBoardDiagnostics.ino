


/* GSE Mini Analog Remote new board diagnostics
 *  
 *  Used to test all sensors, inputs and outputs on a new remote
 *  Current controller hardware version is 2.0pro
 *  
 *  Uses MCP23017 library by Bertrand Lemasle
 *  
 *  
 *  TO DO:
 *    - none
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
 */


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
  #define LED_FLV 7 //M1
  #define LED_FQD 3 //M1
  #define LED_ORV 1 //M2
  #define LED_OPR 4 //M2
  #define LED_OLV 6 //M1
  #define LED_OQD 4 //M1
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
/*  pinMode(LED_status , OUTPUT); digitalWrite(LED_status , HIGH); //hw issue
  pinMode(LED_radio , OUTPUT); digitalWrite(LED_radio , HIGH); //hw issue
  pinMode(LED_error , OUTPUT); digitalWrite(LED_error , HIGH); //hw issue
*/  pinMode(LED_BU_FP , OUTPUT); digitalWrite(LED_BU_FP , LOW);
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

  delay(2000);
  Serial.begin(115200);
  
  delay(100);
  Serial.println("GSE Mini Analog Remote New Board Diagnostics and Health Check");
  Serial.println(" ");Serial.println(" ");

// MCP start
  Wire.begin(400000);
  if (!mcp1.begin_I2C(0x27)) {
    Serial.println("Error starting MCP1");
  }
  if (!mcp2.begin_I2C(0x20)) {
    Serial.println("Error starting MCP2");
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


  printCommands();

}

//************************************************************************************************************************************************************  MAIN LOOP
//************************************************************************************************************************************************************  MAIN LOOP

void loop() {

  char chIn = 255;
  if ( Serial.available() ) {
    do {
     if ( chIn != '0' && chIn != '1' && chIn != '2' && chIn != '$' && chIn != '#' && chIn != '3' && chIn != '4' && chIn != '5' && chIn != '6' && chIn != '7' && chIn != '8' && chIn != '9' && chIn != 'a' && chIn != 'b' && chIn != 'c' && chIn != 'd' && chIn != 'e' && chIn != 'f' && chIn != 'g' && chIn != 'h' && chIn != 'i' && chIn != 'j' && chIn != 'k' && chIn != 'l' && chIn != 'm' && chIn != 'n' && chIn != 'o' && chIn != 'p' && chIn != 'q' && chIn != 'r' && chIn != 's' && chIn != 't'&& chIn != 'u'&& chIn != 'v'&& chIn != 'w'&& chIn != 'x'&& chIn != 'y'&& chIn != 'z' )
        chIn = Serial.read();
      else
        Serial.read();
    }
    while ( Serial.available() );
  }

  switch(chIn) {


    case '4':
      toggleLB(); printCommands(); break;

    case 'd':
      testDisplay(); printCommands(); break;  

      
    case 'p':
      potTest(); printCommands(); break;
    case 'o':
      potDispTest(); printCommands(); break;  
          
    case 'q':
      eraseFlash(); printCommands(); break;
    case 'r':
      printFlashDirectory(); printCommands(); break;
    case 'c':
      copy2SD(); printCommands(); break;
    case 't':
      flashLogging(); printCommands(); break;
      
    case 'x':
      I2Cscan(); printCommands(); break;
    case 'j':
      beepOn(); printCommands(); break;
    case 'k':
      beepOff(); printCommands(); break;      
    case 'l':
      LEDall(); printCommands(); break;   
    case 's':
      switchesTest(); printCommands(); break;  
    case 'u':
      checkTemp(); printCommands(); break;
    case 'v':
      checkVoltage(); printCommands(); break;
    case '9':
      RadioTXtest();printCommands(); break;
    case 'z':
      radioTest();printCommands(); break;
    case '#':
      readFile();printCommands(); break;
  }
  
}


void printCommands() {
  Serial.println("  ");
  Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
  Serial.println("COMMANDS:");
  Serial.println("        I/O:  d-Test 7s Displays, p-Pot Test, o-Pot Display Test, s-Switches Test, l-LED all test, j-beep on, k-beep off"); 
  Serial.println("      Other:  4-LED builtin, u-CPU Temp, v-voltage");   
  Serial.println("        PTs:  x-I2C Scan, ");
  Serial.println("      Radio:  z-Radio Test, 9-Radio TX");
  Serial.println("      FLASH:  p-Low Format, q-Quick Erase, r-Directory, c-copy to SD, t-Flash Logging Test, #-read file");  
  Serial.println("-----------------------------------------------------------------------------------------------------------------------------------------------------------");  
  Serial.println("  ");
}

void switchesTest() {

  Serial.println("Testing ALL switches and buttons on 30 second loop...");
  unsigned long theTimer;
  theTimer = millis() + 30000;
  while(millis() < theTimer) {

    if(digitalRead(SW_POT_FU) == LOW) {Serial.println("Fuel Pot Button");delay(500);}
    if(digitalRead(SW_POT_OX) == LOW) {Serial.println("Lox Pot Button");delay(500);}
    if(digitalRead(BU_FP) == LOW) {Serial.println("Button Fuel Pressurize");delay(500);}
    if(digitalRead(BU_FQD) == LOW) {Serial.println("Button Fuel QD");delay(500);}
    if(digitalRead(BU_OP) == LOW) {Serial.println("Button Ox Pressurize");delay(500);}
    if(digitalRead(BU_OQD) == LOW) {Serial.println("Button Ox QD");delay(500);}
    if(digitalRead(SW_FRV_o) == LOW) {Serial.println("Switch FRV Open");delay(500);}
    if(digitalRead(SW_FRV_c) == LOW) {Serial.println("Switch FRV Close");delay(500);}
    if(mcp2.digitalRead(SW_FPR_o) == LOW) {Serial.println("Switch FPR Open");delay(500);}
    if(mcp2.digitalRead(SW_FPR_c) == LOW) {Serial.println("Switch FPR Close");delay(500);}
    if(mcp1.digitalRead(SW_FLV_o) == LOW) {Serial.println("Switch FLV Open");delay(500);}
    if(mcp1.digitalRead(SW_FLV_c) == LOW) {Serial.println("Switch FLV Close");delay(500);}
    if(mcp2.digitalRead(SW_FQD_o) == LOW) {Serial.println("Switch FQD Open");delay(500);}
    if(mcp2.digitalRead(SW_FQD_c) == LOW) {Serial.println("Switch FQD Close");delay(500);}
    if(digitalRead(SW_ORV_o) == LOW) {Serial.println("Switch ORV Open");delay(500);}
    if(digitalRead(SW_ORV_c) == LOW) {Serial.println("Switch ORV Close");delay(500);}
    if(digitalRead(SW_OPR_o) == LOW) {Serial.println("Switch OPR Open");delay(500);}
    if(digitalRead(SW_OPR_c) == LOW) {Serial.println("Switch OPR Close");delay(500);}
    if(mcp1.digitalRead(SW_OLV_o) == LOW) {Serial.println("Switch OLV Open");delay(500);}
    if(mcp1.digitalRead(SW_OLV_c) == LOW) {Serial.println("Switch OLV Close");delay(500);}
    if(mcp2.digitalRead(SW_OQD_o) == LOW) {Serial.println("Switch OQD Open");delay(500);}
    if(mcp2.digitalRead(SW_OQD_c) == LOW) {Serial.println("Switch OQD Close");delay(500);}
    if(digitalRead(SW_REC_on) == LOW) {Serial.println("Switch Recovery On");delay(500);}
    if(digitalRead(SW_REC_off) == LOW) {Serial.println("Switch Recovery Off");delay(500);}
    if(mcp2.digitalRead(SW_MON) == LOW) {Serial.println("Switch Main On");delay(500);}
    if(mcp2.digitalRead(SW_MOF) == LOW) {Serial.println("Switch Main Off");delay(500);}
    if(digitalRead(BU_IGN) == LOW) {Serial.println("Button Igniter");delay(500);}
    if(mcp1.digitalRead(BU_ABORT) == LOW) {Serial.println("Button Abort");delay(500);}

    if(digitalRead(pin_arm) == LOW) {Serial.println("ARM Switch");delay(500);}
    if(digitalRead(pin_2FA) == LOW) {Serial.println("2FA Button");delay(500);}
    
    
  }
}


void checkVoltage() {

    Serial.println("Checking voltage values");
    int val1 = analogRead(pin_voltage);
    int val2 = analogRead(pin_voltage);
    int val3 = analogRead(pin_voltage);
    float tVolts = ((float) val1 + (float) val2 + (float) val3) / 3.0;
    Serial.print("Analog Read is: ");
    Serial.println((int) tVolts);
    

  
}



void potTest() {

  Serial.println("Testing Pots on 30 second loop. Adjust then use button to report.");
  unsigned long theTimer;
  theTimer = millis() + 30000;
  int pot1;
  int pot2;
  while(millis() < theTimer) {
    pot1 = analogRead(POT_FU);
    pot2 = analogRead(POT_OX);
    if(digitalRead(SW_POT_FU) == LOW) {
      Serial.print("Fuel Pot = ");
      Serial.println(pot1);
      delay(500);
    }
    if(digitalRead(SW_POT_OX) == LOW) {
      Serial.print("Lox Pot = ");
      Serial.println(pot2);
      delay(500);
      }
  }
}

void potDispTest() {

  Serial.println("Testing Pots on 30 second loop. Reported to the LED Displays Only");
  unsigned long theTimer;
  theTimer = millis() + 30000;
  int pot1;
  int pot1m;
  int pot2;
  int pot2m;
  int theRange = 500;

  TM1637TinyDisplay display2(CLK2, DIO2);
  TM1637TinyDisplay display4(CLK4, DIO4);
  display2.setBrightness(BRIGHT_7);
  display4.setBrightness(BRIGHT_7);
  display2.clear();
  display4.clear();
  
  while(millis() < theTimer) {
    pot1 = analogRead(POT_FU);
    if (pot1 != pot1m) {
      pot1m = pot1;
      float i= (float)((float) pot1 / 1023.0) * (float) theRange;
      display2.showNumber((int) i, false, 3, 0);
    }

    pot2 = analogRead(POT_OX);
    if (pot2 != pot2m) {
      pot2m = pot2;
      int i= (pot2 / 1023) * theRange;
      display4.showNumber(i, false, 3, 0);
    }

    if(digitalRead(SW_POT_FU) == LOW) {
      Serial.print("Fuel Pot = ");
      Serial.println(pot1);
      delay(500);
    }
    if(digitalRead(SW_POT_OX) == LOW) {
      Serial.print("Lox Pot = ");
      Serial.println(pot2);
      delay(500);
      }
  }
}




void LEDall() {

  Serial.println("Testing ALL LEDs in order...");
  delay(1000);
  digitalWrite(LED_armR, HIGH);delay(1000);digitalWrite(LED_armR, LOW);
  digitalWrite(LED_armG, HIGH);delay(1000);digitalWrite(LED_armG, LOW);
  pinMode(LED_status , OUTPUT); digitalWrite(LED_status, LOW);delay(1000);pinMode(LED_status , INPUT);
  pinMode(LED_radio , OUTPUT); digitalWrite(LED_radio, LOW);delay(1000);pinMode(LED_radio , INPUT);
  pinMode(LED_error , OUTPUT); digitalWrite(LED_error, LOW);delay(1000);pinMode(LED_error , INPUT);
  
 /* digitalWrite(LED_status, LOW);delay(1000);digitalWrite(LED_status, HIGH);
  digitalWrite(LED_radio, LOW);delay(1000);digitalWrite(LED_radio, HIGH);
  digitalWrite(LED_error, LOW);delay(1000);digitalWrite(LED_error, HIGH);
 */ digitalWrite(LED_BU_FP, HIGH);delay(1000);digitalWrite(LED_BU_FP, LOW);
  digitalWrite(LED_BU_FQD, HIGH);delay(1000);digitalWrite(LED_BU_FQD, LOW);
  digitalWrite(LED_BU_LP, HIGH);delay(1000);digitalWrite(LED_BU_LP, LOW);
  digitalWrite(LED_BU_OQD, HIGH);delay(1000);digitalWrite(LED_BU_OQD, LOW);
  mcp2.digitalWrite(LED_FRV, HIGH);delay(1000);mcp2.digitalWrite(LED_FRV, LOW);
  mcp2.digitalWrite(LED_FPR, HIGH);delay(1000);mcp2.digitalWrite(LED_FPR, LOW);
  mcp1.digitalWrite(LED_FLV, HIGH);delay(1000);mcp1.digitalWrite(LED_FLV, LOW);
  mcp1.digitalWrite(LED_FQD, HIGH);delay(1000);mcp1.digitalWrite(LED_FQD, LOW);
  mcp2.digitalWrite(LED_ORV, HIGH);delay(1000);mcp2.digitalWrite(LED_ORV, LOW);
  mcp2.digitalWrite(LED_OPR, HIGH);delay(1000);mcp2.digitalWrite(LED_OPR, LOW);
  mcp1.digitalWrite(LED_OLV, HIGH);delay(1000);mcp1.digitalWrite(LED_OLV, LOW);
  mcp1.digitalWrite(LED_OQD, HIGH);delay(1000);mcp1.digitalWrite(LED_OQD, LOW);
  mcp2.digitalWrite(LED_REC, HIGH);delay(1000);mcp2.digitalWrite(LED_REC, LOW);
  mcp2.digitalWrite(LED_IGN, HIGH);delay(1000);mcp2.digitalWrite(LED_IGN, LOW);
  mcp1.digitalWrite(LED_MON, HIGH);delay(1000);mcp1.digitalWrite(LED_MON, LOW);
  
}

void testDisplay() {

  Serial.println("Testing 7 Segment LCD...");
  TM1637TinyDisplay display1(CLK1, DIO1);
  TM1637TinyDisplay display2(CLK2, DIO2);
  TM1637TinyDisplay display3(CLK3, DIO3);
  TM1637TinyDisplay display4(CLK4, DIO4);
  display1.setBrightness(BRIGHT_7);
  display2.setBrightness(BRIGHT_7);
  display3.setBrightness(BRIGHT_7);
  display4.setBrightness(BRIGHT_7);
  display1.clear();
  display2.clear();
  display3.clear();
  display4.clear();
  //ID
  display1.showString("ONE");
  display2.showString("TWO");
  display3.showString("TRE");
  display4.showString("FOR");
  delay(3000);
  display1.showNumber(111, false, 3, 0);
  display2.showNumber(222, false, 3, 0);
  display3.showNumber(333, false, 3, 0);
  display4.showNumber(444, false, 3, 0);
  delay(3000);
  for(int i = 1; i < 101; i++ ) {
    display1.showNumber(i, false, 3, 0);
    display2.showNumber(100-i, false, 3, 0);
    display3.showNumber(i, false, 3, 0);
    display4.showNumber(100-i, false, 3, 0);
  }
  
  /*
  display2.showString("\xB0", 1, 3);        // Degree Mark, length=1, position=3 (right)
  display2.showNumber(333, false, 3, 0);    // Number, length=3, position=0 (left)
  delay(3000);
  display2.showString("HELLO");
  delay(2000);
  display2.showString("READY TO FILL");
  delay(2000);
  display2.showString("FILL");
  delay(2000);
  display2.showString("DONE");
  delay(2000);
  display2.showString("TEST");
  delay(2000);
  display2.showString("BURN");
  delay(2000);  
  display2.showString("ERR");
  delay(2000);  
  display2.showString("ABRT");
  delay(2000);
  */

  
  Serial.println("Test Done");

}



//***************************************************  PORT EXTENDER STUFF ********************************************

void beepOn() {
 Serial.println("Beep On");
 mcp1.digitalWrite(2, HIGH);


  
}


void beepOff() {
 Serial.println("Beep Off");
 mcp1.digitalWrite(2, LOW);
  
}



//***************************************************  LED STUFF ********************************************


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





//***************************************************  I2C STUFF STUFF ********************************************

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


void checkTemp(){
      float CPUtemp = 0.0;
      Serial.println("Checking CPU Temperature...");Serial.println(" ");
      Serial.print("CPU Temp (f) = ");
      CPUtemp  = (tempmonGetTemp() * 9.0f / 5.0f) + 32.0f;
      Serial.println(CPUtemp);
      
}

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
      checkRadio();
      if(newWord == 1) {
        Serial.print("Radio Rx:  ");
        Serial.println(theWord);
        char *ptr = strstr(theWord, "LORA611");
        if (ptr != NULL) Serial.println("Radio Set Comms Test Successful!"); 
        mcp1.digitalWrite(pin_radioSet, LOW); //turn radio set pin to low
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
        mcp1.digitalWrite(pin_radioSet, LOW); //turn radio set pin to low
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
  
    unsigned long testTime = millis() + 500; //100
    
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


/* MCP PINS
 *  
 *  MCP23x17 Pin Name  Pin ID
    21  GPA0  0
    22  GPA1  1
    23  GPA2  2
    24  GPA3  3
    25  GPA4  4
    26  GPA5  5
    27  GPA6  6
    28  GPA7  7
    1 GPB0  8
    2 GPB1  9
    3 GPB2  10
    4 GPB3  11
    5 GPB4  12
    6 GPB5  13
    7 GPB6  14
    8 GPB7  15
 *  
 *  
 *  
 *  
 *  
 */
