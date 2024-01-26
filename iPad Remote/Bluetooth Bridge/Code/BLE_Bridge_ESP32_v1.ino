/*  
 *  Works with FAR GSE Mini iPad App

   M.Brinker 2021-2023
    
    BLE Code Credit:
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updated by chegewara
    https://www.uuidgenerator.net 

    RocketTalk Serial TTY to Bluetooth BLE repeater bridge
    v1.0 used with FAR GSE 2.5
    
    Updated:  6/27/23    
                    
*/


bool serialDebug = false;  // set to true to get Serial output for debugging

//Radio
  #define RXD2 32
  #define TXD2 33  
  #define pinBatt 27
  #define pinArm 13
  #define pinLED 23
  #define pinSet 25
  #define internalLED 2 // internal LED

  // *** Radio Setup
  char radioMessage[150];
  int newWord = 0;
  char theWord[200];
  bool radioError = false;
  unsigned long radioErrorTimer = 0;

  unsigned long theTimer = 0;
  unsigned long battTimer = 0;
  unsigned long LEDtimer = 0;
  bool LEDon = false;
  int battPercent = 0;
  bool armDeny = false;
  

//------BLE-------------------------------------------------------------
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>

  BLEServer* pServer = NULL;
  BLECharacteristic* pCharacteristic = NULL;
  bool deviceConnected = false;
  bool oldDeviceConnected = false;
  bool bHello = false;
  uint32_t value = 0;
  std::string rxValue;
  float txValue = 0;
  
  #define SERVICE_UUID        "3f0cf7c9-ee33-4449-88b8-4e05fdf2c163"
  #define CHARACTERISTIC_UUID_RX "2347c909-41b0-466e-b984-267cfca13359"
  #define CHARACTERISTIC_UUID_TX "b339bc4e-a4cd-41aa-a286-7e05f0d1465f"
  #define DEVICE_NAME "RocketTalk_14433"
  //https://www.guidgenerator.com/online-guid-generator.aspx



//**************************************************************************  BLE CLASS CODE ***********************************************

class MyServerCallbacks: public BLEServerCallbacks { //This is for device/server disconnect and connect
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      if(serialDebug) Serial.println("Device Connected");
    };

    void onDisconnect(BLEServer* pServer) {
      if(serialDebug) Serial.println("Device Disconnected");
      deviceConnected = false;
      bHello = false;
    }
};



class MyCallbacks: public BLECharacteristicCallbacks { //This is for RX from iPhone to BLE
    void onWrite(BLECharacteristic *pCharacteristic) {
    rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        if(serialDebug) Serial.print("Received Value: ");
        strcpy(radioMessage, "");
        for (int i = 0; i < rxValue.length(); i++) {
          radioMessage[i] = rxValue[i];
          if(serialDebug) Serial.print(rxValue[i]);
          radioMessage[i + 1] = '\0';
        }
        
        if(serialDebug) Serial.println("");
        // ---------------------- check if key switch is armed -------------------------------------
        if (strncmp("#ARM1,",radioMessage,6) == 0) { 
          if(digitalRead(pinArm) == HIGH) { // deny
           armDeny = true;
           return;
          }
        }
        if (strncmp("#BLEBATT",radioMessage,8) == 0) {  //send battery status now
          battTimer = millis() + 1000;
          return;
        }
        RadioSend();
        }
        
      }

    void RadioSend() {
     
     digitalWrite(pinLED, HIGH);digitalWrite(internalLED, HIGH);LEDon = true;LEDtimer = millis() + 500;
     Serial1.print(radioMessage);
     Serial1.flush(); //waits for outgoing transmission to complete
     if(serialDebug) Serial.print("SENT: ");
     if(serialDebug) Serial.println(radioMessage); 
    }


};



//**************************************************************************  SETUP ***********************************************
void setup() {

  pinMode(pinArm, INPUT_PULLUP);
  pinMode(pinBatt, INPUT);
  pinMode(pinLED, OUTPUT); digitalWrite(pinLED, LOW);
  pinMode(pinSet, OUTPUT); digitalWrite(pinSet, LOW);
  pinMode(internalLED, OUTPUT);digitalWrite(internalLED, LOW);
  
  Serial.begin(115200);
  if(serialDebug) Serial.println("Starting... ");
  
  //Radio Setup
  
  delay(2500);
  radioTest();
  delay(200);
  Serial1.begin(19200, SERIAL_8N1, RXD2, TXD2);
  
  // Create the BLE Device
  BLEDevice::init(DEVICE_NAME);
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );                    
  pCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // set value to 0x00 to not advertise this parameter
  pAdvertising->setMinPreferred(0x12);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  
  if(serialDebug) Serial.println("Self Test Report...");
  checkVoltage();
  if(!radioError && battPercent > 5 ) {  //not much to check :) - three quick is good
    digitalWrite(pinLED, HIGH);digitalWrite(internalLED, HIGH);delay(500);digitalWrite(pinLED, LOW);digitalWrite(internalLED, LOW);delay(500);
    digitalWrite(pinLED, HIGH);digitalWrite(internalLED, HIGH);delay(500);digitalWrite(pinLED, LOW);digitalWrite(internalLED, LOW);delay(500);
    digitalWrite(pinLED, HIGH);digitalWrite(internalLED, HIGH);delay(500);digitalWrite(pinLED, LOW);digitalWrite(internalLED, LOW);
    if(serialDebug) Serial.println("Self Test Good.");
  } else {
    if(radioError) {digitalWrite(pinLED, HIGH);digitalWrite(internalLED, HIGH);delay(3000);digitalWrite(pinLED, LOW);digitalWrite(internalLED, LOW);if(serialDebug) Serial.println("Error Radio.");} // one long is radio error
    if(battPercent <= 5) {
      digitalWrite(pinLED, HIGH);digitalWrite(internalLED, HIGH);delay(3000);digitalWrite(pinLED, LOW);digitalWrite(internalLED, LOW);delay(1000);  // two long ones is battery low
      digitalWrite(pinLED, HIGH);digitalWrite(internalLED, HIGH);delay(3000);digitalWrite(pinLED, LOW);digitalWrite(internalLED, LOW);
      if(serialDebug) Serial.println("Error Battery.");
    }
  }
  
  if(serialDebug) Serial.println("Waiting a BLE client connection to notify...");
  battTimer = millis() + 5000;
  
}

//**************************************************************************  MAIN LOOP ***********************************************
void loop() {

    //------------- Radio TX send --------------------------------------
    // is above in the BLE class under RadioSend()
    

    //------------- Inbound Radio Check --------------------------------------
    checkRadio();

    //------------- Battery Voltage Check Check --------------------------------------
    if(millis() > battTimer) {
      checkVoltage();
    }

    //------------- Bluetooth Connection Check  --------------------------------------
    // Bluetooth disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        if(serialDebug) Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
        bHello = false;
    }
    // Bluetooth connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
        if(serialDebug) Serial.println("connecting loop");
    }

    //------------- First Connection Hello  --------------------------------------
    if (deviceConnected && bHello == false) {
      delay(500);
      char txString[200]; 
      strcpy(txString, "#BM,Bluetooth Hello,!");
      pCharacteristic->setValue(txString);
      pCharacteristic->notify(); // Send the value to the app!
      bHello = true;
      checkVoltage();

      
    }

    //------------- Radio Error Check  --------------------------------------
    if (radioErrorTimer < millis() && radioError == true) {
      if(serialDebug) Serial.println("Retesting Radio");
      delay(1000);
      radioTest();  //retest it
      delay(400);
      Serial1.begin(19200, SERIAL_8N1, RXD2, TXD2);
      char txString[200]; 
      if(deviceConnected) {
        if (radioError == true) {
          strcpy(txString, "#BRE,1,!");
          if (deviceConnected) {
            pCharacteristic->setValue(txString);
            pCharacteristic->notify(); // Send the value to the app!
          }
          
        } else {
          char txString[200]; 
          strcpy(txString, "#BRE,0,!");
          if (deviceConnected) {
            pCharacteristic->setValue(txString);
            pCharacteristic->notify(); // Send the value to the app!
          }
        }
      }
      radioErrorTimer = millis() + 30000;
    }

    //------------- Turn off LED  --------------------------------------
    if(millis() > LEDtimer && LEDon) {
      digitalWrite(pinLED, LOW);
      digitalWrite(internalLED, LOW);
      LEDon = false;
    }
    
    //------------- Arm Key Deny  --------------------------------------
    if(armDeny) {
      delay(200);
      if (deviceConnected) {
        char txString[200]; 
        strcpy(txString, "#BM,ARM DENIED: iPad Safety Key is Off,!");
        pCharacteristic->setValue(txString);
        pCharacteristic->notify(); // Send the value to the app!
        if(serialDebug) Serial.println("ARM Denied");
        armDeny = false;
      }
    }
    
}






//**************************************************************************  BASIC LOGIC ***********************************************

void checkVoltage() {

     /*  Low voltage circuit cuts out at 6.85V but won't allow turn-on at 7.2V
      *  Full 8.4V to 7.2V is 2450 to 2092.  Will go to 1982 (6.85V) but can't power on
      *  So 100% range is 2090 to 2450 = 360 range and a base of 2092.
      * 
      */
  
      float theTemp = 0.0;
      battPercent = 0;
      int theReading = 0;
      theReading = analogRead(pinBatt);
      theReading += analogRead(pinBatt);
      theReading += analogRead(pinBatt);
      theReading = theReading / 3;
      theTemp = (float) theReading - 2092.0;
      if(theTemp <= 0) {
        battPercent = 0;
      } else {
        theTemp = theTemp / (float) 360.0 * (float) 100.0;       
        if(theTemp >= 100.0) {
          battPercent = 100;
        } else {
          battPercent = int (theTemp);
        }
      }
      
      if(serialDebug) {
        Serial.print("Batt Reading:  "); Serial.println(theReading);
        Serial.print("Batt Percent:  "); Serial.println(battPercent);
      }


      // Now send it
      char txString[200]; 
      char str_int[16];
      strcpy(str_int, ""); 
      sprintf(str_int, "%d", battPercent);
      strcpy(txString, "#BV,");
      strcat(txString, str_int);
      strcat(txString, ",!");
      if (deviceConnected) {
        pCharacteristic->setValue(txString);
        pCharacteristic->notify(); // Send the value to the app!
      }      
      battTimer = millis() + 90000; // once every 90 sec

            // also send positive radio status
      delay(200);
      if(radioError == false) { 
          strcpy(txString, "#BRE,0,!");
          if (deviceConnected) {
            pCharacteristic->setValue(txString);
            pCharacteristic->notify(); // Send the value to the app!
          }
      } else {
        strcpy(txString, "#BRE,1,!");
          if (deviceConnected) {
            pCharacteristic->setValue(txString);
            pCharacteristic->notify(); // Send the value to the app!
          }
      }

      
}

void checkArmSwitch() {

      if(serialDebug) {
        if(digitalRead(pinArm) == LOW) {
          Serial.println("Arm is ON");
        } else {
          Serial.println("Arm is OFF");
        }
      }

  
}

void radioTest() {
  
    digitalWrite(pinSet, HIGH); //turn radio set pin to low
    
    delay(200);
    Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
    
    char s[5];
    strcpy(s,"");
    
    s[0] = 0xaa;
    s[1] = 0xfa;
    s[2] = 0xaa; // aa = product model number 
    s[3] = 0x0d; //  /r termination
    s[4] = 0x0a; //  /n termination
    Serial1.println(s);

    radioTestRead();
    if(serialDebug) Serial.print("Radio Response:  ");
    if(serialDebug) Serial.println(theWord);
    
    char *ptr = strstr(theWord, "LORA611");
    if (ptr == NULL) {
      radioError = true; 
      if(serialDebug) Serial.println("ERROR:  No Radio Found");
    } else {
      radioError = false;
    }
    
    digitalWrite(pinSet, LOW);
    
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
          if(serialDebug) Serial.print("Radio Timeout");
          break;
       }
       delay(1);
     }
}

//**************************************************************************  RADIO RX ***********************************************




void checkRadio() {  // **********  CHECK RADIO  *****************
   newWord = 0;
   int vtimeout = 0;
   char receivedChar;
   if (Serial1.available() > 0) {
    strcpy(theWord, "");
    if(serialDebug) Serial.println("radio inbound detect");
    unsigned long testTime = millis() + 700; 
    while (newWord != 1) {
       if(Serial1.available()) {
         receivedChar = Serial1.read();
         append(theWord, receivedChar);
         if(receivedChar == 33) {  // look for ! to end
          newWord = 1;
         }
       }
       if(millis() > testTime) {  // exit and evaluate if it takes too long
          newWord =1;
          vtimeout = 1;
          if(serialDebug) Serial.println("timeout exit error");
          if(serialDebug) Serial.println(theWord);
          exit;
       }
    }
    if(serialDebug) Serial.println(theWord);
    if (vtimeout == 0) ProcessRadio();
   }
}
   

void ProcessRadio() {   // **********  PROCESS RADIO COMMAND  *****************

      int chkNew = 0;
      char theID[20];
      digitalWrite(pinLED, HIGH);digitalWrite(internalLED, HIGH);LEDon = true;LEDtimer = millis() + 500;
 
      //send to BLE
      if (deviceConnected) {
        pCharacteristic->setValue(theWord);
        pCharacteristic->notify(); // Send the value to the app!
        if(serialDebug) Serial.println("*** Sent BLE Value");
      }
      newWord = 0;
      strcpy(theWord,"");
}

void append(char* s, char c) {  //used to concat char to char array
        int len = strlen(s);
        s[len] = c;
        s[len+1] = '\0';
}
