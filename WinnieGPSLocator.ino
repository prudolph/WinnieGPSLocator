
#define TINY_GSM_MODEM_SIM808
#define PIN_TX 3
#define PIN_RX 2
#define KEY_PIN 8
#define PWR_PIN 7
#define MAIN_BATT_PIN A5


#define X_PIN A0
#define Y_PIN A1
#define Z_PIN A2

#define DEBUG false 


#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#include "LowPower.h"
#include "Config.h"

// Your GPRS credentials
const char apn[]  = "hologram";

short RetryLimit=3;
//bool needPositionUpdate = false; 
bool PowerState=false;
bool MovementActive=false;

//Sends location data on first movement. 
//Waits 5 minutes to check for more movement.

unsigned long MovementEndThreshold = 600000;//Send Message after 5 mins no movement
unsigned long MovementStartTime;
unsigned long TIMEOFFSET=0;

//Movement
short xVal,yVal,zVal=0;
short mvmtThreshhold=5;

SoftwareSerial SerialAT(PIN_TX,PIN_RX ); // TX, RX
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

typedef struct{ String lat,lon;} Position;

//signitures
void enableCellPower(bool state);
void connectToModem();
bool connectToNetwork();
void disconnectModem();
void postData(String dataStr);
void sendLocationData(bool mvmtStart);

Position getGPSPosition();
//Position getCellPosition();
void sleepMode();

bool checkForMovement();
Position lastGPSPos;

void setup(){

    //Set Pin Modes
      pinMode(KEY_PIN, OUTPUT);  
      pinMode(PWR_PIN, INPUT);  
      pinMode(MAIN_BATT_PIN, INPUT);  
      pinMode(X_PIN, INPUT);  
      pinMode(Y_PIN, INPUT);  
      pinMode(Z_PIN, INPUT);  
      
      digitalWrite(KEY_PIN, HIGH);
      
      //HIGH== ON
      PowerState = digitalRead(PWR_PIN);
  
      xVal = analogRead(X_PIN);
      yVal = analogRead(Y_PIN);
      zVal = analogRead(Z_PIN);
      
  // Set console baud rate
  #if DEBUG 
  Serial.begin(115200);
  #endif
 
  //If Just turned on let the device settle for a few seconds
  delay(500);

  sleepMode();
  

}

void loop(){
  
  #if DEBUG
  Serial.println(F("Checking for movement"));
  #endif

  if(checkForMovement() ){  
    
    if(MovementActive ==false){
      
      MovementActive=true;
      enableCellPower(true);
      connectToModem();
      modem.enableGPS();
      connectToNetwork();
      updatePosition();
      
      sendLocationData(MovementActive);

      sleepMode();

    
    }
  }
    
  unsigned long elapsedTimeSinceLastMvmt = (millis()+TIMEOFFSET) - MovementStartTime;
        

  if ((elapsedTimeSinceLastMvmt > MovementEndThreshold) && MovementActive){
       MovementActive=false;
       #if DEBUG
       Serial.println(F("-------Movement STOPPED"));
       #endif
       
      MovementActive=false;
      enableCellPower(true);
      connectToModem();
      modem.enableGPS();
      connectToNetwork();
      updatePosition();

      sendLocationData(MovementActive);

      sleepMode(); 
     }


  #if DEBUG
  Serial.println(F("Deep Sleep"));
  Serial.end();
  #endif
  
  //update time to account for deep sleep
  TIMEOFFSET =(millis()+TIMEOFFSET)+(64000);
  
  //deep Sleep for
  for(int i=0;i<8;i++){
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  
  #if DEBUG
  Serial.begin(115200);
  #endif
}

void updatePosition(){
      //Update Positions
      lastGPSPos=getGPSPosition();
  
  }

void enableCellPower(bool state){    
    //HIGH==ON  
     PowerState = digitalRead(PWR_PIN);
 
       
    while( PowerState!=state ){ 
       #if DEBUG
       Serial.print("Setting Power:");
       Serial.println(state);
       #endif
        digitalWrite(KEY_PIN, LOW);
        delay(2000);
        digitalWrite(KEY_PIN, HIGH);
        delay(5000);
        //HIGH==On
       PowerState = digitalRead(PWR_PIN);
    }
}

void connectToModem(){
   #if DEBUG
   Serial.println(F("-------Cnt MDM"));
   #endif
   //delay(4000);
 // Set GSM module baud rate
   SerialAT.begin(9600);
   delay(4000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  modem.restart();
  //modem.init();
   
}


bool connectToNetwork(){
        #if DEBUG
        Serial.println(F("-------Cnt NTWRK"));
        #endif

  int retryCnt=0;
   while (!modem.waitForNetwork() && retryCnt<RetryLimit) {
      #if DEBUG
      Serial.println(F(" fail to connect to networ... retrying"));
      #endif
      
       retryCnt++;
      delay(2000);
    }

 Serial.print(F("RETRYCNT "));
  Serial.print(retryCnt);
  if(retryCnt==RetryLimit){
    return false;
  }

  
  retryCnt=0;
  
   while(!modem.gprsConnect(apn, "", "") && retryCnt<RetryLimit) {
      #if DEBUG
      Serial.println(F(" APN CONN FAIL "));
      #endif
    retryCnt++;
    delay(2000);
  }
  
  if(retryCnt==RetryLimit){
    return false;
  }else{
    return true;
    }

  }

void postData(String dataStr){
/*
    unsigned long timeSinceMessageSend = (millis()+TIMEOFFSET) - LastMessageSendTime;
     #if DEBUG 
     Serial.print(F("Time Since Last MSG SND"));
     Serial.println(timeSinceMessageSend);
     #endif
*/

   
  //Disable Posting
  //if(DEBUG)return;

    int retryCnt=0;
  while (!client.connect("cloudsocket.hologram.io", 9999) && retryCnt<RetryLimit) {
      #if DEBUG 
      Serial.println(" fail connect - hologram");
      #endif
      retryCnt++;
      delay(2000);
  }

  if(retryCnt==RetryLimit){
    return false;
  }
  
  // Make a TCP GET request
  client.print(dataStr);
  

  
  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 10000L) {
    // Print available data
    while (client.available()) {
      char c = client.read();
        #if DEBUG 
        Serial.print(c);
        #endif
      timeout = millis();
    }
  }

  client.stop();  
  }

void sendLocationData(bool mvmtStart){
    if(DEBUG) Serial.println(F("Sending Location Data"));
    
    StaticJsonBuffer<190> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["k"]=CSR_Cred;
    
    JsonArray& tags = root.createNestedArray("t");
    tags.add("loc");
    
   
    StaticJsonBuffer<110> dataBuffer;
    JsonObject& dataRoot = dataBuffer.createObject(); 
     dataRoot["Glt"] =lastGPSPos.lat;
     dataRoot["Gln"] =lastGPSPos.lon;
     
     dataRoot["cb"] =modem.getBattPercent();
     dataRoot["mb"]= digitalRead(MAIN_BATT_PIN);
     dataRoot["mvSt"]= mvmtStart;
     
     String payloadStr;
     dataRoot.printTo(payloadStr);

     root["d"] = payloadStr;
    #if DEBUG
      root.prettyPrintTo( Serial);
    #endif
    
    String dataString;
    
    root.printTo(dataString);
    postData(dataString);
  } 



Position getGPSPosition(){
    float lat,  lon=0;

      short retryCnt=0;
      bool fix= modem.getGPS(&lat, &lon) ;
      while (!fix && retryCnt<RetryLimit){
         #if DEBUG
         Serial.println(F("No Sat FIX...Retry"));        
         #endif
         retryCnt++;
         //delay(10000);
         modem.maintain();
      }
  
     Position pos;

     pos.lat = lat;
     pos.lon = lon;
    return pos;
  }

  /*
Position getCellPosition(){
  String gsmLoc = modem.getGsmLocation();

  char gsmCharBuf[gsmLoc.length()+1];
  gsmLoc.toCharArray(gsmCharBuf,gsmLoc.length()+1);
  String results[5];

  int resultIndex=0;
  for (char *p = strtok(gsmCharBuf,","); p != NULL; p = strtok(NULL, ","))
  {
    results[resultIndex] = String(p);
    resultIndex++;
  }
  
     Position pos;
     pos.lat = results[2];
     pos.lon = results[1];


    return pos;
  }
*/

void sleepMode(){

         #if DEBUG
         Serial.println("Power Down");
         #endif
     
         modem.disableGPS();
         modem.gprsDisconnect();
          SerialAT.end();
         enableCellPower(false);
  }

  bool checkForMovement(){
      bool movement = false;
      short xCurVal,yCurVal,zCurVal=0;
      
      for(short i=0;i<100;i++){
         xCurVal = analogRead(X_PIN);
         yCurVal = analogRead(Y_PIN);
         zCurVal = analogRead(Z_PIN);

        short xDiff=  abs(xCurVal-xVal);
        short yDiff=  abs(yCurVal-yVal);
        short zDiff=  abs(zCurVal-zVal);

        if (xDiff>mvmtThreshhold ||
            yDiff>mvmtThreshhold||
            zDiff>mvmtThreshhold){
             
             movement=true;  
             MovementStartTime=(millis()+TIMEOFFSET);
             break;
        }
  
        delay(10);
      }

      //Update Values
         xVal = xCurVal;    
         yVal = yCurVal;    
        zVal = zCurVal; 
         return movement;
   }

