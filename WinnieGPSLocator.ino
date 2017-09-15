
#define TINY_GSM_MODEM_SIM808
#define PIN_TX 3
#define PIN_RX 2
#define KEY_PIN 8
#define PWR_PIN 7
#define WAKE_PIN A5

#define X_PIN A0
#define Y_PIN A1
#define Z_PIN A2

#define DEBUG true


#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#include "LowPower.h"
#include "Config.h"

// Your GPRS credentials
const char apn[]  = "hologram";

bool needPositionUpdate = false; 
bool lowPowerState=false;

//Sends location data on first movement. 
//Waits 5 minutes to check for more movement.
//check for movement for 2
unsigned long MessageSendIntervalTime = 1000*60*5;//Waits 5 minutes to check for more movement.
unsigned long MovementCheckIntervalTime = 1000*60*1;//check for movement every 1 mintue
unsigned long SleepIntervalTime = 1000*60*10;//Turn off celluar after 10 mins AFTER no movement
unsigned long MovementStartTime,MovementCheckTime, LastMessageSendTime;
unsigned long TIMEOFFSET=0;

//Movement
int xVal,yVal,zVal=0;
int mvmtThreshhold=25;

SoftwareSerial SerialAT(PIN_TX,PIN_RX ); // TX, RX
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

typedef struct{ float lat,lon;} Position;

//signitures
void enableCellPower(bool state);
void connectToModem();
void connectToNetwork();
void disconnectModem();
void postData(String dataStr);
void sendLocationData();

Position getGPSPosition();
Position getCellPosition();

void wakeUp();
void lowPowerMode();

bool checkForMovement();
Position lastGPSPos,lastCELLPos;

void setup(){

    //Set Pin Modes
      pinMode(KEY_PIN, OUTPUT);  
      pinMode(PWR_PIN, INPUT);  
      pinMode(WAKE_PIN, INPUT);  
      pinMode(X_PIN, INPUT);  
      pinMode(Y_PIN, INPUT);  
      pinMode(Z_PIN, INPUT);  
      
      digitalWrite(KEY_PIN, HIGH);
  lowPowerState = !digitalRead(PWR_PIN);
  
      xVal = analogRead(X_PIN);
      yVal = analogRead(Y_PIN);
      zVal = analogRead(Z_PIN);
      
  // Set console baud rate
  Serial.begin(115200);

  //If Just turned on let the device settle for a few seconds
  delay(1000);

  lowPowerMode();
   Serial.println(F("SETUP COMPLETE"));

}

void loop(){


    unsigned long elapsedTimeSinceLastMvmtCheck = (millis()+TIMEOFFSET) - MovementCheckTime;

    
    //Check for any movement if we havent checked recently
    if(elapsedTimeSinceLastMvmtCheck > MovementCheckIntervalTime){
      MovementCheckTime=millis();
         Serial.println(F("Checking for movement"));
     
      if(checkForMovement() ){
          Serial.println(F("-------Movement Detected"));
         if(lowPowerState) wakeUp();
      }
 
    }


  if(needPositionUpdate){
             Serial.println(F("-------Getting Position Updates"));
      Position curGPSPos = getGPSPosition();
      Position curCELLPos = getCellPosition();
     bool positionMoved =false;
      //Check if anything changed
      if(curGPSPos.lat!=lastGPSPos.lat||
         curGPSPos.lon!=lastGPSPos.lon||
         curCELLPos.lat!=curCELLPos.lat||
         curCELLPos.lon!=curCELLPos.lon){
          positionMoved =true;
          //Update Positions
             lastGPSPos=curGPSPos;
             lastCELLPos=curCELLPos;
        }

  
      unsigned long timeSinceMessageSend = (millis()+TIMEOFFSET) - LastMessageSendTime;
          Serial.println(F("Time Since Last MSG SND"));
    Serial.println(timeSinceMessageSend);
       if(timeSinceMessageSend >MessageSendIntervalTime){   
          //Send Data
          sendLocationData();
        }
  }

 
    unsigned long elapsedTimeSinceLastMvmt = (millis()+TIMEOFFSET) - MovementStartTime;  
    if (elapsedTimeSinceLastMvmt > SleepIntervalTime && !lowPowerState){//IF we have gotten past the SleepIntervalTime , Power down celluar
       Serial.println(F("No movment for a while - powerDown"));
         lowPowerMode();
     }


  delay(10000);
  /*
  if(lowPowerState){
  
     Serial.end();
     TIMEOFFSET =(millis()+TIMEOFFSET);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
    Serial.begin(115200);
    
  }else{
    delay(10000);  
  }
*/

}


void enableCellPower(bool state){      
    int pwrState = digitalRead(PWR_PIN);
      Serial.print("Setting Power:");
         Serial.println(state);
    if( pwrState!=state ){ 
         
        digitalWrite(KEY_PIN, LOW);
        delay(2000);
        digitalWrite(KEY_PIN, HIGH);
        delay(5000);
    }
}

void connectToModem(){
      Serial.println(F("-------Cnt MDM"));
 // Set GSM module baud rate
  SerialAT.begin(9600);
  delay(3000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.println(modemInfo);
}


void connectToNetwork(){
      Serial.println(F("-------Cnt NTWRK"));
   if (!modem.waitForNetwork()) {
    Serial.println(F(" fail to connect to network"));
    while (true);
  }


  if (!modem.gprsConnect(apn, "", "")) {
    Serial.println(F(" fail"));
    while (true);
  }

  }

void postData(String dataStr){
    LastMessageSendTime=(millis()+TIMEOFFSET);
    Serial.println("--------------------------");
    Serial.println("POSTING LOCATION DATA");
    Serial.println("--------------------------");
  //Disable Posting
  if(DEBUG)return;

  
  if (!client.connect("cloudsocket.hologram.io", 9999)) {
    Serial.println(" fail connect - hologram");
    delay(10000);
    return;
  }
  
  // Make a TCP GET request
  client.print(dataStr);
  

  
  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 10000L) {
    // Print available data
    while (client.available()) {
      char c = client.read();
      Serial.print(c);
      timeout = millis();
    }
  }

  client.stop();  
  }

void sendLocationData(){
    Serial.println(F("Sending Location Data"));
    
    StaticJsonBuffer<150> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["k"]=CSR_Cred;
    
    JsonArray& tags = root.createNestedArray("t");
    tags.add("loc");
    
   
    StaticJsonBuffer<50> dataBuffer;
    JsonObject& dataRoot = dataBuffer.createObject(); 
     dataRoot["Glt"] =lastGPSPos.lat;
     dataRoot["Gln"] =lastGPSPos.lon;


     dataRoot["Clt"] =lastCELLPos.lat;
     dataRoot["Cln"] =lastCELLPos.lon;


     String payloadStr;
     dataRoot.printTo(payloadStr);

     root["d"] = payloadStr;

    root.prettyPrintTo(Serial);
    String dataString;
    root.printTo(dataString);
    postData(dataString);
  } 



Position getGPSPosition(){
    float lat,  lon, spd;
    int alt, viewd_sats,  used_sats;
      
    
      bool fix= modem.getGPS(&lat, &lon, &spd, &alt, &viewd_sats, &used_sats) ;
      if(!fix){
        Serial.println(F("No Sat FIX...Retry"));        
      }
  
     Position pos;
     pos.lat = lat;
     pos.lon = lon;
    return pos;
  }
  
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
     pos.lon = results[1].toFloat();
     pos.lat = results[2].toFloat();

    return pos;
  }

void wakeUp(){   
    Serial.println("Wakeup");
    int pwrState = digitalRead(PWR_PIN);
      if( pwrState ==0){
        lowPowerState=false; 
        enableCellPower(true);
        connectToModem();
        modem.enableGPS();
      
        connectToNetwork();
        needPositionUpdate=true;
      }
      

  

   
 }

void lowPowerMode(){
    int pwrState = digitalRead(PWR_PIN);
      if( pwrState ==1){
    
        lowPowerState=true;
        Serial.println("Power Down");
        needPositionUpdate=false;
         modem.disableGPS();
         modem.gprsDisconnect();
         SerialAT.end();
         enableCellPower(false);
      }
  }

  bool checkForMovement(){
      bool movement = false;

        int xCurVal,yCurVal,zCurVal=0;
      for(int i=0;i<100;i++){
         xCurVal = analogRead(X_PIN);
         yCurVal = analogRead(Y_PIN);
         zCurVal = analogRead(Z_PIN);

        int xDiff=  abs(xCurVal-xVal);
        int yDiff=  abs(yCurVal-yVal);
        int zDiff=  abs(zCurVal-zVal);
        if (xDiff>mvmtThreshhold ||
            yDiff>mvmtThreshhold||
            zDiff>mvmtThreshhold){
             
             movement=true;  
             MovementStartTime=millis();
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

