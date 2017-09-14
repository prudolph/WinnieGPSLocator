
#define TINY_GSM_MODEM_SIM808
#define PIN_TX 3
#define PIN_RX 2
#define KEY_PIN 8
#define PWR_PIN 7
#define WAKE_PIN A0

#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#include "LowPower.h"
#include "Config.h"

// Your GPRS credentials
const char apn[]  = "hologram";

bool needGPSFix = false; 
unsigned long IntervalTime = 40000;
unsigned long StartTime;

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

Position lastGPSPos,lastCELLPos;

void setup(){

    //Set Pin Modes
      pinMode(KEY_PIN, OUTPUT);  
      pinMode(PWR_PIN, INPUT);  
      pinMode(WAKE_PIN, INPUT);  
      digitalWrite(KEY_PIN, HIGH);

      
  // Set console baud rate
  Serial.begin(115200);
  delay(10);
  wakeUp();
  
}

void loop(){

  
  if(needGPSFix){
      lastGPSPos=getGPSPosition();
      lastCELLPos=getCellPosition();
      sendLocationData();

   }

    unsigned long CurrentTime = millis();
    unsigned long ElapsedTime = CurrentTime - StartTime;
     if(ElapsedTime>IntervalTime){
      lowPowerMode();
    }
     Serial.println(ElapsedTime);
 
}


void enableCellPower(bool state){      
    int pwrState = digitalRead(PWR_PIN);
    if( pwrState!=state ){ 
         Serial.print("Setting Power:");
         Serial.println(state);
 
        digitalWrite(KEY_PIN, LOW);
        delay(2000);
        digitalWrite(KEY_PIN, HIGH);
        delay(5000);
    }
}

void connectToModem(){
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
    //postData(dataString);
  } 


/*
 void enableGPS(bool state){
  needGPSFix=state;
  if(state)modem.enableGPS();
  else modem.disableGPS();
 }
 */


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
   
    StartTime = millis();
   
    enableCellPower(true);
    connectToModem();
    connectToNetwork();
 }

void lowPowerMode(){
    Serial.println("Power Down");
     modem.disableGPS();
     modem.gprsDisconnect();
     SerialAT.end();
     enableCellPower(false);
     
    delay(5000);
    // Triggers an infinite sleep (the device will be woken up only by the registered wakeup sources)
    // The power consumption of the chip will drop consistently


      LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF); 
  }
  

