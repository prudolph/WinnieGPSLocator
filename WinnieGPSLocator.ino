
#define TINY_GSM_MODEM_SIM808
#define PIN_TX 3
#define PIN_RX 2
#define KEY_PIN 8
#define PWR_PIN 7

#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include "Config.h"



// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "hologram";
const char user[] = "";
const char pass[] = "";

const char server[] = "cloudsocket.hologram.io";
const int port = 9999;

bool needGPSFix = false; 

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
void enableGPS(bool state);
Position getGPSPosition();
Position getCellPosition();



Position lastGPSPos,lastCELLPos;

void setup(){

    //Set Pin Modes
      pinMode(KEY_PIN, OUTPUT);  
      pinMode(PWR_PIN, INPUT);  
      digitalWrite(KEY_PIN, HIGH);

      
  // Set console baud rate
  Serial.begin(115200);
  delay(10);


  
  enableCellPower(true);
  connectToModem();
  enableGPS(true);

  connectToNetwork();
  
 // disconnectModem();


  
}

void loop(){

  
  if(needGPSFix){
      lastGPSPos=getGPSPosition();
      lastCELLPos=getCellPosition();
   }


  sendLocationData();

  delay(10000);


  delay(60000);
}


void enableCellPower(bool state){
    int pwrState = digitalRead(PWR_PIN);
    if( pwrState!=state ){ 
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
  //Serial.print(F("Modem: "));
  //Serial.println(modemInfo);
}


void connectToNetwork(){
 
  Serial.print(F("Connecting to network..."));
  if (!modem.waitForNetwork()) {
    Serial.println(F(" fail"));
    while (true);
  }


  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(F(" fail"));
    while (true);
  }
  Serial.println(F(" OK"));
  IPAddress localIPAddress = modem.localIP();
  
  //Serial.print(F(" IP Address:"));
  Serial.println(localIPAddress);
  
  


  }

void postData(String dataStr){
  
 //Serial.println(F("Connecting to "));
 // Serial.print(server);
  if (!client.connect(server, port)) {
   // Serial.println(" fail");
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
  Serial.println();
  client.stop();
  Serial.println(F("Server disconnected"));
  
  }

void sendLocationData(){
    Serial.println(F("Sending Location Data"));
    String payloadData = "{";
    payloadData+="\"k\": \""+ CSR_Cred  +"\",";
    payloadData+= "\"t\": [ \"loc\" ],";
    payloadData+="\"d\": {";
    payloadData+="\"Glt\":\""+String(lastGPSPos.lat)+"\",\"Gln\":\""+String(lastGPSPos.lon)+"\",";
    payloadData+="\"Clt\":\""+String(lastCELLPos.lat) +"\",\"Cln\":\""+String(lastCELLPos.lon)+"\"}";
    payloadData+= "}";
    Serial.println(payloadData);
    postData(payloadData);
  } 
void disconnectModem(){
    SerialAT.end();
    modem.gprsDisconnect();
  }


 void enableGPS(bool state){
  needGPSFix=state;
  if(state)modem.enableGPS();
  else modem.disableGPS();
 }

 
Position getGPSPosition(){
    
    float lat,  lon, spd;
    int alt, viewd_sats,  used_sats;
      
    
      bool fix= modem.getGPS(&lat, &lon, &spd, &alt, &viewd_sats, &used_sats) ;
      if(!fix){
        Serial.println(F("Could Not get Satelite FIX...Retry"));        
      }else{
        enableGPS(false);
      }
  
     Position pos;
     pos.lat = lat;
     pos.lon = lon;

    return pos;
  }
  
Position getCellPosition(){
  Serial.println(F(" Getting CELL Position"));
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



