
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
char http_cmd[] = "{\"k\": \"<CRED>\",\"d\": \"hello from tcp test\",\"t\": [\"location\"]}";

const char server[] = "cloudsocket.hologram.io";
const int port = 9999;

bool needGPSFix = false; 

SoftwareSerial SerialAT(PIN_TX,PIN_RX ); // TX, RX
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

typedef struct{ String lat,lon; int spd;} Position;


//signitures
void enableCellPower(bool state);
void connectToModem();
void connectToNetwork();
void disconnectModem();

void enableGPS(bool state);
Position getGPSPosition();
Position getCellPosition();


void setup(){

    //Set Pin Modes
      pinMode(KEY_PIN, OUTPUT);  
      pinMode(PWR_PIN, INPUT);  
      digitalWrite(KEY_PIN, HIGH);

      
  // Set console baud rate
  Serial.begin(115200);
  delay(10);

  Serial.print(F("Using Credentials: "));
  Serial.println(CSR_Cred);
  
  enableCellPower(true);
  connectToModem();
  enableGPS(true);

  connectToNetwork();
  
 // disconnectModem();

  
}

void loop(){
  delay(5000);
  
  if(needGPSFix){
      getGPSPosition();
      getCellPosition();
   }
}


void enableCellPower(bool state){
  Serial.print(F("Attempting to Set Power State to: "));
  Serial.println(state);
       
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
  Serial.println(F("Initializing modem..."));
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print(F("Modem: "));
  Serial.println(modemInfo);
}


void connectToNetwork(){
 
  Serial.print(F("Connecting to network..."));
  if (!modem.waitForNetwork()) {
    Serial.println(F(" fail"));
    while (true);
  }
  Serial.println(F(" OK"));

  Serial.print(F("Connecting to "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(F(" fail"));
    while (true);
  }
  Serial.println(F(" OK"));

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
    Serial.println(F(" Getting GPS Position"));

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
     pos.spd = spd;
     
      Serial.println(lat);
      Serial.println(lon);
      Serial.println(spd);
      Serial.println(alt);
      Serial.println(viewd_sats);
      Serial.println(used_sats);

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

  for (int i =0;i<5;i++){
  
    Serial.println(results[i]);
   
  }
  
     Position pos;

    return pos;
  }

