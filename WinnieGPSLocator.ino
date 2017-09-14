
#define TINY_GSM_MODEM_SIM808
#define PIN_TX 3
#define PIN_RX 2
#define KEY_PIN 8
#define PWR_PIN 7

#include <TinyGsmClient.h>
#include <SoftwareSerial.h>



// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "hologram";
const char user[] = "";
const char pass[] = "";
char http_cmd[] = "{\"k\": \"a1Djc>)t\",\"d\": \"hello from tcp test\",\"t\": [\"location\"]}";

const char server[] = "cloudsocket.hologram.io";
const int port = 9999;

char buffer[512];


SoftwareSerial SerialAT(PIN_TX,PIN_RX ); // TX, RX
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);


//signitures
void enableCellPower(bool state);
void connectToNetwork();
void setup(){
    
    //Set Pin Modes
      pinMode(KEY_PIN, OUTPUT);  
      pinMode(PWR_PIN, INPUT);  
      digitalWrite(KEY_PIN, HIGH);

      
  // Set console baud rate
  Serial.begin(115200);
  delay(10);

  enableCellPower(true);
  connectToNetwork();


  
}

void loop(){

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


void connectToNetwork(){
  
  
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

  Serial.print(F("Waiting for network..."));
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
