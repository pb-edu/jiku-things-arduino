#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#define MAX_LEN 200
#define ESP_RESET 5
#define SENSING_INTERVAL 600000

bool debug = true;
int sensorInterval = 5000;

unsigned long timeGone = 0;
unsigned long lastSensorMessageSent;
bool espReady = false;
bool configurationSent = false;

char incomingSerial[200];
StaticJsonDocument<200> incomingJson;
StaticJsonDocument<500> outgoingJson;

SoftwareSerial espSerial(2, 3);  // rx, tx

void setup() {
  Serial.begin(9600);  
  pinMode(ESP_RESET, OUTPUT);
  digitalWrite(ESP_RESET, LOW);
  delay(2000);
  digitalWrite(ESP_RESET, HIGH);
  delay(2000);
  espSerial.begin(9600);
}

void loop() {
  if(listenForIncomingMessage()){
    processIncomingMsg();
  }
  if(espReady) {
    processOutgoingMsg();
  }
}

void processOutgoingMsg() {
  timeGone = millis();
  if(espReady && timeGone >= (lastSensorMessageSent + sensorInterval)){
    lastSensorMessageSent = timeGone;
    gatherSensorData();
    sendData();
  }
}

void gatherSensorData(){
  outgoingJson.clear();
  outgoingJson["status"] = 200;
  outgoingJson["time"] = millis();
  outgoingJson["temperature"] = 26;
}

bool listenForIncomingMessage() {  
  uint8_t count = 0;
  while (espSerial.available() > 0) {
    char c = espSerial.read(); 
    delay(100);
    switch (c) {
      case '\r':
        break;
      case '\n':
        incomingSerial[count] = '\0';
        serialDebug("received: " + String(incomingSerial));
        count = 0;
        return true;
      default:
        if (count < (MAX_LEN - 1)) {
          incomingSerial[count++] = c;
        }
        break;
    }
  }
}

void processIncomingMsg() {
  if(strcmp(incomingSerial, "INIT_ESP") == 0){
    clearIncomingSerial();
    espReady = false;
    initializeEsp();
    return;
  }
  if(strcmp(incomingSerial, "WIFI_READY") == 0){
    clearIncomingSerial();
    espReady = true;
    return;
  }
  if(!deserializeIncomingMsg()) return;
  serialDebug(incomingJson["command"]);
}

void clearIncomingSerial(){
  incomingSerial[0] = '\0';
}

bool deserializeIncomingMsg() {
  if(incomingSerial[0] != '{') {
    return false;
  }
  
  serialDebug("deserializing:");
  serialDebug(incomingSerial);
  
  DeserializationError err = deserializeJson(incomingJson, incomingSerial);
  if (err) {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.print(err.f_str());
    return false;
  }
  return true;
}

void sendData(){
  serializeJson(outgoingJson, espSerial);
  espSerial.print("\n"); 
  serialDebug("Sending JSON:");
  if(debug) { 
    serializeJson(outgoingJson, Serial);
    Serial.print("\n"); 
  }
}

void serialDebug(String msg) {
  if(debug) {
    Serial.println(msg);
    delay(10);
  }
}

void initializeEsp(){
  outgoingJson["ssid"] = "";
  outgoingJson["password"] = "";
  outgoingJson["broker"] = ""; 
  outgoingJson["port"] = "1883";
  outgoingJson["client"] = "Gruppe1";
  sendData();
  outgoingJson.clear();
}
