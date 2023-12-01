#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <ESP8266Firebase.h>
#include <ESP8266WiFi.h>

#define _SSID "iPhone"          // WiFi SSID
#define _PASSWORD "ijphone15"      // WiFi Password

// #define _SSID "Bahadir"          // WiFi SSID
// #define _PASSWORD "bahodir270779"      // WiFi Password
#define REFERENCE_URL "iot-cw-00011581-6458a-default-rtdb.europe-west1.firebasedatabase.app"  // Real time db url

// Creating objects
SoftwareSerial arduinoSerial(D2, D3);   // RT, TX pins on ESP8266
Firebase firebase(REFERENCE_URL);  // Initializing the firebase real time db


// Variables for sensors
float temperature;
float humidity;
int resistor;
int moisturePercentage;

// Variables for activators
int waterPumpStatus;
int servoMotorStatus;
int roofMotorStatus;

 
void setup() {
  Serial.begin(115200);  // for debugging (Serial Monitor)
  arduinoSerial.begin(115200);  // Serial to communication with NodeMCU
  while (!Serial) continue;  // Wait till Serial is available

  // Wifi: set mode, disconnect if connected before
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);

  // Connect to WiFi
  WiFi.begin(_SSID, _PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("-");
  }
  Serial.println("");
  Serial.println("WiFi Connected");
}

void loop() {
  if (arduinoSerial.available() > 0){   // If data sent from arduino is available
    receiveSensorDataFromArduino();
    setFirebaseSensorVariables();
  }
  sendActivatorDataToArduino();
}


// Functions to interact with firebase
void setFirebaseSensorVariables() {
  firebase.setFloat("Humidity", humidity);
  firebase.setInt("PhotoResistor", resistor);
  firebase.setFloat("Temperature", temperature);
  firebase.setInt("soilMoisture", moisturePercentage);
}

void getFirebaseActivatorVariables() {
  waterPumpStatus = firebase.getInt("waterPumpStatus");
  servoMotorStatus = firebase.getInt("servoMotorStatus");
  roofMotorStatus = firebase.getInt("roofMotorStatus");
}


// Functions for data transfer
void receiveSensorDataFromArduino() {
  String jsonData = arduinoSerial.readString();  // Reading from Arduino Uno

  Serial.print("Received from Arduino ");
  Serial.println(jsonData);

  StaticJsonDocument<200> doc;  // Creating a static json document
  DeserializationError error = deserializeJson(doc, jsonData);
  
  if (error){
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Setting values of sensors to corresponding variables
  temperature = doc["temp"];
  humidity = doc["hum"];
  resistor = doc["res"];
  moisturePercentage = doc["mois"];
}

void sendActivatorDataToArduino(){
  getFirebaseActivatorVariables();

  Serial.print(" waterPumpStatus ");
  Serial.print(waterPumpStatus);
  Serial.print(" servoMotorStatus ");
  Serial.print(servoMotorStatus);
  Serial.print(" roofMotorStatus ");
  Serial.println(roofMotorStatus);

  arduinoSerial.println(waterPumpStatus);
  arduinoSerial.println(servoMotorStatus);
  arduinoSerial.println(roofMotorStatus);
}
