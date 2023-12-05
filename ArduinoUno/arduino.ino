#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <dht.h>
#include <LCD_I2C.h>
#include <Servo.h>
#include <math.h>

// PIN values for sensors
int DHT11_PIN = 12;
int PHOTORES_PIN = A2;
int SOIL_MOISTURE_PIN = A1;

// PIN values for actuators
int IS_ACTIVE = 1;
int SERVO_PIN = 11;
int ROOF_MOTOR_PIN = 9;
int WATER_PUMP_PIN = 8;

// Declaring objects
dht DHT;
LCD_I2C lcd(0x27, 16, 2);
Servo Servo1;
SoftwareSerial espSerial(2, 3);  // RT, TX pins on Arduino Uno

// Variables for sensors values
float temperature;
float humidity;
int resistor;
int moisturePercentage;
StaticJsonDocument<200> doc;  // Creating a static json document

// Variables for activators values
int waterPumpStatus;
int servoMotorStatus;
int roofMotorStatus;


void setup() {
  // Serial Monitor (for debugging)
  Serial.begin(9600);

  // Sensors pins
  pinMode(DHT11_PIN, INPUT);
  pinMode(PHOTORES_PIN, INPUT);
  pinMode(SOIL_MOISTURE_PIN, INPUT);

  // Activators pins
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(ROOF_MOTOR_PIN, OUTPUT);
  pinMode(WATER_PUMP_PIN, OUTPUT);

  lcd.begin();  // Beginning LCD
  lcd.backlight();  // Activating backlight of LCD
  Servo1.attach(SERVO_PIN);  // Attaching SERVO_PIN to Servo object
  espSerial.begin(115200);   // Serial to communication with NodeMCU
  while (!Serial) continue;  // Wait till Serial is available
}

void loop() {
  int chk = DHT.read11(DHT11_PIN);  // reading data from DHT11 through DHT11_PIN
  // Function calls related to sensors
  readTempHumSensor();
  readPhotoresistorSensor();
  readSoilMoistureSensor();
  sendSensorData2NodeMCU();
  displaySensorsDataInLCD();

  if (espSerial.available() > 0) {
    // Function calls related to activators
    receiveActivatorDataFromNodeMCU();
    rotateServoMotor();
    startWaterPump();
    rotateRoofMotor();
  }
}


// Functions for sensors
void readTempHumSensor() {
  temperature = DHT.temperature;
  humidity = DHT.humidity;
}

void readPhotoresistorSensor() {
  resistor = analogRead(PHOTORES_PIN);
}

void readSoilMoistureSensor() {
  int sensorAnalog = analogRead(SOIL_MOISTURE_PIN);
  moisturePercentage = round(( 100 - ( (sensorAnalog / 1023.00) * 100) ));
}


// Functions for activators
void rotateServoMotor() {
  if (servoMotorStatus == IS_ACTIVE) {
    Serial.println("ROTATED SERVO MOTOR");
    Servo1.write(160);
    delay(1000);
  }
  else {
    Serial.println("STOPPED SERVO MOTOR");
    Servo1.write(-90);
  }
}

void startWaterPump() {
  if (waterPumpStatus == IS_ACTIVE) {
    Serial.println("STARTED WATER PUMP");
    digitalWrite(WATER_PUMP_PIN, HIGH);
  }
  else {
    digitalWrite(WATER_PUMP_PIN, LOW);
    Serial.println("STOPPED WATER PUMP");
  }
}

void rotateRoofMotor() {
  if (roofMotorStatus == IS_ACTIVE) {
    Serial.println("STARTED ROOF MOTOR");
    digitalWrite(ROOF_MOTOR_PIN, HIGH);
  }
  else {
    digitalWrite(ROOF_MOTOR_PIN, LOW);
    Serial.println("STOPPED ROOF MOTOR");
  }
}

void displaySensorsDataInLCD() {
  // Temperature and Humidity
  lcd.print("TEMPERATURE ");
  lcd.print(temperature);
  lcd.setCursor(0, 1);  // Setting the cursor to row 1 and column 0
  lcd.print("HUMIDITY ");
  lcd.print(humidity);
  lcd.print("  ");
  delay(5000);
  lcd.clear();

  // Photoresistor and Soil moisture
  lcd.print("RESISTOR ");
  lcd.print(resistor);
  lcd.setCursor(0, 1);  // Setting the cursor to row 1 and column 0
  lcd.print("MOISTURE ");
  lcd.print(moisturePercentage);
  lcd.print("% ");
  delay(5000);
  lcd.clear();
}


// Functions for data transfer
void sendSensorData2NodeMCU(){
  // Setting values of sensors
  doc["temp"] = temperature;
  doc["hum"] = humidity;
  doc["res"] = resistor;
  doc["mois"] = moisturePercentage;

  char jsonBuffer[200];  // Create string buffer
  serializeJson(doc, jsonBuffer);  // Serialize string to JSON doc

  Serial.print("Sending to NodeMCU ");
  Serial.println(jsonBuffer);
  espSerial.println(jsonBuffer);  // Send to nodemcu
}

void receiveActivatorDataFromNodeMCU(){
  String strWaterPumpStatus = espSerial.readStringUntil('\n');  // Reading from NodeMCU
  String strServoMotorStatus = espSerial.readStringUntil('\n');  // Reading from NodeMCU
  String stringRoofMotorStatus = espSerial.readStringUntil('\n');  // Reading from NodeMCU

  // Converting string "0" and "1" to int 0 and 1 
  waterPumpStatus = strWaterPumpStatus.toInt();
  servoMotorStatus = strServoMotorStatus.toInt();
  roofMotorStatus = stringRoofMotorStatus.toInt();

  Serial.println("Received from NodeMCU ");
  Serial.print("waterPumpStatus ");
  Serial.print(waterPumpStatus);
  Serial.print(" servoMotorStatus ");
  Serial.print(servoMotorStatus);
  Serial.print(" roofMotorStatus ");
  Serial.println(roofMotorStatus);
}
