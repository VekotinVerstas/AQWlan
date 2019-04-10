/**************************************************************************************
   This sketch reads SDS011 dust sensor and bme280/680  for humidity and temperature.

   The mesurement data is sent to an MQTT broker to be saved into a database
   or shown in some visualizations.
   Copyright 2018-2019 Aapo Rista / Vekotinverstas / Forum Virium Helsinki Oy
   MIT license

  NOTE
  You must install libraries below using Arduino IDE's 
  Sketch --> Include Library --> Manage Libraries... command

   PubSubClient (version >= 2.6.0 by Nick O'Leary)
   ArduinoJson (version > 5.13 < 6.0 by Benoit Blanchon)
   WiFiManager (version >= 0.14.0 by tzapu)
   Adafruit Unified Sensor (version >= 1.0.2 by Adafruit)
   Adafruit BME280 Library
   Adafruit BME680 Library
   Nova Fitness Sds dust sensors library
   
 **************************************************************************************/

#include "settings.h"             // Remember to copy settings-example.h to settings.h and check all values!
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>            // Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     // Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager WiFi Configuration Magic

// Sensor support libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BME680.h>
#include <SdsDustSensor.h>

// I2C settings
#define SDA     D2
#define SCL     D1

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  // Serial.println(payload);
}

// Define and set up all variables / objects
WiFiClient wifiClient;
WiFiManager wifiManager;
PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);
String mac_str;
unsigned long lastMqttMsgTime;
unsigned long mqttConnRetries = 0;

/* Sensor variables */

uint32_t status_lastRead = 0;
uint32_t status_lastSend = 0;

// Buttons (digital HIGH / LOW)
// PUSHBUTTON_1 and PUSHBUTTON_1 in settings.h
uint32_t pushButton1_lastRead = 0;
uint32_t pushButton1_lastSend = 0;
float pushButton1_lastState = 0;
float pushButton2_lastState = 0;

// BME680 AQ sensor
Adafruit_BME680 bme680;
uint8_t bme680_ok = 0;
uint32_t bme680_lastRead = 0;
uint32_t bme680_lastSend = 0;
float bme680_lastTemp = -999;
float bme680_lastHumi = -999;
float bme680_lastPres = -999;
float bme680_lastGas = -999;

// BME280 sensor
Adafruit_BME280 bme280;
uint8_t bme280_ok = 0;
uint32_t bme280_lastRead = 0;
uint32_t bme280_lastSend = 0;
float bme280_lastHumi = -999;
float bme280_lastTemp = -999;
float bme280_lastPres = -999;

// SDS011 PM sensor
// SDS011 Software serial settings
// TODO: Explicitly configure if is SDS011 present.
SdsDustSensor sds011(SDS011_RXPIN, SDS011_TXPIN);
uint8_t sds011_ok = 0;
uint32_t sds011_lastRead = 0;
uint32_t sds011_lastSend = 0;
float sds011_lastPM25 = -1.0;
float sds011_lastPM10 = -1.0;

float round_float(float val, int dec) {
  // Return val rounded to dec decimals
  return (int)(val * pow(10,dec) + 0.5) / 1.0 / pow(10,dec);
}

float abs_diff(float a, float b) {
  float c = a - b;
  if (c < 0) {c = -c;}
  return c;
}

float log10_diff(float a, float b) {
  float log_a = log10(a);
  float log_b = log10(b);
  return abs_diff(log_a, log_b);
}

void MqttSetup() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No WiFi so no MQTT. Continuing.");
    return;
  }
  // Generate client name based on MAC address and last 8 bits of microsecond counter
  String clientName;
  clientName += "esp8266-";
  clientName += mac_str;
  clientName += "-";
  clientName += String(micros() & 0xff, 16);

  Serial.print("Connecting to ");
  Serial.print(MQTT_SERVER);
  Serial.print(" as ");
  Serial.println(clientName);

  if (client.connect((char*) clientName.c_str(), MQTT_USER, MQTT_PASSWORD)) {
    
    Serial.println("Connected to MQTT broker");
    Serial.print("Topic is: ");
    Serial.println(MQTT_TOPIC);
    SendStartupToMQTT("version", "0.2.1");
  }
  else {
    Serial.println("MQTT connect failed");
    Serial.println("Will reset and try again...");
    // TODO: quit connecting after e.g. 20 seconds to enable standalone usage
    // abort();
  }
}

void setup() {
  mac_str = WiFi.macAddress();
  Wire.begin(SDA, SCL);
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  lastMqttMsgTime = millis();
  init_sensors();
  char ap_name[30];
  sprintf(ap_name, "Quasimodo_%d", ESP.getChipId());
  Serial.print("AP name would be: ");
  Serial.println(ap_name);
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.autoConnect(ap_name);
  MqttSetup();
}

void loop() {
  if (millis() < lastMqttMsgTime) {
    Serial.println("millis() rollover - reboot");
    ESP.restart();
  }
  if (!client.loop()) {
    Serial.println("Client disconnected...");
    // TODO: increase reconnect from every loop() to every 60 sec or so
    mqttConnRetries++;
    if (mqttConnRetries > 10) {
      Serial.println("ESP.restart() after 30 seconds");
      delay(30000);
      ESP.restart();
    }
    MqttSetup();
    return;
  }
  read_sensors();
  mqttConnRetries = 0;
}

void init_sensors() {
  // init_pushButton();
  init_bme280();
  init_bme680();
  init_sds011();
}

void read_sensors() {
  // read_pushButton();
  read_status();
  read_bme280();
  read_bme680();  
  read_sds011();
}

void read_status() {
  if (
      (millis() > (status_lastRead + STATUS_SEND_DELAY)) || 
      (status_lastSend == 0) 
     ) {
    status_lastRead = millis();
    status_lastSend = millis();
    long rssi = WiFi.RSSI();
    long uptime = millis();
    SendDataToMQTT("status", 
        "rssi", rssi,
        "uptime", uptime,
        "", 0,
        "", 0,
        -1
    );
  }
}

void init_pushButton() {
  Serial.println(F("INIT Pushbutton "));
  pinMode(PUSHBUTTON_1, INPUT);
  pinMode(PUSHBUTTON_2, INPUT);
}

void read_pushButton() {
  // Read PUSHBUTTON if it has been initialised successfully and it is time to read it
  //Serial.println("read_pushButton()");
  if ((millis() > (pushButton1_lastRead + PUSHBUTTON_SEND_DELAY))) {
    pushButton1_lastRead = millis();
    int buttonState1 = digitalRead(PUSHBUTTON_1);
    int buttonState2 = digitalRead(PUSHBUTTON_2);
    /*
    Serial.print("but 1 & 2: ");
    Serial.print(buttonState1);
    Serial.print(" ");
    Serial.println(buttonState2);
    */
    // Send data only when it has changed enough or it is time to send it anyway    
    if (
        (millis() > (pushButton1_lastSend + SENSOR_SEND_MAX_DELAY)) || 
        (pushButton1_lastState != buttonState1 ||
         pushButton2_lastState != buttonState2)
    ) {
      pushButton1_lastSend = millis();
      SendDataToMQTT("button", 
        "b1", buttonState1,
        "b2", buttonState2,
        "", 0,
        "", 0,
        -1
      );
      pushButton1_lastRead = millis();
    }
    pushButton1_lastState = buttonState1;
    pushButton2_lastState = buttonState2;
  }
}

void init_bme280() {
  Serial.print(F("INIT BME280: "));
  if (bme280.begin(0x76)) {
    Serial.println(F("found"));
    bme280_ok = 1;
  } else {
    Serial.println(F("not found"));
  }
}

void read_bme280() {
  // Read BME280 if it has been initialised successfully and it is time to read it
  if (
      (bme280_ok == 1) && (millis() > (bme280_lastRead + BME280_SEND_DELAY)) or (millis() < bme280_lastRead) ) {
    bme280_lastRead = millis();
    float humi = bme280.readHumidity();
    float temp = bme280.readTemperature();
    float pres = bme280.readPressure() / 100.0F;
    // Send data only when it has changed enough or it is time to send it anyway    
    if (
        (millis() > (bme280_lastSend + SENSOR_SEND_MAX_DELAY)) ||
        (abs_diff(bme280_lastTemp, temp) > 0.2) ||
        (abs_diff(bme280_lastHumi, humi) > 1.0) ||
        (abs_diff(bme280_lastPres, pres) > 0.2) ||
        (millis() < bme280_lastSend) // or millis() overflow
        
    ) {
      bme280_lastSend = millis();
      SendDataToMQTT("bme280", 
        "temp", round_float(temp, 2),
        "humi", round_float(humi, 1),
        "pres", round_float(pres, 2),
        "", 0,
        -1
      );
      bme280_lastSend = millis();
      bme280_lastTemp = temp;
      bme280_lastHumi = humi;
      bme280_lastPres = pres;
    }
  }
}

void init_bme680() {
  Serial.print(F("INIT BME680: "));
  if (bme680.begin()) {
    Serial.println(F("found"));
    // Set up oversampling and filter initialization
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, BME680_HEATING_TIME); // 320*C for 150 ms, from settings.h
    bme680_ok = 1;
  } else {
    Serial.println(F("not found"));
  }
}

void read_bme680() {
  // Read BME680 if it has been initialised successfully and it is time to read it
  if ((bme680_ok == 1) && (millis() > (bme680_lastRead + BME680_SEND_DELAY))) {
    if (! bme680.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    } else {
      bme680_lastRead = millis();
      float temp = bme680.temperature;
      float humi = bme680.humidity;
      float pres = bme680.pressure / 100.0F;
      float gas = bme680.gas_resistance / 1000.0F;
      if (
          ((bme680_lastSend + SENSOR_SEND_MAX_DELAY) < millis()) ||
          (abs_diff(bme680_lastTemp, temp) > 0.2) ||
          (abs_diff(bme680_lastHumi, humi) > 1.0) ||
          (abs_diff(bme680_lastPres, pres) > 0.2) ||
          (abs_diff(bme680_lastGas, gas) > 3.0)
      ) {
        SendDataToMQTT("bme680",
          "temp", round_float(temp, 2),
          "humi", round_float(humi, 1),
          "pres", round_float(pres, 2),
          "gas", round_float(gas, 1),
          -1
        );
        bme680_lastSend = millis();
        bme680_lastTemp = temp;
        bme680_lastHumi = humi;
        bme680_lastPres = pres;
        bme680_lastGas = gas;
      }
    }
  }
}

void init_sds011() {
  Serial.print(F("INIT sds011: "));
  sds011.begin();
  delay(1500); // Wait shortly to make sure SDS is responsive
  String undef = String("Mode: undefined");
  Serial.println(undef);
  if (undef == sds011.setContinuousWorkingPeriod().toString()) {
    Serial.println(F("not found"));
  } else {
    Serial.println(sds011.queryFirmwareVersion().toString()); // prints firmware version
    Serial.println(sds011.setActiveReportingMode().toString()); // ensures sensor is in 'active' reporting mode
    Serial.println(sds011.setContinuousWorkingPeriod().toString()); // ensures sensor has continuous working period - default but not recommended
    sds011_ok = 1;
  }
}

void read_sds011() {
  if ((sds011_ok == 1) && (millis() > (sds011_lastRead + SDS011_SEND_DELAY))) {
    sds011_lastRead = millis();
    PmResult pm = sds011.readPm();
    if (pm.isOk()) {
      float pm25 = pm.pm25;
      float pm10 = pm.pm10;
      Serial.print("PM2.5 = ");
      Serial.print(pm25);
      Serial.print(", PM10 = ");
      Serial.println(pm10);
      if (
          ((sds011_lastSend + SENSOR_SEND_MAX_DELAY) < millis()) ||
          (abs_diff(sds011_lastPM25, pm25) > 0.3) ||
          (abs_diff(sds011_lastPM10, pm10) > 0.3) ||
          (sds011_lastSend == 0)
      ) {    
        SendDataToMQTT("sds011",
          "pm25", pm25,
          "pm10", pm10,
          "", 0,
          "", 0,
          -1
        );
        sds011_lastSend = millis();
        sds011_lastPM25 = pm25;
        sds011_lastPM10 = pm10;
      }
    }
  }
}


void SendDataToMQTT(char const sensor[], 
                    char const type1[], float val1, 
                    char const type2[], float val2, 
                    char const type3[], float val3, 
                    char const type4[], float val4,
                    int16_t sn) {
  /**
   * Send data to the MQTT broker. Currently max 4 key/value pairs are supported. 
   * If you set typeX argument empty (""), if will be left out from the payload.
   */
  /* 
   *  NOTE!
   *  MQTT topic + json message to be send can't exceed ~121 bytes
   *  unless MQTT_MAX_PACKET_SIZE is set to 256
   *  Check that message size + topic are at most 120 B.
   */
  // Serial.println("SendDataToMQTT start");
  // StaticJsonBuffer<512> jsonBuffer;
  uint16_t msg_len = 0;
  DynamicJsonBuffer jsonBuffer(512);
  char jsonChar[256];
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = sensor;
  root["mac"] = mac_str;
  if (sn >= 0) {
    root["sn"] = sn;
  }
  JsonObject& data = root.createNestedObject("data");
  if (type1[0] != 0) { data[type1] = val1; }
  if (type2[0] != 0) { data[type2] = val2; }
  if (type3[0] != 0) { data[type3] = val3; }
  if (type4[0] != 0) { data[type4] = val4; }
  root.printTo(jsonChar);
  msg_len = strlen(MQTT_TOPIC) + strlen(jsonChar);
  Serial.print(round_float((millis() / 1000.0), 2));
  Serial.print("s ");
  Serial.print(msg_len);
  Serial.print("B ");
  Serial.print(MQTT_TOPIC);
  Serial.print(" ");
  Serial.println(jsonChar);
  if (msg_len > 120) {
    Serial.println("Warning: TOPIC + JSON > 120 bytes.");
  }
  if (client.publish(MQTT_TOPIC, jsonChar)) {
    lastMqttMsgTime = millis();
  } else {
    Serial.println("Error: Publishing MQTT message failed.");
  }
}

void SendStartupToMQTT(char const key1[], char const val1[]) {
  /**
   * Send startup message to the MQTT broker.
   */
  uint16_t msg_len = 0;
  DynamicJsonBuffer jsonBuffer(512);
  char jsonChar[256];
  JsonObject& root = jsonBuffer.createObject();
  root[key1] = val1;
  root["mac"] = mac_str; 
  root["rssi"] = WiFi.RSSI();
  root.printTo(jsonChar);
  msg_len = strlen(MQTT_TOPIC) + strlen(jsonChar);
  Serial.print(round_float((millis() / 1000.0), 2));
  Serial.print("s ");
  Serial.print(msg_len);
  Serial.print("B ");
  Serial.print(MQTT_TOPIC);
  Serial.print(" ");
  Serial.println(jsonChar);
  if (msg_len > 120) {
    Serial.println("Warning: TOPIC + JSON > 120 bytes.");
  }
  if (client.publish(MQTT_TOPIC, jsonChar)) {
    lastMqttMsgTime = millis();
  } else {
    Serial.println("Error: Publishing MQTT message failed.");
  }
}
