#include <Arduino.h>
#include <Wire.h>
#include <hp_BH1750.h>
#include <math.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <EEPROM.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
// TODO: implement using 24c08 eeprom
#include <EEPROM_24C08_SWI2C.h>

#define D0 16
#define D1 5 // I2C Bus SCL (clock)
#define D2 4 // I2C Bus SDA (data)
#define D3 0
#define D4 2  // Same as "LED_BUILTIN", but inverted logic
#define D5 14 // SPI Bus SCK (clock)
#define D6 12 // SPI Bus MISO
#define D7 13 // SPI Bus MOSI
#define D8 15 // SPI Bus SS (CS)
#define D9 3  // RX0 (Serial console)
#define D10 1 // TX0 (Serial console)
#define MOISTURE_SAMPLE_DELAY 1000
#define DATE_SEND_DELAY 60000

#define MAX_MQTT_HOST_LEN 40
#define MAX_MQTT_PORT_LEN 6

#define LUX_TOPIC "planter2.0/light"
#define MOISTURE_TOPIC "planter2.0/moisture"
#define WATER_DATE_TOPIC "planter2.0/waterdate"
#define REQUEST_WATER_DATE_TOPIC "planter2.0/requestwaterdate"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST "broker.hivemq.com"
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

#define EEPROM_LAST_WATERED_ADDRESS MAX_MQTT_HOST_LEN + MAX_MQTT_PORT_LEN + 1

// #define DEBUG

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiUDP ntpUDP;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);
// You can specify the time server pool and the offset, (in seconds)
// additionally you can specify the update interval (in milliseconds).
// NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);


void connectToMqtt();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttPublish(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);


int soilMoisture = A0;

int waterPump = D6;
int growLights = D5;
long lastMilis = 0;
int btnStartConfigPortal = D7;
int btnSetLastWatered = D8;

char mqtt_host[MAX_MQTT_HOST_LEN] = "broker.hivemq.com";
char mqtt_port[MAX_MQTT_PORT_LEN] = "1883";

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

hp_BH1750 sensor;
void setup() {
  Serial.begin(9600);

  // ================================================================
  // setup pins
  pinMode(waterPump, OUTPUT);
  pinMode(growLights, OUTPUT);
  pinMode(soilMoisture, INPUT);
  pinMode(btnStartConfigPortal, INPUT_PULLUP);
  pinMode(btnSetLastWatered, INPUT_PULLUP);
  digitalWrite(waterPump, HIGH);

  // ================================================================
  // setup Lux sensor
  sensor.begin(BH1750_TO_GROUND);
  sensor.calibrateTiming();
  sensor.start();

  // ================================================================
  // eeprom debug
  EEPROM.begin(512);
  for (int i = 0; i < 128; i++) {
    Serial.println(char(EEPROM.read(i)));
  }

  // ================================================================
  // Wifi manager extra paramters
  // TODO: check if paramters are already saved in eeprom if not setup AP
  WiFiManagerParameter custom_mqtt_host("server", "mqtt host", mqtt_host, MAX_MQTT_HOST_LEN);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_host, MAX_MQTT_PORT_LEN);


  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_mqtt_host);
  wifiManager.addParameter(&custom_mqtt_port);

  //first parameter is name of access point, second is the password
  while(!wifiManager.autoConnect()){
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  Serial.println("connected...yeey :)");

  // ================================================================
  // setup NTP client
  timeClient.begin();
  timeClient.update();


  // ================================================================
  // setup MQTT client
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  connectToMqtt();

  // ================================================================
  // save mqtt host and port in eeprom
  if (shouldSaveConfig) {
    Serial.println("saving config");
    strcpy(mqtt_host, custom_mqtt_host.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    EEPROM.put(0, mqtt_host);
    EEPROM.put(MAX_MQTT_HOST_LEN, mqtt_port);
    EEPROM.commit();
  }
}

void loop() {
  if (sensor.hasValue()) {
    float lux = sensor.getLux();
    if (!sensor.saturated()) {
      #ifdef DEBUG
      Serial.print("Light level: ");
      Serial.println(lux);
      #endif
      mqttClient.publish(LUX_TOPIC, 0, false, String(lux).c_str());
    }
    sensor.adjustSettings(90);
    sensor.start();
    if (lux < 100) {
      int lightDimness = map(roundf(lux), 0, 100, 0, 255);
      analogWrite(growLights, lightDimness);
    } else {
      digitalWrite(growLights, HIGH);
    }
  }
  
  if (millis() - lastMilis > MOISTURE_SAMPLE_DELAY){
    int dryness = analogRead(soilMoisture);
    #ifdef DEBUG
    Serial.print("Soil Moisture: ");
    Serial.println(dryness);
    #endif
    mqttClient.publish(MOISTURE_TOPIC, 0, false, String(dryness).c_str());
    lastMilis = millis();
  }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  
  mqttClient.subscribe(REQUEST_WATER_DATE_TOPIC, 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT. Attempting to reconnect...");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  timeClient.update();
  mqttClient.publish(WATER_DATE_TOPIC, 0, false, ("Pong" + String(timeClient.getEpochTime())).c_str());
}

void waterPlant() {
  digitalWrite(waterPump, LOW);
  delay(5000);
  digitalWrite(waterPump, HIGH);
  // save in eeprom last watered date
  timeClient.update();
  EEPROM.put(EEPROM_LAST_WATERED_ADDRESS, timeClient.getEpochTime());
  EEPROM.commit();
}

void sendLastWateredDate() {
  long lastWateredDate;
  EEPROM.get(EEPROM_LAST_WATERED_ADDRESS, lastWateredDate);
  mqttClient.publish(WATER_DATE_TOPIC, 0, false, String(lastWateredDate).c_str());
}
