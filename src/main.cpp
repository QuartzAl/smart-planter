#include <Arduino.h>
#include <AsyncMqttClient.h>
#include <EEPROM.h>
#include <EEPROM_24C08_SWI2C.h>
#include <NTPClient.h>
#include <Ticker.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <WiFiUdp.h>
#include <Wire.h>
#include <hp_BH1750.h>
#include <time.h>

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
#define MOISTURE_SAMPLE_DELAY 5000
#define PUMP_ON_TIME 5000

#define MAX_MQTT_HOST_LEN 40
#define MAX_MQTT_PORT_LEN 6
#define EEPROM_LAST_WATERED_ADDRESS MAX_MQTT_HOST_LEN + MAX_MQTT_PORT_LEN + 1
#define LUX_TOPIC "planter/light"
#define MOISTURE_TOPIC "planter/moisture"
#define REQUEST_TOPIC "planter/request"
#define REQUEST_WATER_DATE 'D'
#define REQUEST_WATER 'W'
#define REQUEST_LIGHTS_TOGGLE_OVERRIDE 'L'
#define REQUEST_LIGHTS_VALUE 'V'
#define RESPOND_TOPIC "planter/respond"
#define RESPOND_OVERRIDE_TOPIC "planter/respond/override"
#define RESPOND_DATE_TOPIC "planter/respond/date"
#define RESPOND_LIGHT_LEVEL_TOPIC "planter/respond/light"
#define AP_NAME "Smart Planter"

#define MQTT_HOST "192.168.1.150"
#define MQTT_PORT 1883
#define DEBUG_EEPROM

EEPROM_24C08_SWI2C eeprom(D2, D1, 0x50);
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
hp_BH1750 sensor;

void connectToMqtt();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttPublish(uint16_t packetId);
void onMqttMessage(char *topic, char *payload,
                   AsyncMqttClientMessageProperties properties, size_t len,
                   size_t index, size_t total);
void waterPlant();
void recordLastWateredDate();
unsigned long readLongFromEEPROM(int address);
void writeLongToEEPROM(int address, unsigned long value);

int soilMoisture = A0;
int waterPump = D6;
int growLights = D5;
long lastMilis = 0;
int btnStartConfigPortal = D0;
int btnSetLastWatered = D3;
bool lastbtnSetLastWateredState = false;
bool turnOnPump = false;
bool lightsOverride = false;
byte lightsOverrideValue = 0;
long pumpStartTime = 0;

char mqtt_host[MAX_MQTT_HOST_LEN] = "broker.hivemq.com";
char mqtt_port[MAX_MQTT_PORT_LEN] = "1883";

// flag for saving data
bool shouldSaveConfig = false;

// callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  eeprom.begin();

  // ================================================================
  // setup pins
  pinMode(waterPump, OUTPUT);
  pinMode(growLights, OUTPUT);
  pinMode(soilMoisture, INPUT);
  pinMode(btnStartConfigPortal, INPUT_PULLUP);
  pinMode(btnSetLastWatered, INPUT_PULLUP);
  digitalWrite(waterPump, LOW);

  // ================================================================
  // setup Lux sensor
  sensor.begin(BH1750_TO_GROUND);
  sensor.calibrateTiming();
  sensor.start();

  // ================================================================
  // setup NTP client
  timeClient.begin();
  timeClient.update();

  // ================================================================
  // setup MQTT client
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);

// ================================================================
// eeprom debug
#ifdef DEBUG_EEPROM
  Serial.println("eeprom debug");
  Serial.print("\tMQTT host: ");
  char readCharacter;
  for (int i = 0; i < MAX_MQTT_HOST_LEN; i++) {
    readCharacter = char(eeprom.read(i));
    if (readCharacter == 0)
      break;
    Serial.print(readCharacter);
  }
  Serial.println();
  Serial.print("\tMQTT port: ");
  for (int i = 0; i < MAX_MQTT_PORT_LEN; i++) {
    readCharacter = char(eeprom.read(MAX_MQTT_HOST_LEN + i));
    if (readCharacter == 0)
      break;
    Serial.print(readCharacter);
  }
  Serial.println();
  Serial.print("\tLast watered date: ");
  unsigned long lastWateredDate =
      readLongFromEEPROM(EEPROM_LAST_WATERED_ADDRESS);
  time_t lastWateredTime = lastWateredDate;
  struct tm ts;
  char buf[80];

  // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
  ts = *localtime(&lastWateredTime);
  strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
  Serial.print(buf);
  Serial.println();
#endif

  // ================================================================
  // Wifi manager extra paramters
  WiFiManagerParameter custom_mqtt_host("server", "mqtt host", mqtt_host,
                                        MAX_MQTT_HOST_LEN);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port,
                                        MAX_MQTT_PORT_LEN);
  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_mqtt_host);
  wifiManager.addParameter(&custom_mqtt_port);

  if (eeprom.read(0) != 0) {
    for (int i = 0; i < MAX_MQTT_HOST_LEN; i++) {
      Serial.println("Reading from eeprom: " + String(i) + ": " +
                     String(char(eeprom.read(i))));
      mqtt_host[i] = char(eeprom.read(i));
    }
    for (int i = 0; i < MAX_MQTT_PORT_LEN; i++) {
      mqtt_port[i] = char(eeprom.read(MAX_MQTT_HOST_LEN + i));
    }
  } else {
    Serial.println("No config found in eeprom");
    wifiManager.startConfigPortal(AP_NAME);
  }

  // first parameter is name of access point, second is the password
  while (!wifiManager.autoConnect(AP_NAME)) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    // reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  Serial.println("connected...yeey :)");

  char *ptr;
  uint16 mqtt_port_uint = strtoul(mqtt_port, &ptr, 10);
  Serial.print("connecting to mqtt with host: ");
  Serial.print(mqtt_host);
  Serial.print(" and port: ");
  Serial.println(mqtt_port_uint);
  mqttClient.setServer(mqtt_host, mqtt_port_uint);
  connectToMqtt();

  // ================================================================
  // save mqtt host and port in eeprom
  if (shouldSaveConfig) {
    Serial.println();
    Serial.println("saving config");
    strcpy(mqtt_host, custom_mqtt_host.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    Serial.println("mqtt host: " + String(mqtt_host));
    Serial.println("mqtt port: " + String(mqtt_port));
    for (int i = 0; i < MAX_MQTT_HOST_LEN; i++) {
      eeprom.write(i, mqtt_host[i]);
    }
    for (int i = 0; i < MAX_MQTT_PORT_LEN; i++) {
      eeprom.write(MAX_MQTT_HOST_LEN + i, mqtt_port[i]);
    }
    shouldSaveConfig = false;
  }
}

float lux;

void loop() {
  timeClient.update();
  if (sensor.hasValue()) {
    lux = sensor.getLux();
    if (!sensor.saturated()) {
#ifdef DEBUG
      Serial.print("Light level: ");
      Serial.println(lux);
#endif
    }
    sensor.adjustSettings(90);
    sensor.start();
    if (!lightsOverride) {
      if (lux < 100) {
        int lightDimness = map(roundf(lux), 0, 100, 255, 0);
        analogWrite(growLights, lightDimness);
      } else {
        digitalWrite(growLights, LOW);
      }
    } else {
      analogWrite(growLights, lightsOverrideValue);
    }
  }

  if (millis() - lastMilis > MOISTURE_SAMPLE_DELAY) {
    int dryness = analogRead(soilMoisture);
    mqttClient.publish(MOISTURE_TOPIC, 0, false, String(dryness).c_str());
    mqttClient.publish(LUX_TOPIC, 0, false, String(lux).c_str());
    lastMilis = millis();
  }

  if (digitalRead(btnStartConfigPortal) == LOW) {
    Serial.println("Start config portal");
    WiFiManager wifiManager;
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    WiFiManagerParameter custom_mqtt_host("server", "mqtt host", mqtt_host,
                                          MAX_MQTT_HOST_LEN);
    WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port,
                                          MAX_MQTT_PORT_LEN);
    wifiManager.addParameter(&custom_mqtt_host);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.startConfigPortal(AP_NAME);
    if (shouldSaveConfig) {
      Serial.println("saving config");
      strcpy(mqtt_host, custom_mqtt_host.getValue());
      strcpy(mqtt_port, custom_mqtt_port.getValue());
      for (int i = 0; i < MAX_MQTT_HOST_LEN; i++) {
        eeprom.write(i, mqtt_host[i]);
      }
      for (int i = 0; i < MAX_MQTT_PORT_LEN; i++) {
        eeprom.write(MAX_MQTT_HOST_LEN + i, mqtt_port[i]);
      }
      shouldSaveConfig = false;
    }
    ESP.reset();
  }

  if (digitalRead(btnSetLastWatered) == LOW &&
      lastbtnSetLastWateredState == false) {
    Serial.println("Set last watered date");
    recordLastWateredDate();
    mqttClient.publish(RESPOND_DATE_TOPIC, 0, false,
                       String(readLongFromEEPROM(EEPROM_LAST_WATERED_ADDRESS)).c_str());
    lastbtnSetLastWateredState = true;
  } else if (digitalRead(btnSetLastWatered) == HIGH &&
             lastbtnSetLastWateredState == true) {
    lastbtnSetLastWateredState = false;
  }

  if (turnOnPump) {
    pumpStartTime = millis();
    digitalWrite(waterPump, HIGH);
    turnOnPump = false;
  } else if (millis() - pumpStartTime > PUMP_ON_TIME) {
    digitalWrite(waterPump, LOW);
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

  mqttClient.subscribe(REQUEST_TOPIC, 0);
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

void onMqttMessage(char *topic, char *payload,
                   AsyncMqttClientMessageProperties properties, size_t len,
                   size_t index, size_t total) {

  Serial.println("Publish received.");
  Serial.print("  payload: ");
  Serial.println(payload);
  if (payload[0] == REQUEST_WATER_DATE) {
    unsigned long lastWateredDate =
        readLongFromEEPROM(EEPROM_LAST_WATERED_ADDRESS);
    mqttClient.publish(RESPOND_DATE_TOPIC, 0, false,
                       String(lastWateredDate).c_str());

  } else if (payload[0] == REQUEST_WATER) {
    waterPlant();
    mqttClient.publish(RESPOND_TOPIC, 0, false, String(REQUEST_WATER).c_str());

  } else if (payload[0] == REQUEST_LIGHTS_TOGGLE_OVERRIDE) {
    lightsOverride = !lightsOverride;
    if (lightsOverride)
      mqttClient.publish(RESPOND_OVERRIDE_TOPIC, 0, false, "1");
    else
      mqttClient.publish(RESPOND_OVERRIDE_TOPIC, 0, false, "0");

  } else if (payload[0] == REQUEST_LIGHTS_VALUE) {
    byte lightsOverridePercentage = atoi(payload + 1);
    if (lightsOverridePercentage > 100)
      lightsOverridePercentage = 100;
    else if (lightsOverridePercentage < 0)
      lightsOverridePercentage = 0;

    mqttClient.publish(RESPOND_LIGHT_LEVEL_TOPIC, 0, false,
                       String(lightsOverridePercentage).c_str());
    lightsOverrideValue = map(lightsOverridePercentage, 0, 100, 0, 255);
  }
}

void waterPlant() {
  turnOnPump = true;
  recordLastWateredDate();
}

void recordLastWateredDate() {
  unsigned long lastWateredDate = timeClient.getEpochTime();
  writeLongToEEPROM(EEPROM_LAST_WATERED_ADDRESS, lastWateredDate);
}

unsigned long readLongFromEEPROM(int address) {
  unsigned long value = 0;
  for (int i = 3; i >= 0; i--) {
    byte readByte = eeprom.read(address++);
    value |= readByte << (i * 8);
  }
  return value;
}

void writeLongToEEPROM(int address, unsigned long value) {
  for (int i = 3; i >= 0; i--) {
    byte savedByte = (value >> (i * 8)) & 0xFF;
    eeprom.write(address++, savedByte);
  }
}