#define DHT_DEBUG
extern "C"{
#include <user_interface.h>
}

#include <math.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <EEPROM.h>

#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#define DIFF_TOLERANCE    0.12f
#define DIFF_UPDATE       0.08f
#define DIFF_UPDATE_TIME  300000000L
#define DIFF_SENSOR_TIME  2500000L

#define DHTPIN            D4

#define DHTTYPE           DHT12

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

#define CONFIG_SIZE 512

uint32_t delaySec;
typedef union{
  char data[CONFIG_SIZE];
  struct{
    char ssid[32];
    char pass[64];
    char mqtt_host[32];
    char mqtt_user[32];
    char mqtt_pass[32];
    char mqtt_feed[64];
    char mqtt_port[2];
  };
} config_t;

config_t * config;
// "192.168.201.2"
// 0x5b, 0x07
WiFiClient client;

Adafruit_MQTT_Client *mqtt;
Adafruit_MQTT_Publish *pub;


void save_config() {
  //EEPROM.getDataPtr();
  //EEPROM.commit();
}

void MQTT_setup() {
  uint16_t port = ((uint8_t)config->mqtt_port[1] << 8) | (uint8_t)config->mqtt_port[0];
  mqtt = new Adafruit_MQTT_Client(&client, config->mqtt_host, port,
    config->mqtt_user, config->mqtt_pass);
  pub = new Adafruit_MQTT_Publish(mqtt, config->mqtt_feed);
}

int8_t MQTT_connect() {
  int8_t ret;

  if (!mqtt->connected()) {
    Serial.print("Connecting to MQTT... ");

    uint8_t retries = 3;
    while ((ret = mqtt->connect()) != 0) { // connect will return 0 for connected
      Serial.println(mqtt->connectErrorString(ret));
      mqtt->disconnect();
      delay(500);
      retries--;
      if (retries == 0) {
        Serial.print("Failed to connect to MQTT... ");
        return 0;
      }
    }
  }
  return 1;
}

void WiFi_setup() {
  struct station_config wifi_cfg;
  WiFi.mode(WIFI_STA);
  delay(500);

  if (wifi_station_get_config(&wifi_cfg)) {
    wifi_cfg.bssid_set = 0;
    wifi_station_set_config_current(&wifi_cfg);
  }

  Serial.println(WiFi.SSID());
  if (WiFi.begin() != WL_CONNECTED) {
    WiFi.beginSmartConfig();
    while(WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      if(WiFi.smartConfigDone()){
        Serial.println("WiFi Smart Config Done.");
      }
    }
    if (!WiFi.getAutoConnect()) {
      WiFi.setAutoConnect(true);
      Serial.println("Set AutoConnect.");
    }
    if (wifi_station_get_config(&wifi_cfg)) {
      wifi_cfg.bssid_set = 0;
      wifi_station_set_config_current(&wifi_cfg);
    }
  }

  Serial.print("WiFi: ");
  Serial.println(WiFi.SSID());
}

void CONF_setup() {
  EEPROM.begin(512);
  config = (config_t *)EEPROM.getDataPtr();
  if (config->data[CONFIG_SIZE-1] != 0x55) {
    config->data[CONFIG_SIZE-1] = 0x55;
    strcpy(config->mqtt_host, "192.168.201.2");
    strcpy(config->mqtt_user, "hass1");
    strcpy(config->mqtt_pass, "hasstest");
    sprintf(config->mqtt_feed, "home/switch/esp_%06x", ESP.getChipId());
    config->mqtt_port[0] = 0x5b;
    config->mqtt_port[1] = 0x07;
    EEPROM.commit();
    Serial.println("EEPROM init data written.");
  }
}

void OTA_setup() {
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start updating");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void MQTT_publish(Adafruit_MQTT_Publish *pub, int value) {

}

void DHT_update() {
  static float last_temp = -99;
  static float last_temp_calc = -99;
  static uint32_t last_pub_micros = 0;
  static uint32_t last_micros = 0;
  float temp;

  uint32_t curr_micros = micros();
  if(curr_micros - last_micros < DIFF_SENSOR_TIME) {
    return;
  }
  last_micros = curr_micros;

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  temp = event.temperature;
  if (!isnan(temp)) {
    Serial.print("Temp: ");
    Serial.println(temp);
    if (fabsf(last_temp - temp) > DIFF_TOLERANCE) {
      last_temp = temp;
    } else {
      float temp_calc = (last_temp + temp) * 0.5;
      if (fabsf(temp_calc - last_temp_calc) > DIFF_UPDATE
      || curr_micros - last_pub_micros > DIFF_UPDATE_TIME) {
        if (MQTT_connect()) {
          Serial.print("Pub: ");
          Serial.print(last_temp_calc);
          Serial.print(" -> ");
          Serial.println(temp_calc);
          pub->publish(temp_calc);
          last_temp_calc = temp_calc;
          last_pub_micros = curr_micros;
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  CONF_setup();
  WiFi_setup();
  OTA_setup();
  MQTT_setup();
  
  dht.begin();
}

void loop() {
  ArduinoOTA.handle();
  DHT_update();
}
