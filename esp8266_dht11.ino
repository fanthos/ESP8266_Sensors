#define DHT_DEBUG
extern "C"{
#include <user_interface.h>
}

#include <math.h>

#include <EEPROM.h>

#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>

#include <MQTTClient.h>

#define MQTT_WAIT_TIME_MS 2000
#define MQTT_PING_INTERVAL_US 300000000

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DIFF_UPDATE_TIME  300000000L
#define DIFF_SENSOR_TIME  5000000L

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
WiFiClient wifi;

MQTTClient mqtt;

uint8_t switch_status = 0;
char dev_name[8]={0};

void save_config() {
  //EEPROM.getDataPtr();
  //EEPROM.commit();
}

void MQTT_connect() {
  // char sub_feed[70];
  // strcpy(sub_feed, config->mqtt_feed);
  // strcat(sub_feed, "/set");

  int max_retry = 10;

  Serial.print("\nconnecting...");
  while (!mqtt.connect(dev_name, config->mqtt_user, config->mqtt_pass) && max_retry--) {
    Serial.print(".");
    delay(1000);
  }
  // mqtt.subscribe(sub_feed);
  // Serial.println(sub_feed);
}

void MQTT_setup() {
  uint16_t port = ((uint8_t)config->mqtt_port[1] << 8) | (uint8_t)config->mqtt_port[0];
  
  mqtt.begin(config->mqtt_host, port, wifi);
  // mqtt.onMessage(MQTT_callback);

  mqtt.setOptions(60, true, 1000);
  
  MQTT_connect();
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
  Serial.println(WiFi.localIP());
}

void CONF_setup() {
  EEPROM.begin(512);
  config = (config_t *)EEPROM.getDataPtr();
  if (config->data[CONFIG_SIZE-1] != 0x55) {
    config->data[CONFIG_SIZE-1] = 0x55;
    strcpy(config->mqtt_host, "192.168.201.2");
    strcpy(config->mqtt_user, "hass1");
    strcpy(config->mqtt_pass, "hasstest");
    sprintf(config->mqtt_feed, "home/smart/esp_%06x", ESP.getChipId());
    config->mqtt_port[0] = 0x5b;
    config->mqtt_port[1] = 0x07;
    EEPROM.commit();
    Serial.println("EEPROM init data written.");
  }
  sprintf(dev_name, "%06x", ESP.getChipId());
  Serial.println(config->mqtt_host);
  Serial.println(config->mqtt_feed);
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

int16_t DHT_read() {
  sensors_event_t event;
  float temp;
  dht.temperature().getEvent(&event);
  temp = event.temperature * 10;
  if (isnan(temp)) {
    return -99;
  } else if (temp >= 0) {
    return (int16_t)(temp + 0.5);
  } else {
    return (int16_t)(temp - 0.5);
  }
}

void DHT_update() {
  static int16_t last_temp = -99;
  static int16_t last_temp_update = -99;
  static uint32_t last_pub_micros = 0;
  static uint32_t last_micros = 0;
  static uint8_t sync_counter = 0;
  int16_t itemp;
  char print_buf[10];


  uint32_t curr_micros = micros();
  if(curr_micros - last_micros < DIFF_SENSOR_TIME) {
    return;
  }
  last_micros = curr_micros;

  itemp = DHT_read();
  if (itemp > -50) {
    Serial.print("Temp: ");
    Serial.println(itemp);
    if (itemp != last_temp) {
      last_temp = itemp;
      sync_counter = 0;
    } else {
      if (sync_counter >= 2) {
        if (!mqtt.connected()) {
          MQTT_connect();
        }
        if (!mqtt.connected()) {
          return;
        }
        if (sync_counter == 3) {
          if (curr_micros - last_pub_micros < DIFF_UPDATE_TIME) {
            return;
          }
        }
        sync_counter = 3;

        sprintf(print_buf, "%d.%d", itemp/10, itemp%10);

        Serial.print("Pub: ");
        Serial.print(last_temp_update);
        Serial.print(" -> ");
        Serial.println(print_buf);
        mqtt.publish(config->mqtt_feed, print_buf, true, 0);
        last_temp_update = itemp;
        last_pub_micros = curr_micros;
      } else {
        sync_counter ++;
      }
    }
  } else {
    Serial.println("Temp: Fail");
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
  mqtt.loop();
  delay(10);
  if (!mqtt.connected()) {
    MQTT_connect();
  }
}
