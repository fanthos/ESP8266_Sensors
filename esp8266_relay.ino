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

#define RELAY_GPIO D2

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

void MQTT_callback(String &topic, String &data) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(data);
  if (data == "ON") {
    switch_status |= 0x01;
    Serial.println("Turing on");
  } else if (data == "OFF") {
    switch_status &= ~0x01;
    Serial.println("Turing off");
  }
}

void MQTT_connect() {
  char sub_feed[70];
  strcpy(sub_feed, config->mqtt_feed);
  strcat(sub_feed, "/set");

  Serial.print("\nconnecting...");
  while (!mqtt.connect(dev_name, config->mqtt_user, config->mqtt_pass)) {
    Serial.print(".");
    delay(1000);
  }
  mqtt.subscribe(sub_feed);
  Serial.println(sub_feed);
}

void MQTT_setup() {
  uint16_t port = ((uint8_t)config->mqtt_port[1] << 8) | (uint8_t)config->mqtt_port[0];
  
  mqtt.begin(config->mqtt_host, port, wifi);
  mqtt.onMessage(MQTT_callback);

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

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_GPIO, OUTPUT);
  CONF_setup();
  WiFi_setup();
  OTA_setup();
  MQTT_setup();
}

void RELAY_update() {
  if (switch_status & 0x01) {
    digitalWrite(RELAY_GPIO, 0);
  } else {
    digitalWrite(RELAY_GPIO, 1);
  }
}

void loop() {
  ArduinoOTA.handle();
  mqtt.loop();
  delay(10);
  if (!mqtt.connected()) {
    MQTT_connect();
  }
  RELAY_update();
}
