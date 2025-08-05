#include <WiFi.h>
#include "ArduinoJson.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

typedef struct{
  float temp;
  float hum;
  float pres;
  int gas;
}EnvData;

EnvData data;

// WiFi settings
const char* ssid = "TP-LINK_3AA6";
const char* password = "17934244";

// sending settings
unsigned long lastSend = 0;
const unsigned long sendInterval = 3000; 

// buzzer time settings
unsigned long buzzerStartTime = 0;
bool buzzerActive = false;

// Adafruit IO settings
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "ssenanb"
#define AIO_KEY         "aio_ZbZx01gMy5l89jqZoNvWv5GDfb8m"

// pin settings
#define RX_PIN 16
#define TX_PIN 17
#define BUZZER_PIN 2

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Feed definitions
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidityFeed    = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish pressureFeed    = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");
Adafruit_MQTT_Publish gasFeed         = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gas");
Adafruit_MQTT_Publish ambienceFeed    = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/ambience");

void connectToWiFi() {
  Serial.print("WiFi is connecting...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi succesfully connected.");
}

void connectToMQTT() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.println("MQTT connection...");
  while ((ret = mqtt.connect()) != 0) {
    Serial.print("MQTT connection error (");
    Serial.print(ret);
    Serial.print("): ");
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Try again after 3 seconds...");
    delay(3000);
  }

  Serial.println("MQTT succesfully connected.");
}

String evaluate_environment(){
    if (data.temp > 30 && data.hum < 50 &&  data.gas < 50 ){
         if (!buzzerActive) {
          digitalWrite(BUZZER_PIN, HIGH);
          buzzerActive = true;
          buzzerStartTime = millis();
        }
        return "Air is hot, dry, and polluted. Immediate action is advised.";
    }
    else if (data.temp < 15 && data.gas < 50)
        return "Cold and polluted air. Avoid prolonged exposure.";
    else if(data.pres < 90000 && data.gas < 50)
        return "Low pressure and poor air quality. Ventilation is necessary.";
    else if (data.temp > 30 && data.hum < 50)
        return "Hot and dry conditions. Stay hydrated.";
    else if (data.temp < 15) 
        return "Cold environment. Consider heating.";
    else
        return "The conditions are acceptable for living.";
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // UART2

  pinMode(BUZZER_PIN, OUTPUT);
  
  connectToWiFi();
  connectToMQTT();
}

void loop() {
  if (!mqtt.connected()) {
    connectToMQTT();
  }

  mqtt.processPackets(10);
  mqtt.ping();

  if (millis() - lastSend > sendInterval) {
      lastSend = millis();

      String jsonStr = Serial2.readStringUntil('\n');
      Serial.println("Incoming JSON: " + jsonStr);

      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, jsonStr);

      if (error) {
        Serial.print("JSON Parse Error: ");
        Serial.println(error.c_str());
        return;
      }

    data.temp = doc["Temperature"];
    data.hum  = doc["Humidity"];
    data.pres = doc["Pressure"];
    data.gas  = doc["Gas"];

    Serial.printf("Temperature: %.2f, Humidity: %.2f, Pressure: %.2f, Gas: %d\n", data.temp, data.hum, data.pres, data.gas);

      temperatureFeed.publish(data.temp);
      humidityFeed.publish(data.hum);
      pressureFeed.publish(data.pres);
      gasFeed.publish((uint32_t)data.gas);
      ambienceFeed.publish(evaluate_environment().c_str());
  }

  // Buzzer time control (close after 3 seconds)
  if (buzzerActive && (millis() - buzzerStartTime >= 3000)) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
  }

}
