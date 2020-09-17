/*
* Current Weather (WiFi)
*
* Gathers weather data from API vis WiFi
* Packs weather data and sends to main MCU via UART
*
* MCU: ESP8266-01
*
* Weather acquired from openweathermap.org
* API examples: https://openweathermap.org/current
*
* Reuben Strangelove
* Winter 2019
*
*/

#include <FS.h>
#include <math.h>
#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include<LittleFS.h>

#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
// https://github.com/tzapu/WiFiManager/tree/master/examples/AutoConnectWithFSParameters

// The version of WiFiManger used in this project requires ArduinoJason version 5
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#include "msTimer.h"

#define PIN_CAPTIVE_PORTAL_TRIGGER 0
const char accessPointName[] = "Current Weather";
const unsigned long delayBetweenApiCallsMs = 10000; // milliseconds

// User pass parameters set during WiFi setup/login via captured portal.
char zip[6] = "22902";
char key[34] = "8d04433fa71622e1e2b9a8467f6b81e1";
char brightness[4] = "127";

WiFiClient client;

bool GetWeather()
{
  String zipParam(zip);
  String keyParam(key);

  if (client.connect("api.openweathermap.org", 80))
  {
    Serial.println("Connecting to OpenWeatherMap server...");

    // send the HTTP PUT request:
    client.println("GET /data/2.5/weather?zip=" + zipParam + "&units=imperial&APPID=" + keyParam + " HTTP/1.1");
    client.println("Host: api.openweathermap.org");
    client.println("Connection: close");
    client.println();

    // Check HTTP status
    char status[32] = {0};
    client.readBytesUntil('\r', status, sizeof(status));
    // It should be "HTTP/1.0 200 OK" or "HTTP/1.1 200 OK"
    if (strcmp(status + 9, "200 OK") != 0)
    {
      Serial.print(F("Unexpected response: "));
      Serial.println(status);
      return false;
    }

    // Skip HTTP headers
    char endOfHeaders[] = "\r\n\r\n";
    if (!client.find(endOfHeaders))
    {
      Serial.println(F("Invalid response."));
      return false;
    }

    const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(1) + 2 * JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(4) + 2 * JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(13) + 270;
    DynamicJsonBuffer jsonBuffer(capacity);

    JsonObject &root = jsonBuffer.parseObject(client);
    if (!root.success())
    {
      Serial.println(F("Json parsing failed!"));
      return false;
    }

    int weatherId = root["weather"][0]["id"].as<int>();
    int temp = (int)round(root["main"]["temp"].as<float>());
    int humidity = root["main"]["humidity"].as<int>();
    int pressure = root["main"]["pressure"].as<int>();

    unsigned int brightnessValue;
    sscanf(brightness, "%u", &brightnessValue);

    Serial.print("Weather: ");
    Serial.println(weatherId);
    Serial.print("Temperature: ");
    Serial.println(temp);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    Serial.print("Pressure: ");
    Serial.println(pressure);
    Serial.print("Brightness: ");
    Serial.println(brightnessValue);

    // Pack data into custom serial format.
    char data[11];
    unsigned int sum = weatherId + temp + humidity + pressure + brightnessValue;
    data[0] = weatherId & 0xFF;
    data[1] = weatherId >> 8;
    data[2] = temp & 0xFF;
    data[3] = temp >> 8;
    data[4] = humidity & 0xFF;
    data[5] = humidity >> 8;
    data[6] = pressure & 0xFF;
    data[7] = pressure >> 8;                                         
    data[8] = brightnessValue;
    data[9] = sum & 0xFF;
    data[10] = sum >> 8;
    Serial1.write(data, sizeof(data));

    client.stop();

    return true;
  }

  Serial.println("Failed to connect to URL.");
  return false;
}

// Read configuration from FS json
void BeginSpiffs()
{
  Serial.println("Mounting FS...");

  if (LittleFS.begin())
  {
    Serial.println("Mounted file system.");

    if (LittleFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("Reading config file...");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("Opened config file.");

        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);

        if (json.success())
        {
          Serial.println("\nparsed json");
          strcpy(zip, json["zip"]);
          strcpy(key, json["key"]);
          strcpy(brightness, json["brightness"]);
        }
        else
        {
          Serial.println("Failed to load json (config).");
        }
        configFile.close();
      }
    }
  }
  else
  {
    Serial.println("Failed to mount FS.");
    LittleFS.format();
  }
}

void saveConfigCallback()
{
  Serial.println("Saving parameters...");

  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  json["zip"] = zip;
  json["key"] = key;
  json["brightness"] = brightness;

  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println("Failed to open config file for writing.");
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
}

// Blocks
void StartWifi(bool startAsCaptivePortal = false)
{
  if (startAsCaptivePortal)
    Serial.println("Starting On Demand Portal...");
  else
    Serial.println("Connecting to WiFi...");

  WiFiManager wifiManager;

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter custom_zip("zip", "Zip Code", zip, 6);
  WiFiManagerParameter custom_key("key", "API Key", key, 33);
  WiFiManagerParameter custom_brightness("brightness", "LED Brightness (0 - 255)", brightness, 4);

  wifiManager.addParameter(&custom_zip);
  wifiManager.addParameter(&custom_key);
  wifiManager.addParameter(&custom_brightness);

  if (startAsCaptivePortal)
  {
    if (!wifiManager.startConfigPortal(accessPointName))
    {
      Serial.println("Failed to connect and hit timeout.");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  }
  else
  {
    if (!wifiManager.autoConnect(accessPointName))
    {
      Serial.println("Failed to connect to WiFi and hit timeout.");
      delay(1000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  }

  strcpy(zip, custom_zip.getValue());
  strcpy(key, custom_key.getValue());
  strcpy(brightness, custom_brightness.getValue());

  Serial.println("Connected to WiFi.");
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println();

  pinMode(PIN_CAPTIVE_PORTAL_TRIGGER, INPUT_PULLUP);

  BeginSpiffs();

  StartWifi();
}

void loop()
{
  static msTimer timer(0);

  if (timer.elapsed())
  {
    timer.setDelay(delayBetweenApiCallsMs);

    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("WiFi RSSI: ");
    Serial.println(WiFi.RSSI());

    GetWeather();
  }

  // Check if button is pressed to start captive portal.
  if (digitalRead(PIN_CAPTIVE_PORTAL_TRIGGER) == 0)
  {
    StartWifi(true);
  }
}