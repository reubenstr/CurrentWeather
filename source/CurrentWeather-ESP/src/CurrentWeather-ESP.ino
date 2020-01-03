// Weather acquired from openweathermap.org
// Current weather API examples: https://openweathermap.org/current

// WiFiManager
// https://github.com/tzapu/WiFiManager/tree/master/examples/AutoConnectWithFSParameters

#include <FS.h>
#include <math.h>
#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager

// The version of WiFiManger used in this project requires ArduinoJason version 5
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

const char accessPointName[] = "Current Weather";

// User pass parameters set during WiFi setup/login via captured portal.
char zip[6] = "22553";
char key[34] = "8d04433fa71622e1e2b9a8467f6b81e1";

WiFiClient client;


void setup()
{  
  Serial.begin(115200);
  Serial1.begin(115200);

  Serial.println();

  BeginSpiffs();
  
  ConnectToWifi();
 
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
}


void loop()
{  
  Serial.print("WiFi status: ");
  Serial.println(WiFi.status());
  Serial.print("WiFi RSSI: ");
  Serial.println(WiFi.RSSI());

  GetWeather();
  
  delay(2000);  
}


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

    Serial.print("Weather: ");
    Serial.println(weatherId);
    Serial.print("Temperature: ");
    Serial.println(temp);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    Serial.print("Pressure: ");
    Serial.println(pressure);

    // Pack data into custom serial format.
    char data[10];
    int sum = weatherId + temp + humidity + pressure;
    data[0] = weatherId & 0xFF;
    data[1] = weatherId >> 8;
    data[2] = temp & 0xFF;
    data[3] = temp >> 8;
    data[4] = humidity & 0xFF;
    data[5] = humidity >> 8;
    data[6] = pressure & 0xFF;
    data[7] = pressure >> 8;
    data[8] = sum & 0xFF;
    data[9] = sum >> 8;
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

  if (SPIFFS.begin())
  {
    Serial.println("mounted file system");
    
    if (SPIFFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("Reading config file...");
      File configFile = SPIFFS.open("/config.json", "r");
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
  }
}


void saveConfigCallback()
{
  Serial.println("Saving parameters...");
    
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    json["zip"] = zip;
    json["key"] = key;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      Serial.println("Failed to open config file for writing.");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
}



void ConnectToWifi()
{
Serial.println("Attempting to connect to WiFi...");

  WiFiManager wifiManager;

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter custom_zip("zip", "Zip Code", zip, 6);
  WiFiManagerParameter custom_key("key", "API Key", key, 33);

  wifiManager.addParameter(&custom_zip);
  wifiManager.addParameter(&custom_key);

  if (!wifiManager.autoConnect("accessPointName")) 
  {
    Serial.println("Failed to connect to WiFi and hit timeout.");
    delay(1000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  strcpy(zip, custom_zip.getValue());
  strcpy(key, custom_key.getValue());

  Serial.println("Connected to WiFi...");
}


// Blocks
void OnDemandPortal()
{
  Serial.println("Starting On Demand Portal...");

  WiFiManager wifiManager;

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter custom_zip("zip", "Zip Code", zip, 6);
  WiFiManagerParameter custom_key("key", "API Key", key, 33);

  wifiManager.addParameter(&custom_zip);
  wifiManager.addParameter(&custom_key);

  if (!wifiManager.startConfigPortal(accessPointName))
  {
    Serial.println("Failed to connect and hit timeout.");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  strcpy(zip, custom_zip.getValue());
  strcpy(key, custom_key.getValue());

  Serial.println("Connected to WiFi...");
}

