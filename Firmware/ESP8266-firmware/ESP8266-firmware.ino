// Import required libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "HTML_INDEX.h"

// Local server credentials
String server_ssid;
String server_password;

// Update restart
bool NewAPSSID = false;
bool NewAPPassword = false;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

const char* PARAM_STRING_AP_SSID = "AP SSID";
const char* PARAM_STRING_AP_PASSWORD = "AP Password";
const char* PARAM_VOLUME = "JQ6500 Volume";

bool TEST_SOUND_REQUEST = false;
bool CANCEL_REQUEST = false;

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String readFile(fs::FS &fs, const char * path){
  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){
    return String();
  }
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  File file = fs.open(path, "w");
  if(!file)
  {
    //Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message))
  {
    //Serial.println("- file written");
  } else {
    //Serial.println("- write failed");
  }
}

// Replaces placeholder with stored values
String processor(const String& var)
{
  if (var == "AP SSID")
  {
    return server_ssid;
  }
  else if (var == "AP Password")
  {
    return server_password;
  }
  else if (var == "JQ6500 Volume")
  {
    return readFile(SPIFFS, "/JQ6500_Volume.txt");
  }
  return String();
}

void setup() 
{
  Serial.begin(115200);

  if(!SPIFFS.begin())
  {
      return;
  }

  LoadAPCredentials();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(server_ssid, server_password);

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?WiFi SSID=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) 
  {
    String inputMessage;
    bool updateReady = false;
    // GET AP SSID value on <ESP_IP>/get?AP SSID=<inputMessage>
    if (request->hasParam(PARAM_STRING_AP_SSID)) {
      inputMessage = request->getParam(PARAM_STRING_AP_SSID)->value();
      if (String() != inputMessage)
      {
        inputMessage += String(" (192.168.4.1)");
        NewAPSSID = true;
        writeFile(SPIFFS, "/ESP_AP_SSID.txt", inputMessage.c_str());
        LoadAPCredentials();
      }
    }
    // GET AP Password value on <ESP_IP>/get?AP Password=<inputMessage>
    else if (request->hasParam(PARAM_STRING_AP_PASSWORD)) {
      inputMessage = request->getParam(PARAM_STRING_AP_PASSWORD)->value();
      if (String() != inputMessage)
      {
        NewAPPassword = true;
        writeFile(SPIFFS, "/ESP_AP_PASSWORD.txt", inputMessage.c_str());
        LoadAPCredentials();
      }
    }
    // GET JQ6500 Volume value on <ESP_IP>/get?JQ6500 Volume=<inputMessage>
    else if (request->hasParam(PARAM_VOLUME)) {
      inputMessage = request->getParam(PARAM_VOLUME)->value();
      if (String() != inputMessage)
        writeFile(SPIFFS, "/JQ6500_Volume.txt", inputMessage.c_str());
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/text", inputMessage);
  });
  
  server.on("/ForceUpdate", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    NewAPSSID = true;
    NewAPPassword = true;
    request->send(200, "text/text", "ESP restarting...");
  });

  server.on("/play", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    TEST_SOUND_REQUEST = true;
    CANCEL_REQUEST = false;
    request->send(200, "text/text", "Test sound is on.");
  });
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    CANCEL_REQUEST = true;
    TEST_SOUND_REQUEST = false;
    request->send(200, "text/text", "Sound test is cancelled.");
  });
  server.onNotFound(notFound);
  server.begin();
}

void loop() 
{
  if (NewAPSSID && NewAPPassword) ESP.restart();
}

void LoadAPCredentials(void)
{
  String LoadedSSID = readFile(SPIFFS, "/ESP_AP_SSID.txt");
  String LoadedPassword = readFile(SPIFFS, "/ESP_AP_PASSWORD.txt");
  
  if (String() != LoadedSSID) server_ssid = LoadedSSID;
  else server_ssid = "Clock Config: 192.168.4.1";

  if (String() != LoadedPassword) server_password = LoadedPassword;
  else server_password = "123456789";
}
