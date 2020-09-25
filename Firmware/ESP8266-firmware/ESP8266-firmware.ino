// Import required libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "HTML_INDEX.h"
#include "Storage_and_Handlers.h"

NetworkCredentials ESP8266_AP;
TimeData HTML_TimeInput;
CalendarData HTML_CalendarInput;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

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
    return ESP8266_AP.CURRENT_AP_SSID;
  }
  else if (var == "New AP SSID")
  {
    return ESP8266_AP.NEW_AP_SSID;
  }
  else if (var == "AP Password")
  {
    return ESP8266_AP.CURRENT_AP_PASSWORD;
  }
  else if (var == "New AP Password")
  {
    return ESP8266_AP.NEW_AP_PASSWORD;
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
  ESP8266_AP.NEW_AP_SSID = "(none)";
  ESP8266_AP.NEW_AP_PASSWORD = "(none)";
  ESP8266_AP.NewAPSSID = false;
  ESP8266_AP.NewAPPassword = false;
  ESP8266_AP.NewDataSet = true;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ESP8266_AP.CURRENT_AP_SSID, ESP8266_AP.CURRENT_AP_PASSWORD);

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?WiFi SSID=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) 
  {
    String inputMessage;
    String MESSAGE;

    // GET AP SSID value on <ESP_IP>/get?AP SSID=<inputMessage>
    if (request->hasParam(PARAM_STRING_AP_SSID)) 
    {
      inputMessage = request->getParam(PARAM_STRING_AP_SSID)->value();
      if (String() != inputMessage)
      {
        ESP8266_AP.NEW_AP_SSID = inputMessage + String(" (192.168.4.1)");
        if (ESP8266_AP.CURRENT_AP_SSID != ESP8266_AP.NEW_AP_SSID)
        {
          ESP8266_AP.NewAPSSID = true;
          MESSAGE = "New SSID issued.";
        }
      }
      else
      {
        ESP8266_AP.NEW_AP_SSID = "(none)";
        ESP8266_AP.NewAPSSID = false;
        MESSAGE = "No change for SSID.";
      }
    }
    // GET AP Password value on <ESP_IP>/get?AP Password=<inputMessage>
    else if (request->hasParam(PARAM_STRING_AP_PASSWORD)) 
    {
      inputMessage = request->getParam(PARAM_STRING_AP_PASSWORD)->value();
      if (String() != inputMessage)
      {
        ESP8266_AP.NEW_AP_PASSWORD = inputMessage;
        if (ESP8266_AP.CURRENT_AP_SSID != ESP8266_AP.NEW_AP_PASSWORD)
        {
          ESP8266_AP.NewAPPassword = true;
          MESSAGE = "New password issued.";
        }
      }
      else
      {
        ESP8266_AP.NEW_AP_PASSWORD = "(none)";
        ESP8266_AP.NewAPSSID = false;
        MESSAGE = "No change for password.";
      }
    }
    // GET JQ6500 Volume value on <ESP_IP>/get?JQ6500 Volume=<inputMessage>
    else if (request->hasParam(PARAM_VOLUME)) 
    {
      inputMessage = request->getParam(PARAM_VOLUME)->value();
    
      if (String() != inputMessage)
      {
        uint8_t inputNumber = StringToNumber(inputMessage, inputMessage.length());
        if (('e' != inputNumber) & (inputNumber <= 30))
        {
          writeFile(SPIFFS, "/JQ6500_Volume.txt", inputMessage.c_str());
        }
        else
        {
          MESSAGE = "Invalid input numebr.";
        }
      }   
    }
    else 
    {
      MESSAGE = "No message sent";
    }
    request->send(200, "text/text", MESSAGE);
  });
  
  server.on("/UpdateRequest", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    if (ESP8266_AP.NewAPSSID || ESP8266_AP.NewAPPassword)
      ESP8266_AP.NewDataSet = false;

    request->send(200, "text/text", "Credentials updated. ESP restarting...");
  });
  server.on("/Discard", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    ESP8266_AP.NewAPSSID = false;
    ESP8266_AP.NewAPPassword = false;
    ESP8266_AP.NewDataSet = true;

    ESP8266_AP.NEW_AP_SSID = "(none)";
    ESP8266_AP.NEW_AP_PASSWORD = "(none)";

    request->send(200, "text/text", "New credentials discarded!");
  });

//  server.on("/Clock", HTTP_GET, [] (AsyncWebServerRequest *request)
//  {
//    
//  }

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
  if (!ESP8266_AP.NewDataSet) 
  {
    if (ESP8266_AP.NewAPSSID)
    {
      writeFile(SPIFFS, "/ESP_AP_SSID.txt", ESP8266_AP.NEW_AP_SSID.c_str());
    }
    if (ESP8266_AP.NewAPPassword)
    {
      writeFile(SPIFFS, "/ESP_AP_PASSWORD.txt", ESP8266_AP.NEW_AP_PASSWORD.c_str());
    }

    ESP.restart();
  }
}

void LoadAPCredentials(void)
{
  String LoadedSSID = readFile(SPIFFS, "/ESP_AP_SSID.txt");
  String LoadedPassword = readFile(SPIFFS, "/ESP_AP_PASSWORD.txt");
  
  if (String() != LoadedSSID) 
    ESP8266_AP.CURRENT_AP_SSID = LoadedSSID;
  else 
    ESP8266_AP.CURRENT_AP_SSID = "Clock Config: 192.168.4.1";

  if (String() != LoadedPassword) 
    ESP8266_AP.CURRENT_AP_PASSWORD = LoadedPassword;
  else 
    ESP8266_AP.CURRENT_AP_PASSWORD = "123456789";
}

uint8_t StringToNumber(String inputString, uint8_t inputStringLength)
{
  uint8_t outputNumber = 0;

  for (uint8_t index = 0; index < inputStringLength; index++)
  {
    if (('0' <= inputString[index]) & (inputString[index] <= '9'))
    {
      outputNumber = outputNumber*10 + (inputString[index] - '0');
    }
    else
    {
      outputNumber = 'e';
      break;
    }
  }

  return outputNumber;
}
