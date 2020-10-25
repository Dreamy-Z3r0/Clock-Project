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

uint8_t CredentialsResetPin = 2;
bool buttonState = HIGH;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

bool uartACTIVE = false;

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
  return String();
}

void setup() 
{
  Serial.begin(115200);

  pinMode(CredentialsResetPin, INPUT_PULLUP);

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

  HTML_TimeInput.NewDataSet = true;
  HTML_CalendarInput.NewDataSet = true;

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

  server.on("/ClockCalendar", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
    String inputMessage;
    String MESSAGE;
    if (request->hasParam(PARAM_INT_TIME)) 
    {
      inputMessage = request->getParam(PARAM_INT_TIME)->value();

      while (!HTML_TimeInput.NewDataSet);
      
      HTML_TimeInput.MinuteData = (inputMessage[4] - '0') + 10*(inputMessage[3] - '0');
      HTML_TimeInput.HourData = (inputMessage[1] - '0') + 10*(inputMessage[0] - '0');

      HTML_TimeInput.NewDataSet = false;
      MESSAGE = "New time set!";
    }
    else if (request->hasParam(PARAM_INT_DATE)) 
    {
      inputMessage = request->getParam(PARAM_INT_DATE)->value();

      while (!HTML_CalendarInput.NewDataSet);

      HTML_CalendarInput.DateData = (inputMessage[9] - '0') + 10*(inputMessage[8] - '0');
      HTML_CalendarInput.MonthData = (inputMessage[6] - '0') + 10*(inputMessage[5] - '0');

      HTML_CalendarInput.YearData  =       inputMessage[3] - '0';
      HTML_CalendarInput.YearData +=   10*(inputMessage[2] - '0');
      HTML_CalendarInput.YearData +=  100*(inputMessage[1] - '0');
      HTML_CalendarInput.YearData += 1000*(inputMessage[0] - '0');

      HTML_CalendarInput.NewDataSet = false;
      MESSAGE = "New date set!";
    }
    else
    {
      MESSAGE = "No input for new time or date.";
    }

    request->send(200, "text/text", MESSAGE);
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

  if (!HTML_TimeInput.NewDataSet & !uartACTIVE)
  {
    uartACTIVE = true;
    UART_Data_Transfer ('T');
    
    HTML_TimeInput.NewDataSet = true;
    uartACTIVE = false;
  }

  if (!HTML_CalendarInput.NewDataSet)
  {
    uartACTIVE = true;
    UART_Data_Transfer ('D');
    
    HTML_CalendarInput.NewDataSet = true; 
    uartACTIVE = false;
  }

  buttonState = digitalRead(CredentialsResetPin);
  if (LOW == buttonState) REBOOT();
}

void REBOOT (void)
{
  do
  {
    buttonState = digitalRead(CredentialsResetPin);
  } while (LOW == buttonState);
  
  ESP8266_AP.NEW_AP_SSID = "Clock Config (192.168.4.1)";
  ESP8266_AP.NEW_AP_PASSWORD = "12345678";
  
  ESP8266_AP.NewAPSSID = true;
  ESP8266_AP.NewAPPassword = true;
  ESP8266_AP.NewDataSet = false;
}

void LoadAPCredentials (void)
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

uint8_t StringToNumber (String inputString, uint8_t inputStringLength)
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

void UART_Data_Transfer (uint8_t dataCommand)
{
  uint8_t receivedMessage = 'r';
  uint8_t dataBuffer[5] = {dataCommand, 0, 0, 0, 0};
  uint8_t resendRequest[] = {'r', 0, 0, 0, 0};
  
  if ('T' == dataCommand)
  {
    dataBuffer[1] = HTML_TimeInput.HourData;
    dataBuffer[2] = HTML_TimeInput.MinuteData;
  }
  else if ('D' == dataCommand)
  {
    dataBuffer[1] = HTML_CalendarInput.DateData;
    dataBuffer[2] = HTML_CalendarInput.MonthData;
    dataBuffer[3] = (uint8_t)((HTML_CalendarInput.YearData & 0xFF00) >> 8);
    dataBuffer[4] = (uint8_t)(HTML_CalendarInput.YearData & 0x00FF);
  }
   
  do
  {
    if ('r' == receivedMessage)
    {
      for(int index = 0; index < 5; index++)
      {
        Serial.write(dataBuffer[index]);
      }
    }
    else if ('O' != receivedMessage) 
    {
      for(int index = 0; index < 5; index++)
      {
        Serial.write(resendRequest[index]);
      }
    }
    
    while (!Serial.available());
    receivedMessage = Serial.read();
  } while ('O' != receivedMessage);
}
