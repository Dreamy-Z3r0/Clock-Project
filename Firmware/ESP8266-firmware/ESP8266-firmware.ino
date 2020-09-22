// Import required libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Local server credentials
const char* server_ssid     = "Digital Clock: 192.168.4.1";
const char* server_password = "123456789";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

const char* PARAM_STRING_SSID = "WiFi SSID";
const char* PARAM_STRING_PASSWORD = "WiFi Password";
const char* PARAM_VOLUME = "JQ6500 Volume";

bool TEST_SOUND_REQUEST = false;
bool CANCEL_REQUEST = false;

// HTML web page to handle 3 input fields
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script></head><body>
  <form action="/get" target="hidden-form">
    WiFi Credentials:<br>
      Current WiFi SSID: %WiFi SSID%<br>
      
      New SSID:       
        <input type="text" name="WiFi SSID">
        <input type="submit" value="Submit" onclick="submitMessage()">
  </form>
  <form action="/get" target="hidden-form">
      New Password:
        <input type="text" name="WiFi Password">
        <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br><br>

    Speaker:   
    <a href="/play" target="hidden-form"><button class="button">Play test sound</button></a>     
    <a href="/stop" target="hidden-form"><button class="button">Stop test sound</button></a>
    <br>
  <form action="/get" target="hidden-form">
    Current volume: %JQ6500 Volume%<br>
    Set to (0 - 30): <input type="number" name="JQ6500 Volume">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form>
  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

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
  if(!file){
    //Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    //Serial.println("- file written");
  } else {
    //Serial.println("- write failed");
  }
}

// Replaces placeholder with stored values
String processor(const String& var){
  //Serial.println(var);
  if(var == "WiFi SSID")
  {
    String SSID_FOR_HTML = readFile(SPIFFS, "/WiFi_SSID.txt");
    String EMPTY_VALUE = "";
    if (String("") == SSID_FOR_HTML) return "(None)";
    else return SSID_FOR_HTML;
  }
  else if(var == "JQ6500 Volume")
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

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(server_ssid, server_password);

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?WiFi SSID=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET WiFi SSID value on <ESP_IP>/get?WiFi SSID=<inputMessage>
    if (request->hasParam(PARAM_STRING_SSID)) {
      inputMessage = request->getParam(PARAM_STRING_SSID)->value();
      writeFile(SPIFFS, "/WiFi_SSID.txt", inputMessage.c_str());
    }
    // GET WiFi Password value on <ESP_IP>/get?WiFi Password=<inputMessage>
    else if (request->hasParam(PARAM_STRING_PASSWORD)) {
      inputMessage = request->getParam(PARAM_STRING_PASSWORD)->value();
      writeFile(SPIFFS, "/WiFi_Password.txt", inputMessage.c_str());
    }
    // GET JQ6500 Volume value on <ESP_IP>/get?JQ6500 Volume=<inputMessage>
    else if (request->hasParam(PARAM_VOLUME)) {
      inputMessage = request->getParam(PARAM_VOLUME)->value();
      writeFile(SPIFFS, "/JQ6500_Volume.txt", inputMessage.c_str());
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/text", inputMessage);
  });
  server.on("/play", HTTP_GET, [](AsyncWebServerRequest *request){
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

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  WiFi.begin(readFile(SPIFFS, "/WiFi_SSID.txt"), readFile(SPIFFS, "/WiFi_Password.txt"));
}

void loop() 
{
  if (TEST_SOUND_REQUEST)
  {
    TEST_SOUND_REQUEST = false;
    CANCEL_REQUEST = false;
    digitalWrite(2, LOW);
  }
  else if (CANCEL_REQUEST)
  {
    CANCEL_REQUEST = false;
    TEST_SOUND_REQUEST = false;
    digitalWrite(2, HIGH);
  }

  String yourInputString = readFile(SPIFFS, "/WiFi_SSID.txt");
  Serial.print("*** Your WiFi SSID: ");
  Serial.println(yourInputString);
  
//  String yourInputInt = readFile(SPIFFS, "/WiFi_Password.txt");
//  Serial.print("*** Your WiFi Password: ");
//  Serial.println(yourInputInt);
//  delay(5000);

  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.begin(readFile(SPIFFS, "/WiFi_SSID.txt"), readFile(SPIFFS, "/WiFi_Password.txt"));
    delay(5000);
    if (WiFi.status() == WL_CONNECTED)
    {
      server.begin();
    }
  }
  else
  {
    digitalWrite(2, LOW);
  }
}
