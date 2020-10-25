// HTML web page to handle input fields
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Input recorded and issued to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
    function updateMessage() {
      alert("New credentials submitted.");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
    function discardMessage() {
      alert("New credential data discarded!");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
    function TimeCalendarMessage() {
      alert("New time/date issued.");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
    function CredentialUpdateMessage() {
      alert("Local network credentials are being updated.");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script></head>
  <body>
  <h1 align = "center">System Configurations</h1><br><br>
  
  <h3 align = "left">ESP8266 Network Credentials:</h3>
  <form action="/get" target="hidden-form">     
      &nbsp;&nbsp;&nbsp;&nbsp;New AP SSID:<br>&nbsp;&nbsp;&nbsp;&nbsp;
        &nbsp;&nbsp;&nbsp;&nbsp;%AP SSID%<br>&nbsp;&nbsp;&nbsp;&nbsp;
        &nbsp;&nbsp;&nbsp;&nbsp;>>> %New AP SSID%<br>&nbsp;&nbsp;&nbsp;&nbsp;
        &nbsp;&nbsp;&nbsp;&nbsp;>>> <input type="text" name="AP SSID">
        <input type="submit" value="Submit" onclick="submitMessage()">
  </form>
  <form action="/get" target="hidden-form">
      &nbsp;&nbsp;&nbsp;&nbsp;New AP Password:<br>&nbsp;&nbsp;&nbsp;&nbsp;
        &nbsp;&nbsp;&nbsp;&nbsp;%AP Password%<br>&nbsp;&nbsp;&nbsp;&nbsp; 
        &nbsp;&nbsp;&nbsp;&nbsp;>>> %New AP Password%<br>&nbsp;&nbsp;&nbsp;&nbsp;
        &nbsp;&nbsp;&nbsp;&nbsp;>>> <input type="text" name="AP Password">
        <input type="submit" value="Submit" onclick="submitMessage()">
  </form>
  <h4 align = "center">
    <a href="/UpdateRequest" target="hidden-form"><button class="button" onclick="CredentialUpdateMessage()">Update</button></a>
    <a href="/Discard" target="hidden-form"><button class="button" onclick="discardMessage()">Discard</button></a>
  </h4><br>

  <h3 align = "left">Time Setup:</h3>
  <h4 align = "center">
    <form action="/ClockCalendar" target="hidden-form">
      <input type="time" name="Input Time">
      <input type="submit" value="Submit" onclick="TimeCalendarMessage()">
    </form>
  </h4>
  
  <h3 align = "left">Date Setup:</h3>
  <h4 align = "center">
    <form action="/ClockCalendar" target="hidden-form">
      <input type="date" name="Input Date">
      <input type="submit" value="Submit" onclick="TimeCalendarMessage()">
    </form>
  </h4>
  
  <iframe style="display:none" name="hidden-form"></iframe>
  <br><br><br><h4 align = "center"><a href="javascript:location.reload(true)">Refresh this Page</a></h4>
</body></html>)rawliteral";
