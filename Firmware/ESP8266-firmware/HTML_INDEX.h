// HTML web page to handle input fields
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script></head>
  <body>
  <h1 align = "center">ESP8266 Configurations</h1><br><br>
  
  <h3 align = "left">ESP8266 Network Credentials:</h3>
  <form action="/get" target="hidden-form">     
      &nbsp;&nbsp;&nbsp;&nbsp;New AP SSID:<br>&nbsp;&nbsp;&nbsp;&nbsp;
		&nbsp;&nbsp;&nbsp;&nbsp;%AP SSID%<br>&nbsp;&nbsp;&nbsp;&nbsp;
        &nbsp;&nbsp;&nbsp;&nbsp;>>> <input type="text" name="AP SSID">
        <input type="submit" value="Submit" onclick="submitMessage()">
  </form>
  <form action="/get" target="hidden-form">
      &nbsp;&nbsp;&nbsp;&nbsp;New AP Password:<br>&nbsp;&nbsp;&nbsp;&nbsp;
		&nbsp;&nbsp;&nbsp;&nbsp;%AP Password%<br>&nbsp;&nbsp;&nbsp;&nbsp;	
        &nbsp;&nbsp;&nbsp;&nbsp;>>> <input type="text" name="AP Password">
        <input type="submit" value="Submit" onclick="submitMessage()">
  </form>
  
  <br>
  <h4 align = "center"><a href="/ForceUpdate" target="hidden-form"><button class="button">Update / Restart</button></a></h4>
  <br><br>

  <h3 align = "left">Speaker:</h3>
	&nbsp;&nbsp;&nbsp;&nbsp;Current volume: %JQ6500 Volume%&nbsp;&nbsp;&nbsp;&nbsp;
	<a href="/play" target="hidden-form"><button class="button">Play test sound</button></a>      
    <a href="/stop" target="hidden-form"><button class="button">Stop</button></a>
    <br>
  <form action="/get" target="hidden-form">
	&nbsp;&nbsp;&nbsp;&nbsp;Set to (0 - 30): <input type="number" name="JQ6500 Volume">
    <input type="submit" value="OK" onclick="submitMessage()">
  </form>
  <iframe style="display:none" name="hidden-form"></iframe>
  <br><br><br><h4 align = "center"><a href="javascript:location.reload(true)">Refresh this Page</a></h4>
</body></html>)rawliteral";