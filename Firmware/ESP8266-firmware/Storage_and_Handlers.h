// Parameter handlers for HTML inputs
const char* PARAM_STRING_AP_SSID = "AP SSID";
const char* PARAM_STRING_AP_PASSWORD = "AP Password";
const char* PARAM_VOLUME = "JQ6500 Volume";
const char* PARAM_INT_SECOND = "Input Second";
const char* PARAM_INT_MINUTE = "Input Minute";
const char* PARAM_INT_HOUR = "Input Hour";
const char* PARAM_INT_YEAR = "Input Year";
const char* PARAM_INT_MONTH = "Input Month";
const char* PARAM_INT_DATE = "Input Date";

// Network credentials temporary storage for update
typedef struct {
  String CURRENT_AP_SSID;
  String CURRENT_AP_PASSWORD;
  
  String NEW_AP_SSID;
  String NEW_AP_PASSWORD;

  bool NewAPSSID;
  bool NewAPPassword;
  bool NewDataSet;
} NetworkCredentials;

// Clock and Calendar temporary storage for transmission
typedef struct {
  uint8_t SecondData;
  uint8_t MinuteData;
  uint8_t HourData;

  bool NewDataSet;
} TimeData;

typedef struct {
  uint8_t DateData;
  uint8_t MonthData;
  uint16_t YearData;

  bool NewDataSet;
} CalendarData;
