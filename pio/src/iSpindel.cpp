/**************************************************************
*
*  "iSpindel"
*  All rights reserverd by S.Lang <universam@web.de>
*
**************************************************************/

// Modified by Fernando Garcia <info@ferduino.com>

//#define NO_CONSOLE // Uncomment to disable console messages

// includes go here
#include "Globals.h"
#include <MQTT.h>
#include "MPUOffset.h"
// #endif
#include "OneWire.h"
#include "Wire.h"
// #include <Ticker.h>
#include "DallasTemperature.h"
#include "DoubleResetDetector.h" // https://github.com/datacute/DoubleResetDetector
#include "RunningMedian.h"
#include "WiFiManagerKT.h"
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino
#include <FS.h>          //this needs to be first
#include "tinyexpr.h"
#include <time.h>

WiFiClient _client;
MQTTClient _mqttClient(MAX_MESSAGE_SIZE);

// definitions go here
MPU6050_Base accelgyro;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
DeviceAddress tempDeviceAddress;
Ticker flasher;
RunningMedian samples = RunningMedian(MEDIANROUNDSMAX);
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

#define TEMP_CELSIUS       0
#define TEMP_FAHRENHEIT    1

#ifdef USE_DMP
#include "MPU6050.h"

// MPU control/status vars
bool dmpReady = false;     // set true if DMP init was successful
uint8_t mpuIntStatus;      // holds actual interrupt status byte from MPU
uint8_t devStatus;         // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;       // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;        // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];    // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#endif

bool shouldSaveConfig = false;

char my_username[TKIDSIZE];
char my_password[TKIDSIZE];
char my_polynominal[70] = "-0.00031*tilt^2+0.557*tilt-14.054";

String my_ssid;
String my_psk;
uint32_t my_sleeptime = 20; // Seconds
float my_vfact = ADCDIVISOR;
int16_t my_aX = UNINIT, my_aY = UNINIT, my_aZ = UNINIT;
uint8_t my_tempscale = TEMP_CELSIUS;

int my_timeZone  = -3;
bool isSummerTime = false;
int my_timestamp = 1536939980; // Fri, 14 Sep 2018 15:46:20 GMT

uint32_t DSreqTime;
float pitch, roll;

int16_t ax, ay, az;
float Volt, Temperatur, Tilt, Gravity;   // , corrGravity;

bool DSrequested = false;

time_t this_second = 0;

void callback(MQTTClient *client, char topic[], char payload[], int payload_length);

float scaleTemperature(float t)
{
  if (my_tempscale == TEMP_CELSIUS)
  {
    return t;
  }
  else if (my_tempscale == TEMP_FAHRENHEIT)
  {
    return(1.8f * t + 32);
  }
  else
  {
    return t;             // Invalid value for my_tempscale => default to celsius
  }
}


String tempScaleLabel()
{
  if (my_tempscale == TEMP_CELSIUS)
  {
    return "C";
  }
  else if (my_tempscale == TEMP_FAHRENHEIT)
  {
    return "F";
  }
  else
  {
    return "C";             // Invalid value for my_tempscale => default to celsius
  }
}


// callback notifying us of the need to save config
void saveConfigCallback()
{
  shouldSaveConfig = true;
}


void applyOffset()
{
  if ((my_aX != UNINIT) && (my_aY != UNINIT) && (my_aZ != UNINIT))
  {
    CONSOLELN(F("\nApplying offsets"));
    accelgyro.setXAccelOffset(my_aX);
    accelgyro.setYAccelOffset(my_aY);
    accelgyro.setZAccelOffset(my_aZ);
  }
  else
  {
    CONSOLELN(F("\nOffsets not available"));
  }
}


bool readConfig()
{
  CONSOLE(F("\nMounting FS..."));

  if (SPIFFS.begin())
  {
    CONSOLELN(F(" mounted!"));
    if (SPIFFS.exists(CFGFILE))
    {
      // file exists, reading and loading
      CONSOLELN(F("\nReading config file..."));
      File configFile = SPIFFS.open(CFGFILE, "r");
      if (configFile)
      {
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject&       json = jsonBuffer.parseObject(buf.get());

        if (json.success())
        {
          if (json.containsKey("Sleep"))
          {
            my_sleeptime = json["Sleep"];
          }
          if (json.containsKey("Username"))
          {
            strcpy(my_username, json["Username"]);
          }
          if (json.containsKey("ApiKey"))
          {
            strcpy(my_password, json["ApiKey"]);
          }
          if (json.containsKey("Vfact"))
          {
            my_vfact = json["Vfact"];
          }
          if (json.containsKey("TS"))
          {
            my_tempscale = json["TS"];
          }
          if (json.containsKey("SSID"))
          {
            my_ssid = (const char *)json["SSID"];
          }
          if (json.containsKey("PSK"))
          {
            my_psk = (const char *)json["PSK"];
          }
          if (json.containsKey("POLY"))
          {
            strcpy(my_polynominal, json["POLY"]);
          }
          if (json.containsKey("TimeZone"))
          {
            my_timeZone = json["TimeZone"];
          }
          if (json.containsKey("isSummerTime"))
          {
            isSummerTime = json["isSummerTime"];
          }

          my_aX = UNINIT;
          my_aY = UNINIT;
          my_aZ = UNINIT;

          if (json.containsKey("aX"))
          {
            my_aX = json["aX"];
          }
          if (json.containsKey("aY"))
          {
            my_aY = json["aY"];
          }
          if (json.containsKey("aZ"))
          {
            my_aZ = json["aZ"];
          }
          applyOffset();

          CONSOLELN(F("\nParsed config:"));
#ifndef NO_CONSOLE
          json.prettyPrintTo(Serial);
          CONSOLELN();
#endif
          return true;
        }
        else
        {
          CONSOLELN(F("ERROR: Failed to load json config"));
          return false;
        }
      }
      CONSOLELN(F("ERROR: Unable to open config file"));
    }
  }
  else
  {
    CONSOLELN(F(" ERROR: Failed to mount FS!"));
    return false;
  }
  return true;
}


bool shouldStartConfig(bool validConf)
{
  // we make sure that configuration is properly set and we are not woken by
  // RESET button
  // ensure this was called

  rst_info *_reset_info  = ESP.getResetInfoPtr();
  uint8_t _reset_reason = _reset_info->reason;

  // The ESP reset info is sill buggy. see http://www.esp8266.com/viewtopic.php?f=32&t=8411
  // The reset reason is "5" (woken from deep-sleep) in most cases (also after a power-cycle)
  // I added a single reset detection as workaround to enter the config-mode easier
  CONSOLE(F("\nBoot-mode: "));
  CONSOLELN(ESP.getResetReason());
  bool _poweredOnOffOn = _reset_reason == REASON_DEFAULT_RST || _reset_reason == REASON_EXT_SYS_RST;
  if (_poweredOnOffOn)
  {
    CONSOLELN(F("\nPower-cycle or reset detected, config mode"));
  }

  bool _dblreset = drd.detectDoubleReset();
  if (_dblreset)
  {
    CONSOLELN(F("\nDouble Reset detected"));
  }

  bool _wifiCred = (WiFi.SSID() != "");
  uint8_t c         = 0;
  if (!_wifiCred)
  {
    WiFi.begin();
  }
  while (!_wifiCred)
  {
    if (c > 10)
    {
      break;
    }
    CONSOLE('.');
    delay(100);
    c++;
    _wifiCred = (WiFi.SSID() != "");
  }
  if (!_wifiCred)
  {
    CONSOLELN(F("\nERROR no Wifi credentials"));
  }

  if (validConf && !_dblreset && _wifiCred && !_poweredOnOffOn)
  {
    CONSOLELN(F("\nWoken from deepsleep, normal mode"));
    return false;
  }
  // config mode
  else
  {
    CONSOLELN(F("\nGoing to config mode..."));
    return true;
  }
}


void validateInput(const char *input, char *output)
{
  String tmp = input;

  tmp.trim();
  tmp.replace(' ', '_');
  tmp.toCharArray(output, tmp.length() + 1);
}


String htmlencode(String str)
{
  String encodedstr = "";
  char c;

  for (uint16_t i = 0; i < str.length(); i++)
  {
    c = str.charAt(i);

    if (isalnum(c))
    {
      encodedstr += c;
    }
    else
    {
      encodedstr += "&#";
      encodedstr += String((uint8_t)c);
      encodedstr += ';';
    }
  }
  return encodedstr;
}


bool startConfiguration()
{
  WiFiManager wifiManager;

  wifiManager.setDebugOutput(WM_DEBUG);
  wifiManager.setConfigPortalTimeout(PORTALTIMEOUT);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setBreakAfterConfig(true);

  WiFiManagerParameter custom_mqtt_hint("<hr><h3>MQTT settings</h3>");
  wifiManager.addParameter(&custom_mqtt_hint);
  WiFiManagerParameter custom_username("username", "Username:", my_username, TKIDSIZE);
  wifiManager.addParameter(&custom_username);
  WiFiManagerParameter custom_password("password", "ApiKey:", my_password, TKIDSIZE);
  wifiManager.addParameter(&custom_password);
  WiFiManagerParameter custom_sleep("sleep", "Update Interval (s):", String(my_sleeptime).c_str(), 6, TYPE_NUMBER);
  wifiManager.addParameter(&custom_sleep);

  WiFiManagerParameter custom_calib_hint("<hr><h3>Calibration settings</h3>");
  wifiManager.addParameter(&custom_calib_hint);
  WiFiManagerParameter custom_polynom("POLYN", "Polynomial for gravity conversion:", htmlencode(my_polynominal).c_str(), 70 * 2, WFM_NO_LABEL);
  wifiManager.addParameter(&custom_polynom);
  WiFiManagerParameter custom_polynom_lbl("<p>Ex.: -0.00031*tilt^2+0.557*tilt-14.054</p>");
  wifiManager.addParameter(&custom_polynom_lbl);

  WiFiManagerParameter custom_misc_hint("<hr><h3>Other settings</h3>");
  wifiManager.addParameter(&custom_misc_hint);
  WiFiManagerParameter custom_vfact("vfact", "Battery conversion factor:", String(my_vfact).c_str(), 7, TYPE_NUMBER);
  wifiManager.addParameter(&custom_vfact);
  WiFiManagerParameter custom_tempscale_hint("<label for=\"TS\">Unit of temperature:</label>");
  wifiManager.addParameter(&custom_tempscale_hint);
  WiFiManagerParameter tempscale_list(HTTP_TEMPSCALE_LIST);
  wifiManager.addParameter(&tempscale_list);
  WiFiManagerParameter custom_tempscale("tempscale", "tempscale", String(my_tempscale).c_str(), 5, TYPE_HIDDEN, WFM_NO_LABEL);
  wifiManager.addParameter(&custom_tempscale);
  WiFiManagerParameter custom_timezone("timezone", "Time Zone:", String(my_timeZone).c_str(), 5, TYPE_NUMBER);
  wifiManager.addParameter(&custom_timezone);
  WiFiManagerParameter custom_summer_hint("<label for=\"ST\">Summer time:</label>");
  wifiManager.addParameter(&custom_summer_hint);
  WiFiManagerParameter summer_list(HTTP_SUMMER_LIST);
  wifiManager.addParameter(&summer_list);
  WiFiManagerParameter custom_summer("summer", "summer", String(isSummerTime).c_str(), 5, TYPE_HIDDEN, WFM_NO_LABEL);

  wifiManager.addParameter(&custom_summer);

  wifiManager.setConfSSID(htmlencode(my_ssid));
  wifiManager.setConfPSK(htmlencode(my_psk));

  CONSOLELN(F("\nPortal started. Type 192.168.4.1 on browser."));
  wifiManager.startConfigPortal("iSpindel");

  strcpy(my_polynominal, custom_polynom.getValue());

  validateInput(custom_username.getValue(), my_username);
  validateInput(custom_password.getValue(), my_password);
  uint32_t value = String(custom_sleep.getValue()).toInt();

  if ((value > 10) && (value < MAXSLEEPTIME))
  {
    my_sleeptime = value;
  }
  my_tempscale = String(custom_tempscale.getValue()).toInt();

  if (isNumericInt(String(custom_timezone.getValue())))
  {
    int tz = String(custom_timezone.getValue()).toInt();

    if ((tz <= 12) && (tz >= -12))
    {
      my_timeZone = tz;
    }
  }

  isSummerTime = String(custom_summer.getValue()).toInt();


  String tmp = custom_vfact.getValue();
  tmp.trim();
  tmp.replace(',', '.');
  my_vfact = tmp.toFloat();


  // save the custom parameters to FS
  if (shouldSaveConfig)
  {
    // Wifi config
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);

    return saveConfig();
  }
  return false;
}


bool isNumeric(String str)
{
  unsigned int stringLength = str.length();

  if (stringLength == 0)
  {
    return false;
  }

  boolean seenDecimal = false;

  for (unsigned int i = 0; i < stringLength; ++i)
  {
    if (isDigit(str.charAt(i)))
    {
      continue;
    }

    if (str.charAt(i) == '.')
    {
      if (seenDecimal)
      {
        return false;
      }
      seenDecimal = true;
      continue;
    }
    return false;
  }
  return true;
}


bool isNumericInt(String str)
{
  unsigned int stringLength = str.length();

  if (stringLength == 0)
  {
    return false;
  }

  boolean seenDecimal = false;

  for (unsigned int i = 0; i < stringLength; ++i)
  {
    if (isDigit(str.charAt(i)))
    {
      continue;
    }

    if ((str.charAt(i) == '-') || (str.charAt(i) == '+'))
    {
      if (seenDecimal)
      {
        return false;
      }
      seenDecimal = true;
      continue;
    }
    return false;
  }
  return true;
}


void formatSpiffs()
{
  CONSOLE(F("\nneed to format SPIFFS: "));
  SPIFFS.end();
  SPIFFS.begin();
  CONSOLELN(SPIFFS.format());
}


bool saveConfig()
{
  CONSOLE(F("saving config..."));

  // if SPIFFS is not usable
  if ((!SPIFFS.begin()) || (!SPIFFS.exists(CFGFILE)) || (!SPIFFS.open(CFGFILE, "w")))
  {
    formatSpiffs();
  }

  DynamicJsonBuffer jsonBuffer;
  JsonObject&       json = jsonBuffer.createObject();

  json["Sleep"] = my_sleeptime;
  json["Username"]     = my_username;
  json["ApiKey"]       = my_password;
  json["Vfact"]        = my_vfact;
  json["TS"]           = my_tempscale;
  json["TimeZone"]     = my_timeZone;
  json["isSummerTime"] = isSummerTime;

  // Store current Wifi credentials
  json["SSID"] = WiFi.SSID();
  json["PSK"]  = WiFi.psk();

  json["POLY"] = my_polynominal;
  json["aX"]   = my_aX;
  json["aY"]   = my_aY;
  json["aZ"]   = my_aZ;

  File configFile = SPIFFS.open(CFGFILE, "w+");
  if (!configFile)
  {
    CONSOLELN(F("failed to open config file for writing"));
    SPIFFS.end();
    return false;
  }
  else
  {
#ifndef NO_console
    json.printTo(Serial);
#endif
    json.printTo(configFile);
    configFile.close();
    SPIFFS.end();
    CONSOLELN(F("saved successfully"));
    return true;
  }
}


void uploadData()
{
  char pub_message[MAX_MESSAGE_SIZE];

  DynamicJsonBuffer jsonBuffer;

  JsonObject& Json = jsonBuffer.createObject();

  char CLIENT_ID[MAX_TOPIC_ID_SIZE];
  char PUB_TOPIC[MAX_TOPIC_ID_SIZE];
  char SUB_TOPIC[MAX_TOPIC_ID_SIZE];

  strcpy(CLIENT_ID, "TILT_ID: ");
  strcat(CLIENT_ID, my_username);

  strcpy(PUB_TOPIC, "tilt/log/");
  strcat(PUB_TOPIC, my_username);
  strcat(PUB_TOPIC, "/");
  strcat(PUB_TOPIC, my_password);

  strcpy(SUB_TOPIC, "tilt/config/");
  strcat(SUB_TOPIC, my_username);
  strcat(SUB_TOPIC, "/");
  strcat(SUB_TOPIC, my_password);

  CONSOLELN(F("\nStarting MQTT..."));

  CONSOLE(F("\nSubscribe topic: "));
  CONSOLELN(SUB_TOPIC);

  CONSOLE(F("\nPublish topic: "));
  CONSOLELN(PUB_TOPIC);

  _mqttClient.begin(MY_BROKER, MY_PORT, _client);
  _mqttClient.setOptions(5, false, 1000);       //keepAlive, cleanSession, timeout);
  _mqttClient.onMessageAdvanced(callback);

  byte i = 0;

  while (!_mqttClient.connected() && (i < 3))
  {
    CONSOLELN(F("\nAttempting MQTT connection..."));

    if (_mqttClient.connect(CLIENT_ID, my_username, my_password))
    {
      CONSOLELN(F("\nConnected to MQTT"));
    }
    else
    {
      CONSOLE(F("\nFailed MQTT connection, return code: "));

      int Status = _mqttClient.returnCode();

      switch (Status)
      {
      case -4:
        CONSOLELN(F("Connection timeout"));
        break;

      case -3:
        CONSOLELN(F("Connection lost"));
        break;

      case -2:
        CONSOLELN(F("Connect failed"));
        break;

      case -1:
        CONSOLELN(F("Disconnected"));
        break;

      case 1:
        CONSOLELN(F("Bad protocol"));
        break;

      case 2:
        CONSOLELN(F("Bad client ID"));
        break;

      case 3:
        CONSOLELN(F("Unavailable"));
        break;


      case 4:
        CONSOLELN(F("Bad credentials"));
        break;

      case 5:
        CONSOLELN(F("Unauthorized"));
        break;
      }

      CONSOLELN(F("Retrying MQTT connection in 5 seconds"));

      i++;
      delay(5000);
    }
  }

  if (_mqttClient.connected())
  {
    Json[F("username")] = my_username;
    Json[F("time")]        = my_timestamp;
    Json[F("tilt")]        = Tilt;
    Json[F("temperature")] = scaleTemperature(Temperatur);
    Json[F("temp_units")]  = tempScaleLabel();
    Json[F("battery")]     = Volt;
    Json[F("gravity")]     = Gravity;
    Json[F("interval")]    = my_sleeptime;
    Json[F("timezone")]    = my_timeZone;
    Json[F("summer")]      = isSummerTime;
    Json[F("RSSI")]        = WiFi.RSSI();

    Json.printTo(pub_message, Json.measureLength() + 1);
    _mqttClient.publish(PUB_TOPIC, pub_message, false, 1);             // retained, Qos

#ifndef NO_CONSOLE
    CONSOLELN("\nMessage published:");
    Json.prettyPrintTo(Serial);
    CONSOLELN();
#endif

    _mqttClient.subscribe(SUB_TOPIC, 1);             //Topic, Qos
    _mqttClient.loop();

    CONSOLELN(F("\nClosing MQTT connection"));
    _mqttClient.disconnect();
    delay(100);             // allow gracefull session close
  }
}


void callback(MQTTClient *client, char topic[], char payload[], int payload_length)
{
  char RES_TOPIC[MAX_TOPIC_ID_SIZE];
  char SUB_TOPIC[MAX_TOPIC_ID_SIZE];
  DynamicJsonBuffer jsonBuffer;
  JsonObject&       json    = jsonBuffer.parseObject((char *)payload);
  bool has_key = false;

  strcpy(RES_TOPIC, "tilt/response/");
  strcat(RES_TOPIC, my_username);
  strcat(RES_TOPIC, "/");
  strcat(RES_TOPIC, my_password);

  strcpy(SUB_TOPIC, "tilt/config/");       // Used to send a message back to delete retained messages
  strcat(SUB_TOPIC, my_username);
  strcat(SUB_TOPIC, "/");
  strcat(SUB_TOPIC, my_password);

  CONSOLE(F("\nResponse topic: "));
  CONSOLELN(RES_TOPIC);

  if (json.success())
  {
    CONSOLELN(F("\nMessage received: "));
#ifndef NO_CONSOLE
    json.prettyPrintTo(Serial);
    CONSOLELN();
#endif

    if (json.containsKey("interval"))
    {
      uint32_t interval = json["interval"];
      if ((interval != my_sleeptime) && (interval < MAXSLEEPTIME) && (interval > 10))
      {
        my_sleeptime = interval;
        has_key      = true;
      }
      else
      {
        has_key = false;
      }
    }
    else if (json.containsKey("factor"))
    {
      String tmp = json["factor"];
      tmp.trim();
      tmp.replace(',', '.');
      if (isNumeric(tmp))
      {
        my_vfact = tmp.toFloat();
        has_key  = true;
      }
      else
      {
        has_key = false;
      }
    }
    else if (json.containsKey("unit"))
    {
      if (json["unit"] == "c")
      {
        my_tempscale = 0;
        has_key      = true;
      }
      else if (json["unit"] == "f")
      {
        my_tempscale = 1;
        has_key      = true;
      }
      else
      {
        has_key = false;
      }
    }
    else if (json.containsKey("summer"))
    {
      if (json["summer"] == true)
      {
        isSummerTime = 1;
        has_key      = true;
      }
      else if (json["summer"] == false)
      {
        isSummerTime = 0;
        has_key      = true;
      }
      else
      {
        has_key = false;
      }
    }
    else if (json.containsKey("timezone"))
    {
      if (isNumericInt(json["timezone"]))
      {
        int tz = json["timezone"];

        if ((tz <= 12) && (tz >= -12))
        {
          my_timeZone = tz;
          has_key     = true;
        }
        else
        {
          has_key = false;
        }
      }
      else
      {
        has_key = false;
      }
    }

    if (has_key == true)
    {
      if (saveConfig() == true)
      {
        CONSOLELN(F("\nNew configuration was saved. Will be applied in next time."));
        _mqttClient.publish(RES_TOPIC, "New configuration was saved. Will be applied in next time.", false, 1);                         // retained, Qos
      }
      else
      {
        CONSOLELN(F("\nCan't save configuration. Please try again."));
        _mqttClient.publish(RES_TOPIC, "Can't save configuration. Please try again.", false, 1);                         // retained, Qos
      }
    }
    else
    {
      CONSOLELN(F("\nKey not found or value invalid."));
      _mqttClient.publish(RES_TOPIC, "Key not found or value invalid.", false, 1);                   // retained, Qos
    }
  }
  else
  {
    CONSOLELN(F("\nCan't parse JSON! Check if it's valid in jsonlint.com"));
    _mqttClient.publish(RES_TOPIC, "Can't parse JSON! Check if it's valid in jsonlint.com", false, 1);             // retained, Qos
  }
}


void goodNight(uint32_t seconds)
{
  uint32_t _seconds   = seconds;
  uint32_t left2sleep = 0;
  uint32_t validflag  = RTCVALIDFLAG;

  drd.stop();

  // workaround for DS not floating
  pinMode(ONE_WIRE_BUS, OUTPUT);
  digitalWrite(ONE_WIRE_BUS, LOW);

  // we need another incarnation before work run
  if (_seconds > MAXSLEEPTIME)
  {
    left2sleep = _seconds - MAXSLEEPTIME;
    ESP.rtcUserMemoryWrite(RTCSLEEPADDR, &left2sleep, sizeof(left2sleep));
    ESP.rtcUserMemoryWrite(RTCSLEEPADDR + 1, &validflag, sizeof(validflag));
    CONSOLELN(String(F("\nStep-sleep: ")) + MAXSLEEPTIME + "s; left: " + left2sleep + " s; Time spent: " + millis() + " ms\n");
    ESP.deepSleep(MAXSLEEPTIME * 1e6, WAKE_RF_DISABLED);
  }
  // regular sleep with RF enabled after wakeup
  else
  {
    // clearing RTC to mark next wake
    left2sleep = 0;
    ESP.rtcUserMemoryWrite(RTCSLEEPADDR, &left2sleep, sizeof(left2sleep));
    ESP.rtcUserMemoryWrite(RTCSLEEPADDR + 1, &validflag, sizeof(validflag));
    CONSOLELN(String(F("\nGoing to sleep ")) + _seconds + " s; Time spent: " + millis() + " ms\n");
    // WAKE_RF_DEFAULT --> auto reconnect after wakeup
    ESP.deepSleep(_seconds * 1e6, WAKE_RF_DEFAULT);
  }
  // workaround proper power state init
  delay(500);
}


void sleepManager()
{
  uint32_t left2sleep, validflag;

  ESP.rtcUserMemoryRead(RTCSLEEPADDR, &left2sleep, sizeof(left2sleep));
  ESP.rtcUserMemoryRead(RTCSLEEPADDR + 1, &validflag, sizeof(validflag));

  // check if we have to incarnate again
  if ((left2sleep != 0) && !drd.detectDoubleReset() && (validflag == RTCVALIDFLAG))
  {
    goodNight(left2sleep);
  }
  else
  {
    CONSOLELN(F("\nWorker run!"));
  }
}


void requestTemp()
{
  if (!DSrequested)
  {
    DS18B20.requestTemperatures();
    DSreqTime   = millis();
    DSrequested = true;
  }
}


void initDS18B20()
{
  // workaround for DS not enough power to boot
  pinMode(ONE_WIRE_BUS, OUTPUT);
  digitalWrite(ONE_WIRE_BUS, LOW);
  delay(100);

  DS18B20.begin();
  DS18B20.setWaitForConversion(false);
  DS18B20.getAddress(tempDeviceAddress, 0);
  DS18B20.setResolution(tempDeviceAddress, RESOLUTION);
  requestTemp();
}


bool isDS18B20ready()
{
  return millis() - DSreqTime > OWinterval;
}


void initAccel()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(D2, D1);
  Wire.setClock(100000);
  Wire.setClockStretchLimit(2 * 230);

  // init the Accel
  accelgyro.initialize();
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);
  accelgyro.setTempSensorEnabled(true);
#ifdef USE_DMP
  accelgyro.setDMPEnabled(true);
  packetSize = accelgyro.dmpGetFIFOPacketSize();
#endif
  accelgyro.setInterruptLatch(0);       // pulse
  accelgyro.setInterruptMode(1);       // Active Low
  accelgyro.setInterruptDrive(1);       // Open drain
  accelgyro.setRate(17);
  accelgyro.setIntDataReadyEnabled(true);
  testAccel();
}


float calculateTilt()
{
  float _ax   = ax;
  float _ay   = ay;
  float _az   = az;
  float pitch = (atan2(_ay, sqrt(_ax * _ax + _az * _az))) * 180.0 / M_PI;
  float roll  = (atan2(_ax, sqrt(_ay * _ay + _az * _az))) * 180.0 / M_PI;

  return sqrt(pitch * pitch + roll * roll);
}


bool testAccel()
{
  uint8_t res = Wire.status();

  if (res != I2C_OK)
  {
    CONSOLELN(String(F("I2C ERROR: ")) + res);
  }
  bool con = accelgyro.testConnection();
  if (!con)
  {
    CONSOLELN(F("Acc Test Connection ERROR!"));
  }
  return res == I2C_OK && con == true;
}


void getAccSample()
{
  accelgyro.getAcceleration(&ax, &az, &ay);
}


float getTilt()
{
  uint32_t start = millis();
  uint8_t i     = 0;

  for (; i < MEDIANROUNDSMAX; i++)
  {
    while (!accelgyro.getIntDataReadyStatus())
    {
      delay(2);
    }
    getAccSample();
    float _tilt = calculateTilt();
    samples.add(_tilt);

    if ((i >= MEDIANROUNDSMIN) && isDS18B20ready())
    {
      break;
    }
  }
  CONSOLE(F("\nSamples:"));
  CONSOLE(++i);
  CONSOLE(F(" Min:"));
  CONSOLE(samples.getLowest());
  CONSOLE(F(" Max:"));
  CONSOLE(samples.getHighest());
  CONSOLE(F(" Time:"));
  CONSOLE(millis() - start);
  CONSOLELN(F(" ms"));
  return samples.getAverage(MEDIANAVRG);
}


float getTemperature(bool block = false)
{
  // we need to wait for DS18b20 to finish conversion
  float t = Temperatur;

  // if we need the result we have to block
  while (block && (millis() - DSreqTime <= OWinterval))
  {
    yield();
  }

  if (millis() - DSreqTime >= OWinterval)
  {
    t           = DS18B20.getTempCByIndex(0);
    DSrequested = false;

    if ((t == DEVICE_DISCONNECTED_C) || (t == 85.0))                    // we read 85 uninitialized
    {
      CONSOLELN(F("\nERROR: DS18B20 DISCONNECTED"));
      pinMode(ONE_WIRE_BUS, OUTPUT);
      digitalWrite(ONE_WIRE_BUS, LOW);
      delay(100);
      oneWire.reset();

      if (block)
      {
        CONSOLELN(F("\nTrying again..."));
        initDS18B20();
        delay(OWinterval + 100);
        t = getTemperature(false);
      }
    }
  }
  return t;
}


float getBattery()
{
  analogRead(A0);       // drop first read
  return analogRead(A0) / my_vfact;
}


float calculateGravity()
{
  double _tilt    = Tilt;
  double _temp    = Temperatur;
  float _gravity = 0;
  int err;
  te_variable vars[] = { { "tilt", &_tilt }, { "temp", &_temp }};
  te_expr     *expr = te_compile(my_polynominal, vars, 2, &err);

  if (expr)
  {
    _gravity = te_eval(expr);
    te_free(expr);
  }
  else
  {
    CONSOLELN(String(F("Parse error at ")) + err);
  }
  return _gravity;
}


void flash()
{
  // triggers the LED
  Volt = getBattery();
  getAccSample();
  Tilt       = calculateTilt();
  Temperatur = getTemperature(false);
  Gravity    = calculateGravity();
  requestTemp();
}


bool isSafeMode(float _volt)
{
  if (_volt < LOWBATT)
  {
    CONSOLELN(F("\nWARNING: low Battery"));
    return true;
  }
  else
  {
    return false;
  }
}


void connectBackupCredentials()
{
  WiFi.disconnect();
  WiFi.begin(my_ssid.c_str(), my_psk.c_str());
  CONSOLELN(F("Rescue Wifi credentials"));
  delay(100);
}


void setup()
{
  Serial.begin(115200);

  CONSOLELN(F("\n\nFirmware version: " FIRMWAREVERSION));
  CONSOLE(F("\nSDK version: "));
  CONSOLELN(ESP.getSdkVersion());

  sleepManager();

  bool validConf = readConfig();
  if (!validConf)
  {
    CONSOLELN(F("\nERROR config corrupted"));
  }
  initDS18B20();
  initAccel();

  // decide whether we want configuration mode or normal mode
  if (shouldStartConfig(validConf))
  {
    uint32_t tmp;
    ESP.rtcUserMemoryRead(WIFIENADDR, &tmp, sizeof(tmp));

    // DIRTY hack to keep track of WAKE_RF_DEFAULT --> find a way to read WAKE_RF_*
    if (tmp != RTCVALIDFLAG)
    {
      drd.setRecentlyResetFlag();
      tmp = RTCVALIDFLAG;
      ESP.rtcUserMemoryWrite(WIFIENADDR, &tmp, sizeof(tmp));
      CONSOLELN(F("reboot RFCAL"));
      ESP.deepSleep(100000, WAKE_RFCAL);
      delay(500);
    }
    else
    {
      tmp = 0;
      ESP.rtcUserMemoryWrite(WIFIENADDR, &tmp, sizeof(tmp));
    }

    flasher.attach(1, flash);

    // rescue if wifi credentials lost because of power loss
    if (!startConfiguration())
    {
      // test if ssid exists
      if ((WiFi.SSID() == "") &&
          (my_ssid != "") && (my_psk != ""))
      {
        connectBackupCredentials();
      }
    }
    uint32_t left2sleep = 0;
    ESP.rtcUserMemoryWrite(RTCSLEEPADDR, &left2sleep, sizeof(left2sleep));

    flasher.detach();
  }
  // to make sure we wake up with STA but AP
  WiFi.mode(WIFI_STA);
  Volt = getBattery();
  // we try to survive
  if (isSafeMode(Volt))
  {
    WiFi.setOutputPower(0);
  }
  else
  {
    WiFi.setOutputPower(20.5);
  }

#ifndef USE_DMP
  Tilt = getTilt();
#else
  while (fifoCount < packetSize)
  {
    //do stuff
    CONSOLELN(F("Wait DMP"));

    fifoCount = accelgyro.getFIFOCount();
  }
  if (fifoCount == 1024)
  {
    CONSOLELN(F("FIFO overflow"));
    accelgyro.resetFIFO();
  }
  else
  {
    fifoCount = accelgyro.getFIFOCount();

    accelgyro.getFIFOBytes(fifoBuffer, packetSize);

    accelgyro.resetFIFO();

    fifoCount -= packetSize;

    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetEuler(euler, &q);

    /*
     * for (int i = 1; i < 64; i++) {
     * CONSOLE(fifoBuffer[i]);
     * CONSOLE(" ");
     * }
     */

    CONSOLE(F("Euler\t"));
    CONSOLE((euler[0] * 180 / M_PI));
    CONSOLE("\t");
    CONSOLE(euler[1] * 180 / M_PI);
    CONSOLE("\t");
    CONSOLELN(euler[2] * 180 / M_PI);

    ax = euler[0];
    ay = euler[2];
    az = euler[1];

    float _ax   = ax;
    float _ay   = ay;
    float _az   = az;
    float pitch = (atan2(_ay, sqrt(_ax * _ax + _az * _az))) * 180.0 / M_PI;
    float roll  = (atan2(_ax, sqrt(_ay * _ay + _az * _az))) * 180.0 / M_PI;
    Tilt = sqrt(pitch * pitch + roll * roll);
  }
#endif

  Temperatur = accelgyro.getTemperature() / 340.00 + 36.53;
  accelgyro.setSleepEnabled(true);

  CONSOLE(F("\nAx: "));
  CONSOLE(ax);
  CONSOLE(F("\tAy: "));
  CONSOLE(ay);
  CONSOLE(F("\tAz: "));
  CONSOLELN(az);

  CONSOLE(F("\nAbs. tilt: "));
  CONSOLELN(Tilt);
  CONSOLE(F("\nBattery: "));
  CONSOLE(Volt);
  CONSOLELN(F(" V"));

  // calc gravity on user defined polynominal
  Gravity = calculateGravity();
  CONSOLE(F("\nGravity: "));
  CONSOLE(Gravity);
  CONSOLELN(F(" g/cm3"));

  CONSOLE(F("\nGY temp.: "));
  CONSOLE(Temperatur);
  CONSOLELN(F(" C"));
  // call as late as possible since DS needs converge time
  Temperatur = getTemperature(true);
  CONSOLE(F("\nDS temp.: "));
  CONSOLE(Temperatur);
  CONSOLELN(F(" C"));

  // water anomaly correction
  // float _temp = Temperatur - 4; // polynominal at 4
  // float wfact = 0.00005759 * _temp * _temp * _temp - 0.00783198 * _temp * _temp - 0.00011688 * _temp + 999.97;
  // corrGravity = Gravity - (1 - wfact / 1000);
  // CONSOLE(F("\tcorrGravity: "));
  // CONSOLELN(corrGravity);

  CONSOLELN(F("\nConnecting to WiFi..."));

  unsigned long startedAt = millis();

  CONSOLE(F("\nAfter wait "));
  // int connRes = WiFi.waitForConnectResult();
  uint8_t wait = 0;
  while (WiFi.status() == WL_DISCONNECTED)
  {
    delay(100);
    wait++;
    if (wait > 50)
    {
      break;
    }
  }
  auto waited = (millis() - startedAt);
  CONSOLE(waited);
  CONSOLE(F(" ms, result "));
  CONSOLELN(WiFi.status());

  if (WiFi.status() == WL_CONNECTED)
  {
    int offsetTMZ = (my_timeZone * 3600);
    byte i = 0;

    if(isSummerTime == true)
    {
      offsetTMZ -= 3600; // workaround to set DST.
    }

    configTime(offsetTMZ, 0, "pool.ntp.org", "time.nist.gov");

    CONSOLE(F("\nConnecting to NTP server "));
    while (!this_second && i < 10)
    {
      my_timestamp = time(&this_second);
      CONSOLE(".");
      delay(100);
      i++;
    }
    CONSOLELN();

    if (!this_second)
    {
      CONSOLE("\nCan't get time");
    }
    else
    {
      CONSOLE("\nCurrent time: ");
      CONSOLE(String(ctime(&this_second)));
    }

    CONSOLE(F("\nConnected. IP: "));
    CONSOLELN(WiFi.localIP());

    uploadData();
    delay(100);             // workaround for https://github.com/esp8266/Arduino/issues/2750
  }
  else
  {
    connectBackupCredentials();
    CONSOLELN(F("\nFailed to connect"));
  }

  // survive - 60min sleep time
  if (isSafeMode(Volt))
  {
    my_sleeptime = EMERGENCYSLEEP;
  }
  goodNight(my_sleeptime);
}


void loop()
{
  CONSOLELN(F("\nShould never be here!"));
}
