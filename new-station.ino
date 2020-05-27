/*
  New Weather Station - Arduino code
  Version:  3.0.526 alpha
  Date:     26th May, 2020
  Author:   Brian Naylor
  Source:   (to be posted) https://github.com/brian.naylor/arduino/weather-station

  Compiler settings: Board: WeMos D1 R1;  CPU Frequency 80Mhz;  IwIP variant: v2 Higher Bandwidth;  SSL Support : All SSL ciphers;  Flash Size 4Mb / FS 2Mb

      *  Pin Mapping for WemoS D1 Mini Board
      *
      *  Board Pin    Function    ESP8266     Connection
      *  TX           TXD         TXD         Not used
      *  RX           RXD         RXD         Not used
      *  A0           Analog      A0          Wind Direction to Davis Anemometer
      *  D0           i/o         GPIO16      Not used
      *  D1           i/o, SCL    GPIO5       BME280 SCL line
      *  D2           i/o, SDA    GPIO4       BME280, SDA line
      *  D3           i/o         GPIO0       Not used
      *  D4           i/o, LED    GPIO2       Not used
      *  D5           i/o, SCK    GPIO14      Wind Speed to Davis Anemometer
      *  D6           i/o, MISO   GPIO12      DS12B80 data line
      *  D7           i/o, MISK   GPIO13      Not used
      *  D8           i/o         GPIO15      Not used
      *  GND          Ground      GND         Common Ground
      *  5V           5V                      Power to Davis Anemometer
      *  3V3          3.3V        3.3V        Common Power rail (BME, DS12B80)
      *  RES          Reset       RST         No used

*/

// include files
#include <LittleFS.h>                   // File system for storing local config.json file with parameters
#include <WiFiManager.h>                // https://github.com/tzapu/WiFiManager
#include <ESP8266httpUpdate.h>
#include <ArduinoJson.h>                // https://github.com/bblanchon/ArduinoJson,  tested with 6.15.2
#include "PubSubClient.h"               // MQTT library for publishing and subscribing to topics (placed in directory of source INO)
#include "TimeLib.h"                    // hour, minute, day and time conversions
#include <Adafruit_Sensor.h>            // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BME280.h>            // https://github.com/adafruit/Adafruit_BME280_Library

// setup PIN mapping for devices
#define WindSensorPin (14)              // The pin location of the anemometer sensor / GPIO14 (D5 for NodeMcu)
#define WindVanePin (A0)                // The pin the wind vane sensor is connected to
#define LED 2

// globals
#define VERSION     "\n\n-----------------  MKSC Wind Station v3.0.526 OTA -----------------"
#define NameAP      "WindStationAP"
#define PasswordAP  "87654321"
int wifiRetries = 10;             // number of times to retry wifi reconnection
int requestRestart = 0;           // request the restart of the Arduino
bool shouldSaveConfig = false;    //flag for saving data to config.json

const char* otaUpdateBinary = "https://mksc.org.uk/wp-content/bin/mkscwgstn.bin";
const char* secureFingerprint = "D5:A4:FF:3E:04:86:C1:7D:61:96:6E:39:3C:F1:75:9F:6A:14:61:90";


// globals for config.json on file system.  This file contains the control params for the weather station
char mqtt_server[30] = "m24.cloudmqtt.com";
char mqtt_port[6] = "10818";
char mqtt_user[20] = "aescqugp";
char mqtt_pass[20] = "WUsOsaQhianC";
char wind_correction[4] = "100";                            // wind correction 1-999%
char windguru_uid[30] = "WG_MKSC_UK";                       // WindGuru user id
char windguru_pass[20] = "259#Nay4Who";                     // WindGuru password
char windguru_upload[4] = "ON";                             // Uploads to WindGuru, either ON or OFF
char vaneMaxADC[5] = "1024";                                // ADC range for input voltage 0..1V
char vaneOffset[4] = "0";                                   // define the offset for caclulating wind direction from magnetic north
char owApiKey[34] = "cc7d2795579e3f511f0c6abde97ea100";     // API key for openweathermap
char owLat[10] = "52.0059";                                 // Latitude for openweathermap readings eg. "52.0059"
char owLong[10] = "-0.7028";                                // Longtitude for openweathermap readings eg. "-0.7028"
int ELEVATION = 65;                                         // Height of your weather station above sea level (in meters)

// MQTT topic definitions & globals
#define MQTT_TOPIC      "mkscnewstn"        // mqtt topic (Must be unique for each device)
#define MQTT_TOPICc     "mkscnewstn/c"      // wind correction for speed calibration
#define MQTT_TOPICv     "mkscnewstn/v"      // vane offset for direction calibration
#define MQTT_TOPICw     "mkscnewstn/w"      // wind source - either device/probe or Internet
#define MQTT_TOPICt     "mkscnewstn/t"      // temp source - either device/probe or Internet
#define MQTT_TOPICp     "mkscnewstn/p"      // pressure source - either device/probe or Internet
#define MQTT_TOPICu     "mkscnewstn/u"      // upload readings to WindGuru - on or off
#define MQTT_TOPICf     "mkscnewstn/f"      // flash Arduino with new code.
#define MQTT_TOPICr     "mkscnewstn/r"      // reset / reboot Arduino
#define MQTT_TOPICd     "mkscnewstn/d"      // debugging level sent to MQTT

//Variables to hold good/true weather metrics and variables from open weather.
float temperature = 0, pressure = 0, humidity = 0, mslp = 0;
float owTemp = 0, owPressure = 0, owHumidity = 0, owWindAvg = 0, owWindMax = 0, owWindMin = 0;
float windAvg = 0, windMax = 0, windMin = 999, windTot = 0;
int direction = 0, owDirection = 0;

//Variables defining the source of the weather metrics - note if primary source is invalid, alternate will be used.
int sourceWind = 0;                         // 0 = Davis Anemometer (default) ; 1 = OpenWeather
int sourceDir = 1;                          // 0 = Davis Anemometer (default) ; 1 = OpenWeather
int sourceTemp = 1;                         // 0 = DS1280B probe (default) ; 1 = OpenWeather; 2 = Alternative probe / BME280
int sourcePressure = 0;                     // 0 = BME280 (default); 1 = OpenWeather
int sourceHumidity = 0;                     // 0 = BME280 (default); 1 = OpenWeather


// Timer Variables and wind counters
volatile unsigned long rotations;           // cup rotation counter used in interrupt routine
volatile unsigned long contactBounceTime;   // Timer to avoid contact bounce in isr
unsigned long timerCount;                   // counter to track elapsed millisecs
unsigned long davisUpdateFreq = 3;          // update frequency to sample wind from Davis Anemometer (in seconds)
unsigned long wgUpdateFreq = 60;            // update interval in seconds for WindGuru (UPLOAD) (in seconds)
unsigned long wgUpdateCounter = 0;          // counter to track progress against update interval for WindGuru
unsigned long owUpdateFreq = 5;             // update interval in minutes for OpenWeather API (DOWNLOAD)  (in minutes)
unsigned long owUpdateCounter = 99;         // counter to track progress against update interval
float windSpeed;                            // speed miles per hour


// Debugging variable
int debugLevel = 1;                         // 0=off; 1=informational; 2;=extensive/mqtt; 3=verbose/mqtt


//Sensor variables
Adafruit_BME280 bme;                        // Setup BME data structure to hold temp, pressure, humidity
WiFiClient WifiClient;
PubSubClient mqttClient(WifiClient);


//pre-declared functions
static void ota_update();


//callback notifying us of the need to save config.json file locally
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
} // end saveConfigCallback


// MQTT inbound messages from topic subscription
void callback(const MQTT::Publish& pub) {
  //Display inbound message - topic and payload
  String payload;

  payload = pub.payload_string();
  Serial.println("MQTT Topic: " + pub.topic() + " MQTT Payload: " + payload);

  if (pub.topic() == MQTT_TOPICc ) // wind_correction
  {
    Serial.println("Wind correction MQTT received : " + payload);
    strcpy(wind_correction, payload.c_str());
  } // endif wind_correction

  if (pub.topic() == MQTT_TOPICv ) // vane offset
  {
    Serial.println("Vane offset MQTT received");
    strcpy(vaneOffset, payload.c_str());
  } // endif vane offset

  if (pub.topic() == MQTT_TOPICu ) // WindGuru uploads ON / OFF
  {
    Serial.println("WindGuru uploads MQTT received");
    strcpy(windguru_upload, payload.c_str());
  } // endif WindGuru uploads

  if (pub.topic() == MQTT_TOPICf ) // flash Arduino
  {
    Serial.println("Flash MQTT received");
    ota_update();    // call for over-the-air binary program update
  } // endif flash

  if (pub.topic() == MQTT_TOPICr ) // reset / reboot Arduino
  {
    Serial.println("Restart MQTT received");
    requestRestart = 1;
  }  // endif reset / reboot

  if (pub.topic() == MQTT_TOPICd ) // debug to MQTT and debug level in payload
  {
    // 0 = DS1280B probe (default) ; 1 = OpenWeather; 2 = Alternative probe / BME280
    Serial.print("Debug MQTT received, and level: " + payload);
    switch (atoi(payload.c_str())) {
      case 0:
        Serial.println(" OFF");
        debugLevel=0;
        break;
      case 1:
        Serial.println(" ON / INFORMATIONAL");
        debugLevel=1;
        break;
      case 2:
        Serial.println(" EXTENSIVE");
        debugLevel=2;
        break;
      case 3:
        Serial.println(" VERBOSE");
        debugLevel=3;
        break;
     }  // end case

  } // endif debug level

  if (pub.topic() == MQTT_TOPICt )  // temp source - either device/probe or Internet
  {
    Serial.print("Temp Source MQTT received, and level: " + payload);
     switch (atoi(payload.c_str())) {
       case 0:
         Serial.println(" DS1280B");
         sourceTemp = 0;
         break;
       case 1:
         Serial.println(" OpenWeather");
         sourceTemp = 1;
         break;
       case 2:
         Serial.println(" BME280");
         sourceTemp = 2;
         break;
      }  // end case
  } // endif Temp Source message

  if (pub.topic() == MQTT_TOPICw )  // wind source - either device/probe or Internet
  {
    Serial.print("Wind Source MQTT received, and level: " + payload);
     switch (atoi(payload.c_str())) {
       case 0:  // David Anemometer
         Serial.println(" Davis Anemometer");
         sourceWind = 0;
         break;
       case 1:
         Serial.println(" OpenWeather");
         sourceWind = 1;
         break;
      }  // end case
  } // endif Wind Source message

  if (pub.topic() == MQTT_TOPICp )  // pressure source - either device/probe or Internet
  {
    Serial.print("Pressure Source MQTT received, and level: " + payload);
     switch (atoi(payload.c_str())) {
       case 0:  // Adafruit_BME280
         Serial.println(" BME280");
         sourcePressure = 0;
         break;
       case 1:
         Serial.println(" OpenWeather");
         sourcePressure = 1;
         break;
      }  // end case
  } // endif Pressure Source message

  if (debugLevel > 1) sendMQTTmsg(MQTT_TOPIC"/debug","saving config");

  if (debugLevel > 0) Serial.println("saving config");

  DynamicJsonDocument json(1024);
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_pass"] = mqtt_pass;
  json["wind_correction"] = wind_correction;
  json["windguru_uid"] = windguru_uid;
  json["windguru_pass"] = windguru_pass;
  json["windguru_upload"] = windguru_upload;
  json["vaneMaxADC"] = vaneMaxADC;
  json["vaneOffset"] = vaneOffset;
  json["owApiKey"] = owApiKey;
  json["owLat"] = owLat;
  json["owLong"] = owLong;

  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile)
  {
      Serial.println("failed to open config file for writing");
  } // end if !configFile

  serializeJson(json, Serial);
  serializeJson(json, configFile);
  configFile.close();

} // end callback



// Main code to setup and initialise
void setup()
{
        // Setup communication, and basic version information to log
        Serial.begin(115200);
        Serial.println(VERSION);
        Serial.print("\nESP ChipID: ");
        Serial.println(ESP.getChipId(), HEX);

        //read configuration from FS json
        //clean FS, erase config.json in case of damaged file
        //LittleFS.format();
        Serial.println("Mounting FS...");
        if (LittleFS.begin())
        {
          Serial.println("mounted file system");

          if (LittleFS.exists("/config.json"))
          {   //file exists, reading and loading
            Serial.println("reading config file");
            File configFile = LittleFS.open("/config.json", "r");

            if (configFile)
            {
              Serial.println("opened config file");

              DynamicJsonDocument json(1024);
              DeserializationError error = deserializeJson(json, configFile);

              if (error)
              {
                 Serial.print(F("deserializeJson() failed with code "));
                 Serial.println(error.c_str());

              }
              else
              {
                serializeJson(json, Serial);
                Serial.println("\nparsed json");
                strcpy(mqtt_server, json["mqtt_server"]);
                strcpy(mqtt_port, json["mqtt_port"]);
                strcpy(mqtt_user, json["mqtt_user"]);
                strcpy(mqtt_pass, json["mqtt_pass"]);
                strcpy(wind_correction, json["wind_correction"]);
                strcpy(windguru_uid, json["windguru_uid"]);
                strcpy(windguru_pass, json["windguru_pass"]);
                strcpy(windguru_upload, json["windguru_upload"]);
                strcpy(vaneMaxADC, json["vaneMaxADC"]);
                strcpy(vaneOffset, json["vaneOffset"]);
                strcpy(owApiKey, json["owApiKey"]);
                strcpy(owLat, json["owLat"]);
                strcpy(owLong, json["owLong"]);
              }  // if (error)
            }  // if (configFile)

          }
          else
          {  // cannot find config.json
            Serial.println("failed to open config.json file");
          }

        }
        else
        {  // cannot mount file system
          Serial.println("failed to mount FS");
        }
        //end read configuration

        // setup the WiFiManager parameters to be shown in web portal
        // id/name placeholder/prompt default length
        WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 30);
        WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
        WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 20);
        WiFiManagerParameter custom_mqtt_pass("pass", "mqtt password", mqtt_pass, 20);
        WiFiManagerParameter custom_wind_correction("wind_correction", "wind correction 1-999%", wind_correction, 4);
        WiFiManagerParameter custom_windguru_uid("windguru_uid", "windguru station UID", windguru_uid, 30);
        WiFiManagerParameter custom_windguru_pass("windguru_pass", "windguru API pass", windguru_pass, 20);
        WiFiManagerParameter custom_windguru_upload("windguru_upload", "windguru uploads ON / OFF", windguru_upload, 4);
        WiFiManagerParameter custom_vaneMaxADC("vaneMaxADC", "Max ADC value 1-1024", vaneMaxADC, 5);
        WiFiManagerParameter custom_vaneOffset("vaneOffset", "Wind vane offset 0-359", vaneOffset, 4);
        WiFiManagerParameter custom_owApiKey("owApiKey", "OpenWeather API key", owApiKey, 34);
        WiFiManagerParameter custom_owLat("owLat", "Location Latitude", owLat, 10);
        WiFiManagerParameter custom_owLong("owLong", "Location Longitude", owLong, 10);

        // setup LEDs
        pinMode(LED, OUTPUT);
        pinMode(WindSensorPin, INPUT_PULLUP);
        digitalWrite(LED, HIGH);

        // connect inbound MQTT topics and payloads to processing function
        mqttClient.set_callback(callback);

        WiFiManager wifiManager;
        wifiManager.setSaveConfigCallback(saveConfigCallback);   //set config save notify callback
        wifiManager.setTimeout(180); //sets timeout until configuration portal gets turned off

        //add all your parameters here
        wifiManager.addParameter(&custom_mqtt_server);
        wifiManager.addParameter(&custom_mqtt_port);
        wifiManager.addParameter(&custom_mqtt_user);
        wifiManager.addParameter(&custom_mqtt_pass);
        wifiManager.addParameter(&custom_wind_correction);
        wifiManager.addParameter(&custom_windguru_uid);
        wifiManager.addParameter(&custom_windguru_pass);
        wifiManager.addParameter(&custom_windguru_upload);
        wifiManager.addParameter(&custom_vaneMaxADC);
        wifiManager.addParameter(&custom_vaneOffset);
        wifiManager.addParameter(&custom_owApiKey);
        wifiManager.addParameter(&custom_owLat);
        wifiManager.addParameter(&custom_owLong);

        if(!wifiManager.autoConnect(NameAP, PasswordAP))
        {
          Serial.println("failed to connect and hit timeout");
          delay(1000);
          //reset and try again, or maybe put it to deep sleep
          ESP.reset();
          delay(5000);
        }

        //read updated parameters
        strcpy(mqtt_server, custom_mqtt_server.getValue());
        strcpy(mqtt_port, custom_mqtt_port.getValue());
        strcpy(mqtt_user, custom_mqtt_user.getValue());
        strcpy(mqtt_pass, custom_mqtt_pass.getValue());
        strcpy(wind_correction, custom_wind_correction.getValue());
        strcpy(windguru_uid, custom_windguru_uid.getValue());
        strcpy(windguru_pass, custom_windguru_pass.getValue());
        strcpy(windguru_upload, custom_windguru_upload.getValue());
        strcpy(vaneMaxADC, custom_vaneMaxADC.getValue());
        strcpy(vaneOffset, custom_vaneOffset.getValue());
        strcpy(owApiKey, custom_owApiKey.getValue());
        strcpy(owLat, custom_owLat.getValue());
        strcpy(owLong, custom_owLong.getValue());

        //save the custom parameters to FS
        if (shouldSaveConfig)
        {
          Serial.println("saving config");
          DynamicJsonDocument json(1024);
          json["mqtt_server"] = mqtt_server;
          json["mqtt_port"] = mqtt_port;
          json["mqtt_user"] = mqtt_user;
          json["mqtt_pass"] = mqtt_pass;
          json["wind_correction"] = wind_correction;
          json["windguru_uid"] = windguru_uid;
          json["windguru_pass"] = windguru_pass;
          json["windguru_upload"] = windguru_upload;
      	  json["vaneMaxADC"] = vaneMaxADC;
      	  json["vaneOffset"] = vaneOffset;
          json["owApiKey"] = owApiKey;
          json["owLat"] = owLat;
          json["owLong"] = owLong;

          File configFile = LittleFS.open("/config.json", "w");
          if (!configFile)
          {
            Serial.println("failed to open config file for writing");
          }

          serializeJson(json, Serial);
          serializeJson(json, configFile);
          configFile.close();
          //end save parameters

        } // if (shouldSaveConfig)

        Serial.print("\nConnecting to WiFi");
        while ((WiFi.status() != WL_CONNECTED) && wifiRetries --)
        {
          delay(500);
          Serial.print(" .");
        } // end while

        if (WiFi.status() == WL_CONNECTED)
        {
          Serial.println(". Connected");
          Serial.print("IP Address is: "); Serial.println(WiFi.localIP());
          Serial.print("macAddress is: "); Serial.println(WiFi.macAddress());
          Serial.print("Connecting to ");Serial.print(mqtt_server);Serial.print(" Broker..");
          delay(500);
          mqttClient.set_server(mqtt_server, atoi(mqtt_port));

          while (!mqttClient.connect(MQTT::Connect(String(ESP.getChipId(), HEX)).set_keepalive(90).set_auth(mqtt_user, mqtt_pass)) && wifiRetries --)
          {
            Serial.print(".");
            delay(1000);
          }  // end while

          if (mqttClient.connected())
          {
            Serial.println(". MQTT connected");
            sendMQTTmsg(MQTT_TOPIC"/debug", "Compiled: " __DATE__ " / " __TIME__);
            //mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/debug", "Compiled: " __DATE__ " / " __TIME__).set_retain().set_qos(1));

            //  Subscribe to topics for inbound messages
            mqttClient.subscribe(MQTT_TOPIC );
            mqttClient.subscribe(MQTT_TOPICc );    // wind correction for speed calibration
            mqttClient.subscribe(MQTT_TOPICv );    // vane offset for direction calibration
            mqttClient.subscribe(MQTT_TOPICw );    // wind source - either device/probe or Internet
            mqttClient.subscribe(MQTT_TOPICt );    // temp source - either device/probe or Internet
            mqttClient.subscribe(MQTT_TOPICp );    // pressure source - either device/probe or Internet
            mqttClient.subscribe(MQTT_TOPICu );    // upload readings to WindGuru - on or off
            mqttClient.subscribe(MQTT_TOPICf );    // flash Arduino with new code.
            mqttClient.subscribe(MQTT_TOPICr );    // reset / reboot Arduino
            mqttClient.subscribe(MQTT_TOPICd );    // debugging level sent to MQTT

          } // mqtt.connected()
          else
            Serial.println(". MQTT NOT connected");
        } // wifi.stat() == connected

        else
        {
            Serial.println(" WiFi CONNECTION FAILED!");
            Serial.println();
        } // end if wifi.status() != connected

        setupSensors();
}

// procedure to ensure wifi and MQTT broker still connected
void checkConnection()
{
  if (WiFi.status() == WL_CONNECTED)  {
    if (mqttClient.connected()) {
      Serial.println("mqtt broker connection . . . . . . . . . . OK");
    }
    else {
      Serial.println("mqtt broker connection . . . . . . . . . . LOST");
      if (requestRestart < 1) requestRestart = 60;
    }
  }
  else {
    Serial.println("WiFi connection . . . . . . . . . . LOST");
    if (requestRestart < 1) requestRestart = 5;
  }
}

// check to see if restart / reboot required
void checkRestartStatus() {
  if (requestRestart == 1) {
    //blinkLED(LED, 400, 4);
    ESP.restart();
    delay(500);
  }
}

// NOT SURE - CHECK THIS OUT
void ICACHE_RAM_ATTR windcount() {
  if((long)(micros() - contactBounceTime) > 15 ) {
    rotations++;
    contactBounceTime = millis();
    digitalWrite(LED, !digitalRead(LED));
  }
}

// procedure to initialise the probes and sensors attached to Arduino
void setupSensors()
{
  //Setup BME Temp / Humidity / Pressure sensor
  Serial.println("\nSetting up BME sensors");
  bme.begin(0x76);

  Serial.print("BME T="); Serial.print(bme.readTemperature()); Serial.print("°C, ");
  Serial.print("P="); Serial.print(bme.readPressure() / 100.0F); Serial.print("hPa, ");
  Serial.print("H="); Serial.print(bme.readHumidity()); Serial.println("%");

  // Need code for DS1280B probes
  // Goes here

  // Wind speed sampling & timer setup
  timerCount = millis();
  wgUpdateCounter = millis();
  rotations = 0;    // Set Rotations to 0 ready for calculations
  pinMode(WindSensorPin, INPUT);

  //call windcount() when interupt on WindSensorPin fires.  Increaments 'rotation' counter.
  attachInterrupt(WindSensorPin, windcount, FALLING);

  GetInternetWeather(owApiKey, owLat, owLong);     // Get Internet weather from OpenWeather API

}


// procedure to get the temperature readings, from source device/probe
float getTemperature()
{
  float _temperature;
  switch (sourceTemp) {
    case 0: // DS1280B
      //Serial.println(" DS1280B");
      //sourceTemp = 0;
      _temperature = owTemp;
      Serial.println("_temperature 0 :"+String(_temperature));
      break;
    case 1: // OpenWeather
      _temperature = owTemp;
      Serial.println("_temperature 1 :"+String(_temperature));
      break;
    case 2: // BME280
      _temperature = bme.readTemperature();
      Serial.println("_temperature 2 :"+String(_temperature));
      if (_temperature == 0)
      {
        _temperature = owTemp;  // use backup
        Serial.println("_temperature backup :"+String(_temperature));
        sourceTemp = 1;
      }
      break;
   }  // end case
   return(_temperature);
}

// procedure to get the pressure readings, from source device/probe
float getPressure()
{
  float _pressure;
  switch (sourcePressure) {
    case 0: // BME280
      _pressure = bme.readPressure() / 100.0F;
      Serial.println("_pressure 0 :"+String(_pressure));
      if (_pressure < 1)  // suspect reading, so switch to backup
      {
        _pressure = owPressure; // use backup
        Serial.println("_pressure backup :"+String(_pressure));
        sourcePressure = 1;
      }
      break;
    case 1: // OpenWeather
      _pressure = owPressure;
      Serial.println("_pressure 1 :"+String(_pressure));
      break;
   }  // end case
   return(_pressure);
}

// procedure to get the humidity readings, from source device/probe
float getHumidity()
{
  float _humidity;
  switch (sourceHumidity) {
    case 0: // BME280
      _humidity = bme.readHumidity();
      Serial.println("_humidity 0 :"+String(_humidity));
      if (_humidity < 1)
      {
        _humidity = owHumidity;  // use backup
        Serial.println("_humidity backup :"+String(_humidity));
        sourceHumidity = 1;
      }
      break;
    case 1: // OpenWeather
      _humidity = owHumidity;
      Serial.println("_humidity 1 :"+String(_humidity));
      break;
   }  // end case
   return(_humidity);
}

// Get Wind Direction
int getWindDirection()
{
  int _direction, _calDirection;

  switch (sourceDir) {
    case 0: // Davis Wind vane
      _direction = map(analogRead(WindVanePin), 0, 1023, 0, 359);
      Serial.println("_direction 0 :"+String(_direction));
      _calDirection = _direction + atoi(vaneOffset);

      if(_calDirection > 360)  _calDirection = _calDirection - 360;
      if(_calDirection < 0) _calDirection = _calDirection + 360;
      break;

    case 1: // OpenWeather
      Serial.println("_direction 1 :"+String(owDirection));
      _calDirection = owDirection;
  } // end case

  return(_calDirection);
}

// Validate wind speed readings, and switch to backup if needed
void getWindSpeed()
{
  switch (sourceWind) {
    case 0: //  Davis anemometer
      Serial.println("sourceWind 0 :"+String(windAvg));
      if (windMin == windMax) // suspect reading, so use OpenWeather instead
      {
        windMin = owWindMin;
        windAvg = owWindAvg;
        windMax = owWindMax;
        Serial.println("sourceWind 0 (backup) :"+String(windAvg));
      }
      break;
    case 1: //  OpenWeather
      Serial.println("sourceWind 1 :"+String(windAvg));
      windMin = owWindMin;
      windAvg = owWindAvg;
      windMax = owWindMax;
      break;
  }
  if (debugLevel > 0)
  {
    Serial.println("Wind=" + String(windMin,2) + " << " + String(windAvg,2) + " >> " + String(windMax,2) + " knots");
    if (debugLevel > 1)  sendMQTTmsg(MQTT_TOPIC"/debug", "Wind=" + String(windMin,2) + " << " + String(windAvg,2) + " >> " + String(windMax,2) + " knots");
  }
}

// procedure to read values from attached devices / probes
void getSensors()
{
  GetInternetWeather(owApiKey, owLat, owLong);     // Get Internet weather from OpenWeather API
  getWindSpeed();
  temperature = getTemperature();
  humidity = getHumidity();
  pressure = getPressure();
  direction = getWindDirection();
  mslp = (((pressure * 100.0)/pow((1-((float)(ELEVATION))/44330), 5.255))/100.0);
}


// Three primary purposes of timedTasks functions
// (a) Get the wind readings and (b) determine if it is time to sample sensors and (c) send report
void timedTasks()
{
  float _elapsedMS, _windFactor, _correction;
  String _logMsg;

  if ((millis() > timerCount + (davisUpdateFreq * 1000)) || (millis() < timerCount ))
  {
    // time to calculate wind speed from the rotations in the davisUpdateFreq period
    _elapsedMS = millis() - timerCount;        // How much time has elapsed since last reading
    timerCount = millis();

    // convert to mp/h using the formula V=P(2.25/T)
    _windFactor = 2250 / _elapsedMS;
    _correction = (float) atoi(wind_correction) / 100.0;       // Convert % wind_correction into fraction
    windSpeed = getKnots(rotations * _windFactor * _correction);
    setWindRange(windSpeed); // calculate min and max variance in wind
    _logMsg = "R=" + String(rotations);
    rotations = 0; // Reset count for next sample
    if (debugLevel > 0)
    {
      _logMsg = _logMsg + " in " + String((_elapsedMS / 1000), 2) + " secs " + " W=" + String(windSpeed,2) + "knots @ " + String((_correction*100),2) + "%";
      Serial.println(_logMsg);
      if (debugLevel > 1)  sendMQTTmsg(MQTT_TOPIC"/debug", _logMsg);
    }

    // flash LED
    if(digitalRead(LED) == LOW)
    {
       blinkLED(LED, 100, 1);
     } else {
       blinkLED(LED, 100, 1);
       digitalWrite(LED, LOW);
    } // endif digitalRead

  }   // endif millis > sampling period

  if ((millis() > wgUpdateCounter + (wgUpdateFreq * 1000)) || (millis() < wgUpdateCounter ))
  {
    // time to send updates to WindGuru
    wgUpdateCounter = millis();
    windAvg = windTot / ( wgUpdateFreq / davisUpdateFreq );
    Serial.println("windAvg="+String(windAvg,1)+" from "+String(windTot)+" / ("+String(wgUpdateFreq)+" / "+String(davisUpdateFreq)+")");

    getSensors();
    SendToWindguru();

    windMin = 0;  windAvg = 0;  windMax = 0; windTot = 0;

  }   // endif millis > sampling period

}

// Set the min / max range from the wind speed readings in Knots
void setWindRange(float _windSpeed)
{
  windAvg = _windSpeed;
  if (_windSpeed > windMax) windMax = _windSpeed;   // Knots
  if (_windSpeed < windMin) windMin = _windSpeed;   // Knots
  windTot = windTot + _windSpeed;                   // Cumulative readings in Knots
}

// Flash LED
void blinkLED(int pin, int duration, int n) {
  for(int i=0; i<n; i++)  {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
    delay(duration);
  }
}

// procedure to download latest code OTA via Internet, and restart Arduino
void ota_update()
{
  Serial.println("Loading OTA update");
  ESPhttpUpdate.onStart(ota_update_started);
  ESPhttpUpdate.onEnd(ota_update_finished);
  ESPhttpUpdate.onProgress(ota_update_progress);
  ESPhttpUpdate.onError(ota_update_error);

  t_httpUpdate_return ret = ESPhttpUpdate.update(otaUpdateBinary,"",secureFingerprint);

  switch (ret)
  {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  } // end switch
} // end ota_update

// series of update functions for the OTA process
void ota_update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
} // end ota_update_started

void ota_update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}  // end ota_update_finished

void ota_update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}  // end ota_update_progress

void ota_update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}  // end series of OTA updates


// Function to convert MPH to Knots
float getKnots(float speed) {
return speed * 0.868976;
}  // end getKnots

// Function to convert Meters per second to Knots
float getKnotsFromMS(float speed) {
return speed * 1.94384;
} // end getKnotsFromMS

// Determine if wind speed is good in range of knots from 0 to 150 kts.  Note 0 could be suspicuous
bool validateWind(float functWind) {
  if (functWind == 0 || functWind < 0 || functWind > 150) return false; else return true;
} // end validateWind


// Module to get key weather metrics from the OpenWeather.com API, to use for backup readings.
void GetInternetWeather(String apiKey, String Latitude, String Longitude)
  // Limited to 1,000 api calls per day, so this will be updated every 10 mins at most.
  // Sends API key and Latitude & Longitude to API service, and then processes JSON response.
  {
  WiFiClient client;
  HTTPClient http; //must be declared after WiFiClient for correct destruction order, because used by http.begin(client,...)
  String getURL, baseURL;
  const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(6) + JSON_OBJECT_SIZE(13) + 280;

  if (owUpdateCounter < owUpdateFreq)
    { // Only update every few minutes, as the OpenWeather current readings only change every 5-15 minutes
      Serial.print("Skipping internet Weather download.  Next update in ");
      Serial.print(owUpdateFreq - owUpdateCounter);
      Serial.println(" minutes");
      owUpdateCounter = owUpdateCounter + 1;
    }
    else
    { // time to get update from openweathermap
    owUpdateCounter = 1;
    DynamicJsonDocument doc(capacity);
    baseURL = "http://api.openweathermap.org/data/2.5/weather";
    getURL = baseURL + "?lat=" + String(Latitude) + "&lon=" + String(Longitude) + "&appid=" + String(apiKey) + "&units=metric";

    if (WiFi.status() == WL_CONNECTED)
      {          //Check WiFi connection status
        Serial.print("Connecting to OpenWeather... ");
        http.begin(client, getURL);               //Specify request destination
        int httpCode = http.GET();                //Send the request
        if (httpCode > 0)
        {   //Check the returning code - http result OK
            String payload = http.getString();    //Get the request response payload
            DeserializationError error = deserializeJson(doc, payload);
           if (error)
             {  // JSON failed
               Serial.print(F("deserializeJson() failed: "));
               Serial.println(error.c_str());
             }
            else
            { // JSON ok
               Serial.println("Connected");
               JsonObject main = doc["main"];
               float _main_temp = main["temp"]; // 8.23 degrees C
               int _main_pressure = main["pressure"];             // 1014 millibars
               int _main_humidity = main["humidity"];             // 87 percentage

               JsonObject wind = doc["wind"];
               float _wind_speed = getKnotsFromMS(wind["speed"]);   // 8.7 - units are meters per second
               float _wind_gust = getKnotsFromMS(wind["gust"]);     // 14.9 - units are meters per second
               int _wind_deg = wind["deg"];                         // 40 degrees

               Serial.print("W="); Serial.print(_wind_speed);
               Serial.print(" ("); Serial.print(_wind_gust); Serial.print(") knots");
               Serial.print(" @ "); Serial.print(_wind_deg); Serial.print("°");
               Serial.print(", T="); Serial.print(_main_temp); Serial.print("°C, ");
               Serial.print("P="); Serial.print(_main_pressure); Serial.print("hPa, ");
               Serial.print("H="); Serial.print(_main_humidity); Serial.println("%");

               //Set globals to hold values from open weather
               owTemp = _main_temp;
               owPressure = _main_pressure;
               owHumidity = _main_humidity;
               owDirection = _wind_deg;
               owWindAvg = _wind_speed;
               if (_wind_gust > _wind_speed) owWindMax = _wind_gust; else owWindMax = _wind_speed;
               owWindMin = 0.0;

               /// send ow weather spot reading to MQTT
               if (debugLevel > 0)
                  sendMQTTmsg(MQTT_TOPIC"/ow","{\"temp\": "+String(owTemp, 1)+", "+"\"pressure\": "+String(owPressure, 1)+", "+"\"humidity\": "+String(owHumidity, 1)+", "+"\"wind_speed\": "+String(owWindAvg, 1)+", "+"\"wind_gust\": "+String(owWindMax, 1)+", "+"\"wind_dir\": "+String(owDirection) + "}");

            } // JSON parsed successfully
          http.end();   //Close connection
      } // end http return code
      else  // no wifi-connection
         Serial.println("wi-fi connection failed");
      } // end wifi CONNECTED
    }
} // end GetInternetWeather


// send info to windguru.cz
bool SendToWindguru()
{
  //char message_buff[60];
  //String pubString;
  WiFiClient client;
  HTTPClient http; //must be declared after WiFiClient for correct destruction order, because used by http.begin(client,...)
  String getData, link;
  unsigned long time;

  if (WiFi.status() == WL_CONNECTED)
    { //Check WiFi connection status
      link = "http://www.windguru.cz/upload/api.php?";
      time = millis();

      //--------------------------md5------------------------------------------------
      MD5Builder md5;
      md5.begin();
      md5.add(String(time) + String(windguru_uid) + String(windguru_pass));
      md5.calculate();
      //--------------------------md5------------------------------------------------

      //wind speed during interval (knots)
      //getData = "uid=" + String(windguru_uid) + "&salt=" + String(time) + "&hash=" + md5.toString() + "&wind_min=" + String(WindMin * kKnots, 2) + "&wind_avg=" + String(WindAvr * kKnots/meterWind, 2) + "&wind_max=" + String(WindMax * kKnots, 2);
      getData = "uid=" + String(windguru_uid) + "&salt=" + String(time) + "&hash=" + md5.toString();

      //wind_speeds - avg, min, max in Knots
      getData = getData + "&wind_avg=" + String(windAvg, 1);
      if (windMax != windAvg) getData = getData + "&wind_max=" + String(windMax, 1);
      if (windMin != windAvg) getData = getData + "&wind_min=" + String(windMin, 1);

      //wind_direction; wind direction as degrees (0 = north, 90 east etc...)
      getData = getData + "&wind_direction=" + String(direction);

      //temp, pressure, humidity
      if (!isnan(temperature)) getData = getData + "&temperature=" + String(temperature,1);
      if (!isnan(humidity)) getData = getData + "&rh=" + String(humidity,1);
      if (!isnan(mslp)) getData = getData + "&mslp=" + String(mslp,1);

      if (windguru_upload[0] == 'O' && windguru_upload[1] == 'N')  // send to WindGuru ON
      {
        Serial.println("Uploading : " + link + getData);
        http.begin(client, link + getData);    //Specify request destination

        int httpCode = http.GET();             //Send the request
        if (httpCode > 0)
          {                    //Check the returning code
            String payload = http.getString();   //Get the request response payload
            Serial.println(payload);             //Print the response payload
            if (payload != "OK")  sendMQTTmsg(MQTT_TOPIC"/debug",payload);
          }
        http.end();   //Close connection
      } // windguru_upload != "ON"
      else
      {
        Serial.println("Skipping Upload : " + link + getData);
      }  // end else if.
    }
    else
    {
        Serial.println("wi-fi connection failed");
        return false; // fail;
    }

    // Now send weather metrics to MQTT
    sendMQTTmsg(MQTT_TOPIC"/temp", "{\"temp\": "+String(temperature, 1)+", "+"\"humidity\": "+String(humidity, 1) + ", "+"\"mslp\": "+String(mslp, 1) + "}");
    sendMQTTmsg(MQTT_TOPIC"/wind", "{\"wind_min\": "+String(windMin, 1)+", "+"\"wind_avg\": "+String(windAvg, 1)+", "+"\"wind_max\": "+String(windMax, 1)+", "+"\"wind_dir\": "+String(direction) + "}");

  return true; //done
}

// Send message to MQTT topic and deliver payload
void sendMQTTmsg(String _topic, String _payload)
{
  if (mqttClient.connected())
  {
    mqttClient.publish(MQTT::Publish(_topic, _payload).set_retain().set_qos(1));
  } // endif CONNECTED
  else
  {
    if (debugLevel > 1)   // 0=off; 1=informational; 2;=extensive/mqtt; 3=verbose/mqtt
      Serial.println("Not connected MQTT "+_topic+" : "+_payload);
  }
}

// main code loop
void loop()
{
  // check for incomming MQTT messages
  // check timer based readings - is it time to sample and send report?
  // check increase restart command issued
  mqttClient.loop();
  timedTasks();
  checkRestartStatus();
}
