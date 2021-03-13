#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <TZ.h>
#include <coredecls.h> // required for settimeofday_cb() (NTP sync callback)
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

//////////////////////////////////////////////////////////////////////////
// FAN Timer - Driven by light on/of toggling.
// -----------------------------------------------------------------------
// This program uses an ESP8266, with a relay board on D1, and an I2C 
// light sensor type TSL2561 on pins D3 (sda) and D4 (scl).
// -----------------------------------------------------------------------
// Normal operation:
//
// - If the light goes ON, the FAN will be OFF, and will be started
//   after 5 minutes.
// - If the light goes OFF, the FAN will start, and run for 30 minutes.
//
// Special operation:
// Triggered by toggling the light once extra quickly;
//
// - Toggle Light "On - Off - On"; the FAN will start running for 30
//   minutes.
// - Toggle light "Off - On - Off"; the FAN will NOT run.
//
// The "quick" toggle must happen within 2 seconds. If you toggle more
// than the mentioned 3 times, then the last 3 states will be honoured.
// -----------------------------------------------------------------------
// Note: this program uses WiFi, which is totally NOT needed, but nice
// to use for OTA (Over The Air) firmware updates, and debugging.
// -----------------------------------------------------------------------
// March 6. 2021, Thijs Kaper.
//////////////////////////////////////////////////////////////////////////


///////////////////////// start of settings //////////////////////////////

// WiFi Network settings SSID and Password.
const char* ssid = "YOUR_WIFI_NETWORK";
const char* pass = "YOUR_WIFI_PASSWORD";

#define OTA_PASSWORD "fancontrol"
#define OTA_HOSTNAME "FAN_Control"

// FAST_TOGGLE_SECONDS is the time in which a repeated light toggle will have impact on action.
// If you wait longer than this time, a toggle is counted as "inital" change, otherwise as "repeated" toggle.
#define FAST_TOGGLE_SECONDS 2

// The time the FAN will keep running.
#define FAN_RUN_SECONDS (60*30)

// The delay time before the FAN will start.
#define DELAYED_START_SECONDS (60*5)

// The timezone to sync the date/time to, using NTP. For timezone to use, see TZ.h.
// For this application totally not needed, just a nice-to-have.
#define MY_TZ TZ_Europe_Amsterdam

// NTP server list to use for syncing time.
#define NTP_SERVERS "0.nl.pool.ntp.org", "1.nl.pool.ntp.org", "2.nl.pool.ntp.org"

// The PIN's from the ESP to use for the light sensor "TSL2561".
#define SDA_PIN D3
#define SCL_PIN D4

// The (initial) compare value to check if light is on or off.
// The system will find the min and max lux values (after reboot), and set trigger to 2/3 between min and max.
// Note: it is best to change this value to whatever comes out of that automatic calculation, to have a sane default.
#define LUX_LIGHT_DETECTION 95
// Set next one to false, if auto-detect does do more harm then help out.
#define ENABLE_AUTO_LUX_DETECTION_MIN_MAX true
// Min/Max difference must be met, before automatic level determination kicks in.
#define MIN_LUX_DIFFERENCE 50
#define MAX_LUX_DIFFERENCE 1000

///////////////////////// end of settings //////////////////////////////

ESP8266WebServer server(80);

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
TwoWire MyWire = TwoWire();

boolean relayState = true;
int fanRunSeconds = 0;
int startFanAfterSeconds = 0;

float luxLightOnDetection = LUX_LIGHT_DETECTION;
float currentLux = 0;
float minLux = 65535;
float maxLux = 0;

boolean previousLightStatus = false;
time_t lastLightOnTimeStamp = time(nullptr);
time_t lastLightOffTimeStamp = time(nullptr);

boolean timeIsSet = false;
time_t lastNtpSet = time(nullptr);

time_t lastTime = time(nullptr);

char statusMessage[1024];

// Turn relay on/off
void relay(boolean enabled) {
  if (enabled && !relayState) {
    Serial.println("-- relay switch ON --");
  } else if (!enabled && relayState) {
    Serial.println("-- relay switch OFF --");
  }
  pinMode(D1, OUTPUT);
  digitalWrite(D1, enabled);
  relayState = enabled;
}

// Initialize light sensor
void initSensor() {
  // Set custom pins for I2C
  MyWire.begin(SDA_PIN, SCL_PIN);
 
  /* Initialise the sensor */
  if(!tsl.begin(&MyWire)) {
    Serial.print("No TSL2561 detected");
    delay(2000);
    ESP.restart();
  }
 
  sensor_t sensor;
  tsl.getSensor(&sensor);
  
  Serial.println("\n------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------\n");
  delay(3000);

   /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
}

// Record the NPT set time
void timeUpdated() {
  timeIsSet = true;
  lastNtpSet = time(nullptr);
  Serial.print("NTP Updated: "); Serial.println(ctime(&lastNtpSet));
}

// Initialize WiFi / OTA / WebServer
void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname(OTA_HOSTNAME);
  WiFi.begin(ssid, pass);
  Serial.println("\n........");
  Serial.println("Find WIFI.");
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("ONLINE\n");

  // implement NTP update of timekeeping (with automatic hourly updates)
  configTime(MY_TZ, NTP_SERVERS);

  // callback, when NTP changes the time
  settimeofday_cb(timeUpdated);

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}
  
// Global setup
void setup() {
  Serial.begin(115200);
  Serial.println("\n\nStart\n\n");
  setupWifi();
  initSensor();
  relay(false);
}

// Global processing loop
void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  
  // Check if light is on or off, and on CHANGE record the new status timestamp.
  // Also check if the light was toggled an extra time fast, to detect special case.
  boolean isLight = isLightOn();
  handleLightChanges(isLight);

  time_t current = time(nullptr); // seconds
  
  // One second has passed?
  if (current != lastTime) {
    lastTime = current;
    handleTimerCounters();

    // Setup status message (both used for web server, and for serial debug)
    sprintf(statusMessage, "Time: %s, NtpSync: %1.1f min ago, current lux: %1.1f, compare >= %1.1f lux?, light: %s, startFanAfterSeconds: %d, fanRunSeconds: %d, fan: %s",
            my_r_trim(ctime(&current)), (current - lastNtpSet) / 60.0,
            currentLux, luxLightOnDetection, (isLight ? "ON" : "OFF"),
            startFanAfterSeconds, fanRunSeconds, (relayState ? "ON" : "OFF")
    );
    Serial.println(statusMessage);
  }

  delay(50);
}

// If light goes on/off, memorize the moment, and handle action
void handleLightChanges(boolean isLight) {
  if (previousLightStatus == isLight) {
    // No change? No action...
    return;
  }
  
  if (isLight) {
    // Light is ON
    lastLightOnTimeStamp = time(nullptr);
    if ((lastLightOnTimeStamp - lastLightOffTimeStamp) <= FAST_TOGGLE_SECONDS) {
      // fast toggle
      onOffOn();
    } else {
      // slow toggle
      offOn();
    }
  } else {
    // Light is OFF
    lastLightOffTimeStamp = time(nullptr);
    if ((lastLightOffTimeStamp - lastLightOnTimeStamp) <= FAST_TOGGLE_SECONDS) {
      // fast toggle
      offOnOff();
    } else {
      // slow toggle
      onOff();
    }
  }
  previousLightStatus = isLight;
}

// If counters are non-zero, count down. When going zero, do some action.
void handleTimerCounters() {
    if (fanRunSeconds > 0) {
      if((--fanRunSeconds) == 0) {
        relay(false);
      } else {
        relay(true);
      }
    }

    if (startFanAfterSeconds > 0) {
      if((--startFanAfterSeconds) == 0) {
        // start fan for 30 minutes
        fanRunSeconds = FAN_RUN_SECONDS;
        relay(true);
      }
    }
}

//////////////// start of toggle logic //////////////////

// "fast" double toggle
void onOffOn() {
  Serial.println("Light toggled: ON-OFF-ON = start fan for 30 minutes");
  // start fan for 30 minutes
  fanRunSeconds = FAN_RUN_SECONDS;
  startFanAfterSeconds = 0;
  relay(true);
}

// "slow" single toggle
void offOn() {
  Serial.println("Light toggled: WAIT-OFF-ON = stop fan + start fan after 5 minutes for 30 minutes");
  // stop fan
  // start fan after 5 minutes for 30 minutes
  fanRunSeconds = 0;
  startFanAfterSeconds = DELAYED_START_SECONDS;
  relay(false);
}

// "fast" double toggle
void offOnOff() {
  Serial.println("Light toggled: OFF-ON-OFF = stop fan, no start");
  // stop fan
  fanRunSeconds = 0;
  startFanAfterSeconds = 0;
  relay(false);
}

// "slow" single toggle
void onOff() {
  Serial.println("Light toggled: WAIT-ON-OFF = start fan for 30 minutes");
  // start fan for 30 minutes
  fanRunSeconds = FAN_RUN_SECONDS;
  startFanAfterSeconds = 0;
  relay(true);
}

//////////////// end of toggle logic //////////////////

char * my_r_trim(char * input) {
  char * ptr = input + strlen(input) - 1;
  // shop off last char if blank
  while (ptr > input && isspace(*ptr)) {
    *ptr-- = 0;
  }
  return input;  
}

// Read light sensor, and detemine max/min and new switch level if needed.
boolean isLightOn() {
  /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);
  
  if (!event.light) {
    Serial.println("Sensor overload - asume ligth off");
    return false;    
  }
  currentLux = event.light;

  if(currentLux > maxLux) {
    maxLux = currentLux;
  }
  if(currentLux < minLux) {
    minLux = currentLux;
  }
  if (ENABLE_AUTO_LUX_DETECTION_MIN_MAX && (maxLux - minLux) > MIN_LUX_DIFFERENCE && (maxLux - minLux) < MAX_LUX_DIFFERENCE) {
    // Set detection to 2/3 between min and max
    luxLightOnDetection = ((maxLux - minLux) * 2 / 3) + minLux;
  }
  
  return (event.light >= luxLightOnDetection);
}

// Render web status page on root, and handle fan override parameter.
void handleRoot() {
  if (server.arg("fan") == "on") {
    // start fan for 30 minutes
    fanRunSeconds = FAN_RUN_SECONDS;
    startFanAfterSeconds = 0;
    relay(true);
    server.sendHeader("Location", String("/"), true);
    server.send ( 302, "text/plain", "");
    return;
  }
  if (server.arg("fan") == "off") {
    // stop fan
    fanRunSeconds = 0;
    startFanAfterSeconds = 0;
    relay(false);
    server.sendHeader("Location", String("/"), true);
    server.send ( 302, "text/plain", "");
    return;
  }
  char buffer[2048];
  snprintf(buffer, 2048,
    "<head><title>fan control</title><meta http-equiv='refresh' content='1'></head>\n\
    <body>\n\
    %s<br>\n\
    <a href='?fan=on'>ON</a> | <a href='?fan=off'>OFF</a>\n\
    </body>\n",
    statusMessage
  );

  server.send(200, "text/html", buffer);
}

// Render 404
void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}
