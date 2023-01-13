/*
 * TODO:
 * - pinA (temperature) and lampPin need not to be the same
 *   (first one e.g. could also activate a fan instead of deactivating the lamp)
 * - add square ware support for stroboscope
 * - re-add http API
 */

// Enable support for temperature monitoring with dallas ds18b20 sensors.
// If the fixture gets too hot, counter mesures can be taken such as switching
// it off or activating a fan.
// This adds approx. 6k to program storage space.
#define WITH_TEMP_MONITORING

// Enable WiFi configuration at runtime.
// This adds ~22k to program storage space.
#define WITH_WIFI_MANAGER

// Enable (Arduino IDE) over the air upload.
// This adds ~26k to program storage space.
#define WITH_OTA

// Enable debug logging via UDP.
// This adds ~300 bytes to program storage space.
#define WITH_UDP_LOGGING

// Enable a web server for configuration and status report.
#define WITH_WEB_SERVER

// Enable outputting a square wave on adefined pin to control an anlog stroboscope.
#define WITH_STROBOSCOPE

/* ==================================================================== */

#define WITH_DEBUG_TEMP

/* ==================================================================== */

#include <ArtnetWifi.h> // https://github.com/rstephan/ArtnetWifi
#include <EEPROM.h>

#ifdef WITH_OTA
#include <ArduinoOTA.h>
#endif
#ifdef WITH_UDP_LOGGING
#include <WiFiUdp.h>
#endif
#ifdef WITH_TEMP_MONITORING
#include <DallasTemperature.h>
#include <OneWire.h>
#endif
#ifdef WITH_WIFI_MANAGER
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#endif
#ifdef WITH_WEB_SERVER
#include <ESP8266WebServer.h>
#endif

/**************************************
 * Configuration
 */
#define VERSION "V.1.0.5"
#define BUILD   "Build " __DATE__ " " __TIME__

// Used for checking if eeprom data is valid.
// Whenever you change someting in the config struct, make sure to change this
// magic at least temporarily or make sure to clear eeprom before next upload.
#define MAGIC 0x4713

String CHIP_ID = String(ESP.getChipId(), 16);

// Everything that needs to go to EEProm
typedef struct { 
  // magic needed to detect if eeprom content valid / written before
  int     magic; 
  // custom name for this device that will appear as access point name
  char    fixture[32+1];
  int16_t universe; 

  // Channel A
  int16_t channelA; 
  int8_t  pinA;
  uint8_t activeA;

  // Channel B
  int16_t channelB; 
  int8_t  pinB;
  uint8_t activeB;

  #ifdef WITH_STROBOSCOPE
  // Strobe channel
  int16_t channelStrobe;
  int8_t  pinStrobe;
  #endif

  #ifdef WITH_TEMP_MONITORING
  // Cooling
  int8_t pinCool;
  uint8_t activeCool;

  // temperature monitoring eabled?
  bool    monitor;  
  int8_t  low;
  int8_t  warn;
  int8_t  high;
  #endif
} Config;

Config config;

void initConfig() {
  config.magic         = MAGIC;

  config.fixture[0]    =    0;
  config.universe      =   -1;  // any universe will match
  
  config.channelA      =  134;
  config.pinA          =   D7;
  config.activeA       = HIGH;

  config.channelB      =   -1;
  config.pinB          =   -1;
  config.activeB       =  LOW;
  
  #ifdef WITH_STROBOSCOPE
  config.channelStrobe =   -1;
  config.pinStrobe     =   -1;
  #endif

  #ifdef WITH_TEMP_MONITORING
  config.pinCool       =   D7; // same as pinA
  config.activeCool    =  LOW; // inverse of activeA

  config.monitor       = true;
  config.low           =   55; // all fine below this temperature
  config.warn          =   56; // start to modulate and warn if this temp/ reached
  config.high          =   58; // switch of (lamp) if this temperature reached.
  #endif
}

void dumpConfig() {
  log(String("Fixture name:   '") + config.fixture + "'"); 
  log(String("DMX universe:   ")  + config.universe); 
  log(String("Channel A:      ")  + config.channelA); 
  log(String("Pin A:          ")  + config.pinA); 
  log(String("Active A:       ")  + config.activeA); 
  log(String("Channel B:      ")  + config.channelB); 
  log(String("Pin B:          ")  + config.pinB); 
  log(String("Active B:       ")  + config.activeB); 
  #ifdef WITH_STROBOSCOPE
  log(String("Strobe channel: ")  + config.channelStrobe); 
  log(String("Strobe pin:     ")  + config.pinStrobe); 
  #endif
  #ifdef WITH_TEMP_MONITORING
  log(String("Cooling pin:    ")  + config.pinCool); 
  log(String("Cooling active: ")  + config.activeCool); 
  log(String("Monitor:        ") + config.monitor); 
  log(String("Low:            ") + config.low); 
  log(String("Warn:           ") + config.warn); 
  log(String("High:           ") + config.high);   
  #endif
}

/**************************************
 * Temperature monitoring
 */
#ifdef WITH_TEMP_MONITORING

#define ONE_WIRE_BUS D4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

const float lowerLimit = 55;
const float warnLimit  = lowerLimit+1;
const float upperLimit = lowerLimit+3;

long lastCheck = millis();
long lastError = 0;

boolean measureFailed = false;
unsigned long lastMeasureSuccess = 0;

float lastTempC = upperLimit; 
boolean temperaturesRequested = false;

#endif

#ifndef WITH_WIFI_MANAGER
/**************************************
 * Static WiFi setup without WiFiManager
 */
String STA_SSID("your_ssid");
String STA_PASS("your_pass");  
#endif

IPAddress AP_ADDR(192, 168, 47, 12);
String AP_SSID("✅AP_ARTNET_RELAY");
String AP_PASS("47111174");

/**************************************
 * OTA (over the air) programming credentials
 */
#ifdef WITH_OTA
String OTA_NAME = "OTA_DMX_RELAY";
String OTA_PASS = "47111174"; // might differ from AP_PASS
#endif

/**************************************
 * WiFiManager
 */
#ifdef WITH_WIFI_MANAGER

WiFiManager wm;

struct {
  WiFiManagerParameter fixture  = WiFiManagerParameter("fixture",       "Fixture name", AP_SSID.c_str(), 32);
  WiFiManagerParameter universe = WiFiManagerParameter("universe",      "DMX universe", "-1", 2);
  
  WiFiManagerParameter channelA = WiFiManagerParameter("channelA",  "DMX channel A (0 to 512)", "134", 3);
  WiFiManagerParameter pinA     = WiFiManagerParameter("pinA",      "GPIO pin A (-1 to 16)",     "13", 2); // D7/13
  WiFiManagerParameter activeA  = WiFiManagerParameter("activeA",   "Active state A (0/1)",      "1",  1); // active high
  
  WiFiManagerParameter channelB = WiFiManagerParameter("channelB",  "DMX channel B (0 to 512)",   "0", 3); // none
  WiFiManagerParameter pinB     = WiFiManagerParameter("pinB",      "GPIO pin B (-1 to 16)",     "-1", 2); // none
  WiFiManagerParameter activeB  = WiFiManagerParameter("activeB",   "Active state B (0/1)",       "0", 1); // active low

  WiFiManagerParameter channelStrobe = WiFiManagerParameter("channelS",  "Strobe channel (0 to 512)",  "0", 3); // none
  WiFiManagerParameter pinStrobe     = WiFiManagerParameter("pinS",      "Strobe pin (-1 to 16)",     "-1", 2); // none

  WiFiManagerParameter pinCool      = WiFiManagerParameter("pinCool",    "GPIO cooling (0 to 16)",     "13", 3); // same as pin A ->
  WiFiManagerParameter activeCool   = WiFiManagerParameter("activeCool", "Cooling active state (0/1)", "0", 1); // active low -> load A off

  #ifdef WITH_TEMP_MONITORING
  WiFiManagerParameter monitor = WiFiManagerParameter("monitor", "Monitor temperature (0/1)", "0", 1);
  WiFiManagerParameter low     = WiFiManagerParameter("low",     "Low °C (-126 to 127)",  "55", 3); // dallas sensors report > -127 C
  WiFiManagerParameter warn    = WiFiManagerParameter("warn",    "Warn °C (-126 to 127)", "56", 3);
  WiFiManagerParameter high    = WiFiManagerParameter("high",    "High °C (-126 to 127)", "58", 3);
  #endif
} custom;

#endif

#ifdef WITH_WEB_SERVER
ESP8266WebServer server(80);
#endif

/**************************************
 * UDP logging
 */
#ifdef WITH_UDP_LOGGING
#define UDP_PORT 4220
WiFiUDP UDP;
boolean doLogUDP = true;
unsigned long udpLoggingEnds = 10*60*1000; // 10 minutes
#endif

/**************************************
 * DMX and artnet stull
 */
ArtnetWifi artnet;

struct {

  int lastSequence = -1;
  unsigned lastSequenceTimes[256];
  
  unsigned long firstTimeAnyPacket, firstTimeOrderlyPacket, lastTimeAnyPacket;
  unsigned countAnyPacket, countOrderlyPacket;
  
  unsigned long lastStatsLogged;

} dmxState;

// if the value received for DMX channel A requested the lamp/fixture to be on
boolean shouldBeOnA = true;
// if the value received for DMX channel A requested the lamp/fixture to be on
boolean shouldBeOnB = true;

// if lamp may be on regarding temperature limits.
// always true without temperature monitoring.
boolean mayBeOn  = true;

// if temperature is rising, use PWM before shutting off completely
boolean pwmActive = true;

boolean connReported = false;

/**************************************
 * WiFi Manager
 */

#ifdef WITH_WIFI_MANAGER

int parseParam(WiFiManagerParameter& p, int min, int max) {
   int i = String(p.getValue()).toInt();
   return i<=min ? min : i>=max ? max : i;
}

void saveParamsCallback () {
  dumpConfig();

  const char * name = custom.fixture.getValue();
  if (name[0]>0) {
    int len = sizeof(config.fixture)-1;
    strncpy(config.fixture, name, len);
    config.fixture[len] = 0;
    AP_SSID = config.fixture;
  }
  
  config.universe      = parseParam(custom.universe,      -1, 7);
  // Channel A
  config.channelA      = parseParam(custom.channelA,      0, 512);
  config.pinA          = parseParam(custom.pinA,         -1,  16);
  config.activeA       = parseParam(custom.activeA,       0,   1);
  // Channel B
  config.channelB      = parseParam(custom.channelB,      0, 512);
  config.pinB          = parseParam(custom.pinB,         -1,  16);
  config.activeB       = parseParam(custom.activeB,       0,   1);
  #ifdef WITH_STROBOSCOPE
  // Stroboscope
  config.channelStrobe = parseParam(custom.channelStrobe, 0, 512);
  config.pinStrobe     = parseParam(custom.pinStrobe,    -1,  16);
  #endif
  #ifdef WITH_TEMP_MONITORING
  // Temperature monitoring
  config.pinCool       = parseParam(custom.pinCool,      -1,  16);
  config.activeCool    = parseParam(custom.activeCool,    0,   1);
  config.monitor       = parseParam(custom.monitor, 0,   1);
  config.low           = parseParam(custom.low,  -126, 127);
  config.warn          = parseParam(custom.warn, -126, 127);
  config.high          = parseParam(custom.high, -126, 127);
  #endif  

  EEPROM.begin(sizeof(config));  
  EEPROM.put(0, config);
  EEPROM.commit();  

  dumpConfig();
}
#endif

#ifdef WITH_OTA
void setupOTA();
#endif

#ifdef WITH_STROBOSCOPE

int oldStrobeValue = -1;
volatile uint8_t strobeValue = 0;
volatile uint32_t interval = 2500000; // 0.5s
volatile long oneshotEnd = 0;

// how often the temparture is measured/checked 
#define MEASURE_INTERVAL 1000
long tempMonitorInterval = MEASURE_INTERVAL; // 2.5s

// saves old state regarding whether recieving DMX data 
boolean wasReceiving = false;

int loopCount = 0;


// interrupt timer routine generating stroboscope signal
void ICACHE_RAM_ATTR timerRoutine() {
  if (oneshotEnd>0) {
    long now = millis();
    //log(String("timerRoutine: now=") + now + ", oneshotEnd=" + oneshotEnd);
    if (now>oneshotEnd) {
      log("LED OFF");
      digitalWrite(config.pinStrobe, LOW);      
      oneshotEnd = 0;
    }
    timer1_write(250000/2); // 10 times/s
  }
  else if (0==interval) {
    digitalWrite(config.pinStrobe, LOW);
    timer1_write(250000/2); // 10 times/s
  } 
  else {
    digitalWrite(config.pinStrobe, 1-digitalRead(config.pinStrobe));
    timer1_write(interval);
  }
}
#endif

void setup() {

  Serial.begin(115200);
  Serial.println();
  Serial.println(String("ArtnetRelay ") + VERSION + " " + BUILD + " CHID " + CHIP_ID);
  
  #ifdef WITH_TEMP_MONITORING
  Serial.print("ONE_WIRE_BUS: " ); Serial.println(ONE_WIRE_BUS);
  #endif

  memset(&dmxState, 0, sizeof(dmxState));

  initConfig();
  
  EEPROM.begin(sizeof(config));  
  EEPROM.get(0, config);
  Serial.println(String("Config magic: ") + config.magic);
  if (MAGIC != config.magic) {
    Serial.println("First time config"); 
    initConfig(); // reset
    #ifdef WITH_WIFI_MANAGER
    wm.resetSettings(); // as well
    #endif
    EEPROM.put(0, config);
    EEPROM.commit();
  }
  EEPROM.end();

  dumpConfig();
  if (config.fixture[0]==0) {
    AP_SSID += "_";
    AP_SSID += CHIP_ID;   
    log(String("No custom fixture name. Appended chip ID to AP name: '") + AP_SSID + "'");
  } 
  else {
    AP_SSID = String(config.fixture);
    log(String("Using custom fixture name for AP name: '") + config.fixture + "'");
  }

  #ifdef WITH_WIFI_MANAGER
  
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP    
  //wm.resetSettings();
  wm.addParameter(&custom.fixture);
  wm.addParameter(&custom.universe);  
  wm.addParameter(&custom.channelA);
  wm.addParameter(&custom.pinA); 
  wm.addParameter(&custom.activeA);  
  wm.addParameter(&custom.channelB); 
  wm.addParameter(&custom.pinB);
  wm.addParameter(&custom.activeB);
  wm.addParameter(&custom.channelStrobe);
  wm.addParameter(&custom.pinStrobe);
  wm.addParameter(&custom.pinCool);
  wm.addParameter(&custom.activeCool);
  #ifdef WITH_TEMP_MONITORING
  wm.addParameter(&custom.monitor);
  wm.addParameter(&custom.low); // dallas sensors report > -127 C
  wm.addParameter(&custom.warn);
  wm.addParameter(&custom.high);
  #endif
  
  wm.setSaveParamsCallback(saveParamsCallback);  
  wm.setConfigPortalBlocking(false);  
  if (wm.autoConnect(AP_SSID.c_str())){
    Serial.println("Auto-connect was successful");
  }
  else {
    Serial.println("Configportal running");
  }

  #else

  WiFi.mode(WIFI_AP_STA);
  log(String("Connecting to '") + STA_SSID + "'");
  WiFi.begin(STA_SSID, STA_PASS);  
  log(String("Starting access point '") + AP_SSID + "'");
  WiFi.softAPConfig(AP_ADDR, AP_ADDR, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS);
  //dnsServer.start(DNS_PORT, "*", AP_ADDR);
  
  #endif

  #ifdef WITH_OTA
  setupOTA();
  #endif

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(config.pinA, OUTPUT);
  for (int i=0; i<2; i++) {
    digitalWrite(config.pinA, !config.activeA);
    delay(200);
    digitalWrite(config.pinA, config.activeA);
    delay(200);
  }

  // Start up the library
  #ifdef WITH_TEMP_MONITORING
  if (config.monitor) {
    sensors.begin();
    sensors.setResolution(9);
    sensors.requestTemperatures();
    temperaturesRequested = true;
  }
  #endif

  #ifdef WITH_WEB_SERVER
  server.on("/",     handleRoot);
  server.on("/conf", handleConf);
  server.begin();
  #endif

  #ifdef WITH_UDP_LOGGING
  UDP.begin(UDP_PORT);  
  #endif
  
  artnet.begin();
  artnet.setArtDmxCallback(onDmxPacket);

  myDigitalWrite("setup", config.pinA, shouldBeOnA ? config.activeA : 1-config.activeA);
  myDigitalWrite("setup", config.pinB, shouldBeOnB ? config.activeB : 1-config.activeB);

  #ifdef WITH_STROBOSCOPE
  if (config.pinStrobe>-1) {
    timer1_attachInterrupt(timerRoutine);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
    // 2500000 / 5 ticks per us from TIM_DIV16 == 500,000 us interval -> 1Hz
    timer1_write(2500000); 
  }
  #endif
}

bool shouldBeOnAOld = shouldBeOnA;

void checkConnection() {
  #ifdef WITH_WIFI_MANAGER
  wm.process();
  #endif

  if (WiFi.status() == WL_CONNECTED) {
    if (!connReported) {      
      String msg = String("ArtnetRelay ") + VERSION + " " + BUILD + " CHID " + CHIP_ID;
      msg += ": Connected to WiFi: '" + WiFi.SSID() + "' with IP " + WiFi.localIP().toString();
      log(msg);
      dumpConfig();      

      log(String("(Re-)enabling access point ") + AP_SSID);
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(AP_SSID, AP_PASS);
      Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
      
      #ifdef WITH_UDP_LOGGING
      log("Re-enabling UDP logging");
      UDP.stop();  
      UDP.begin(UDP_PORT);  
      log(String("Logging on UDP port ") + UDP_PORT);    
      #endif

      connReported = true;
    }
  } 
}

void loop(void) { 

  loopCount++;

  checkConnection();  
  checkUdpLogging();

  artnet.read();

  #ifdef WITH_OTA
  ArduinoOTA.handle();  
  #endif
  
  #ifdef WITH_WEB_SERVER
  server.handleClient();
  #endif

  int oldA = myDigitalRead(config.pinA), oldB = myDigitalRead(config.pinB);
  int newA = oldA, newB = oldB;

  #ifdef WITH_TEMP_MONITORING
  int oldC = myDigitalRead(config.pinCool);
  int newC = oldC;
  #endif

  bool temperatureOk = checkTemperature();

  if (config.pinA>-1) {
    newA = shouldBeOnA ? config.activeA : 1-config.activeA;
  }
  if (config.pinB>-1) {
    newB = shouldBeOnB ? config.activeB : 1-config.activeB;
  }

  #ifdef WITH_TEMP_MONITORING
  if (config.pinCool>-1) {
    if (config.pinCool==config.pinA) {
      bool before = newA;
      newA = temperatureOk ? newA : config.activeCool;
    } 
    else if (config.pinCool==config.pinB) {      
      newB = temperatureOk ? newB : config.activeCool;
    }
    else {
      newC = temperatureOk ? 1-config.activeCool : config.activeCool;
    }
  }
  #endif

  if (oldA != newA) {
    myDigitalWrite("A changed", config.pinA, newA);
  }
  if (oldB != newB) {
    myDigitalWrite("B changed", config.pinB, newB);
  }
  #ifdef WITH_TEMP_MONITORING
  if (oldC != newC) {
    myDigitalWrite("C changed", config.pinCool, newC);
  }
  #endif

  unsigned long now = millis();

  boolean isReceiving = 
    (dmxState.firstTimeAnyPacket>0) && (now-dmxState.lastTimeAnyPacket<1500);

  if (wasReceiving && !isReceiving) {
    log("DMX stream ended");
    digitalWrite(LED_BUILTIN, HIGH);
    dmxState.firstTimeAnyPacket = dmxState.firstTimeOrderlyPacket = 0;
    dmxState.countAnyPacket = dmxState.countOrderlyPacket = 0;
  }

  if (now-dmxState.lastStatsLogged >= 10*1000) {
      if (isReceiving && dmxState.firstTimeOrderlyPacket>0) {
        float ppsAny     = (1000.0 * dmxState.countAnyPacket)/(now-dmxState.firstTimeAnyPacket);
        float ppsOrderly = (1000.0 * dmxState.countOrderlyPacket)/(now-dmxState.firstTimeOrderlyPacket);
        log(String("packets: ") + ppsAny + " p/s, orderly: " + ppsOrderly + " p/s");
      } 
      else if (isReceiving) {
        float ppsAny     = (1000.0 * dmxState.countAnyPacket)/(now-dmxState.firstTimeAnyPacket);
        log(String("packets: ") + ppsAny + " p/s");
      }
      dmxState.lastStatsLogged = now;
  }

  if (!wasReceiving && isReceiving) {
      log(String("DMX stream started from ") + artnet.getSenderIp().toString() 
        + ", shouldBeOnA: " + shouldBeOnA 
        + ", shouldBeOnB: " + shouldBeOnB
      ); 
  }
  
  wasReceiving = isReceiving;

  #ifdef WITH_STROBOSCOPE
  if (isReceiving) {
    int value = strobeValue;
    if (value<10) {
      digitalWrite(config.pinStrobe, LOW);      
      interval = 0; // off        
    } else if (value<=20) {
      if (oldStrobeValue!=value && 0==oneshotEnd) {
        digitalWrite(config.pinStrobe, HIGH);      
        interval = 0; // off        
        oneshotEnd = millis()+100; // 100ms later
      }
    }
    else {
      // -> [~0.3 Hz,...,~24Hz] -> [~0.6,...,~12] flashes/s      
      interval = 25*1000*1000 / (value-20); 
    } 
    strobeValue = value;
  }
  #endif

}

void myDigitalWrite(String reason, int pin, int status) {
  if (pin<0) return;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, status);
  int check = digitalRead(pin);
  log(String("************************ myDigitalWrite: reason: ") + reason + ", pin " + pin + " -> " + status + " check: " + check);
}

int myDigitalRead(int pin) {
  if (pin<0) return -1;
  pinMode(pin, OUTPUT);
  int reading = digitalRead(pin);
  //log(String("myDigitalWrite: myDigitalRead ") + pin + " -> " + reading);
  return reading;
}

bool checkTemperature() {
    
  #ifdef WITH_TEMP_MONITORING  

  unsigned long now = millis();

  if (now-lastCheck > tempMonitorInterval) {
    //log(String("checkTemperature: lastCheck=") + lastCheck + ", tempMonitorInterval=" + tempMonitorInterval);
    lastCheck = now;
  
    if (!config.monitor) {
      tempMonitorInterval = 10000; // stop spamming
    } 
    else {
      tempMonitorInterval = MEASURE_INTERVAL; // config might change, re-enable 2.5s if so
    }

    lastCheck = now;
    float tempC = -300;
    
    if (config.monitor) {
      tempC = getTemperature();
      //log(String("tempC: ") + tempC);

      // Check if reading was successful:
      if (tempC == DEVICE_DISCONNECTED_C) {
        measureFailed = true;
        log("Error: measure failed\n");
      }
      else {
        measureFailed = false;
        lastMeasureSuccess = now;
      }
    } 

    bool debug = true;
    if (debug) {
      //log(String("checkTemperature: shouldBeOnA: ") + shouldBeOnA + ", shouldBeOnB: " + shouldBeOnB + ", mayBeOn: " + mayBeOn + ", pwmActive: " + pwmActive + ", tempC: " + tempC);      
    }

    if (!config.monitor) {
      //log(String("[") + now + "] Monitoring was disabled in config");      
      mayBeOn = true;
    } 
    else if (tempC>upperLimit) {      
      if (debug) {
        log(String("HOT, L:") + lowerLimit + ", T:" + tempC + ", W:" + warnLimit + ", U:" + upperLimit);
      }
      if (mayBeOn) {
        log("Upper limit reached");
      }
      mayBeOn = false;
    } 
    else if (tempC<lowerLimit) {
      if (debug) {
        log(String("CLD, L:") + lowerLimit + ", T:" + tempC + ", W:" + warnLimit + ", U:" + upperLimit);
      }
      if (!mayBeOn) {
        log("Lower limit reached");
      }
      mayBeOn = true;
    } 
    else if (tempC>warnLimit && mayBeOn) {      
      String line = String("WRN, L: ") + lowerLimit + ", T:" + tempC + ", W:" + warnLimit + ", U:" + upperLimit;
      //log("Warning limit reached\n");
      
      float range = upperLimit-warnLimit; // 62-61.5 = 0.5
      float delta = upperLimit-tempC; // ie. 0.5...0.0
      int pos = round(100.0*delta/range); // 100...0 as temp approaches upper limit
      pos = 10-pos/10; // 0 ... 9
      
      long secs = now/1000; // 1 secs
      long rem  = secs % 10; // 0 .... 9
      pwmActive  = rem>=pos; //
      
      line += String(", t:") + secs + ", r:" + rem + ", p:" + pos + + " -> PWM: " + pwmActive;
      log(line);    
    } 
    else {
      log(String("U/D t:") + millis() + ", L:" + lowerLimit + ", T:" + tempC + ", W:" + warnLimit + ", U:" + upperLimit);
    }
    if (debug) {
      //log(String("shouldBeOnA: ") + shouldBeOnA + ", mayBeOn: " + mayBeOn);
    }
  }

  return (mayBeOn && pwmActive);  
  #endif
  
  return true;
}

#ifdef WITH_TEMP_MONITORING
float getTemperature() {
  
  #ifdef WITH_DEBUG_TEMP
  unsigned long now = millis();
  float sequence = 0.1*((now/3000)%100); // 0.0, ..., 10.0
  float zigzag = sequence<5 ? sequence : 10-sequence;
  return 54.0+zigzag;  
  #endif

  //return 0;
  float tempC = DEVICE_DISCONNECTED_C;
  for (int i=0; i<20; i++) {
    if (!temperaturesRequested) {
      sensors.requestTemperatures();
      temperaturesRequested = true;
    }
    tempC = sensors.getTempCByIndex(0);
    if (tempC != DEVICE_DISCONNECTED_C) {
      break;
    } else {
      log("Error: DEVICE_DISCONNECTED_C, retry");      
      delay(1);
    }
  }  

  sensors.requestTemperatures();
  temperaturesRequested = true;
  return tempC;
}
#endif


void checkUdpLogging() {
  #ifdef WITH_UDP_LOGGING
  long now  = millis();
  if (now >= udpLoggingEnds) {
    if (doLogUDP) {
      log("*** UDP logging stops ***");
    }
    doLogUDP = false;
  }
  #endif
}

#ifdef WITH_OTA
void setupOTA() {
  OTA_NAME += "_";
  OTA_NAME += CHIP_ID;
  ArduinoOTA.setHostname(OTA_NAME.c_str());
  ArduinoOTA.begin();
}
#endif

// This will be called for each UDP packet received
void onDmxPacket(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t * data) {
  unsigned long now  = millis();
  unsigned long last = dmxState.lastSequenceTimes[sequence];

  // just in case this might be useful (e.g. to block a concurrent DMX broadcaster)
  //IPAddress source = artnet.getSenderIp();

  dmxState.lastTimeAnyPacket = now;
  dmxState.countAnyPacket++;
  if (0==dmxState.firstTimeAnyPacket) {
    dmxState.firstTimeAnyPacket = now;
    dmxState.lastStatsLogged = now-9*1000; // print next stats asap
  }

  // assumption: 50 packets/sec. 256/50 = 5.12 sec.
  if (now-last < 4000) {
  }
  else {
    if (0==dmxState.firstTimeOrderlyPacket) {
      dmxState.firstTimeOrderlyPacket = now;
    }
    dmxState.countOrderlyPacket++;

    if (length>config.channelA) {
      byte value = data[config.channelA-1];
      bool old = shouldBeOnA;
      shouldBeOnA = value>127;
      if (old!=shouldBeOnA) {
        log(String("DMX: shouldBeOnA: ") + old + " -> " + shouldBeOnA);        
      }
    }
    if (length>config.channelB) {
      byte value = data[config.channelB-1];
      shouldBeOnB = value>127;
    }
    #ifdef WITH_STROBOSCOPE
    if (length>config.channelStrobe) {
      strobeValue = data[config.channelStrobe-1];
    }
    #endif
  }
  
  dmxState.lastSequenceTimes[sequence] = now;
  dmxState.lastSequence = sequence;
}

#ifdef WITH_WEB_SERVER
void handleNotFound() {
  //digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  // message += "\nArguments: ";
  // message += server.args();
  // message += "\n";
  // for (uint8_t i = 0; i < server.args(); i++) {
  //   message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  // }
  server.send(404, "text/plain", message);
  //digitalWrite(led, 0);
}

int parseArg(String& s, int min, int max) {
  int i = s.toInt();
  return i<=min ? min : i>=max ? max : i;
}

void handleConf() {
  log(String("handleConf: args=") + server.args());

  Config copy;
  memcpy(&copy, &config, sizeof(copy));

  boolean apply   = false;
  boolean persist = false;
  for (int i = 0; i < server.args(); i++) {
    String name  = server.argName(i);
    String value = server.arg(i);

    if (name.equals("submit")) {      
      apply   = value.equals("Apply");
      persist = value.equals("Persist");
    } else if (name.equals("name")) {      
      memcpy(copy.fixture, value.c_str(), sizeof(copy.fixture)-1);
      copy.fixture[sizeof(copy.fixture)-1] = 0;
    } else if (name.equals("chnA")) {      
      copy.universe = parseArg(value,  0, 512);
    } else if (name.equals("chnB")) {      
      copy.universe = parseArg(value,  0, 512);
    } else if (name.equals("chnS")) {      
      copy.universe = parseArg(value,  0, 512);
    } else if (name.equals("pinA")) {      
      copy.universe = parseArg(value, -1,  16);
    } else if (name.equals("pinB")) {      
      copy.universe = parseArg(value, -1,  16);
    } else if (name.equals("pinC")) {      
      copy.universe = parseArg(value, -1,  16);
    } else if (name.equals("actA")) {      
      copy.universe = parseArg(value,  0,   1);
    } else if (name.equals("actB")) {      
      copy.universe = parseArg(value,  0,   1);
    } else if (name.equals("actC")) {      
      copy.universe = parseArg(value,  0,   1);
    } 
    #ifdef WITH_TEMP_MONITORING
    else if (name.equals("moni")) {      
      copy.monitor = parseArg(value,   0,   1);
    } else if (name.equals("low")) {      
      copy.low = parseArg(value,    -127, 127);
    } else if (name.equals("warn")) {      
      copy.warn = parseArg(value,   -127, 127);
    } else if (name.equals("high")) {      
      copy.high = parseArg(value,   -127, 127);
    }
    #endif
  }

  boolean restartAP = false;
  if (apply || persist) {
    //dumpConfig();    
    if (strcmp(config.fixture, copy.fixture)) {
      restartAP = true;
    }
    memcpy(&config, &copy, sizeof(config));
    dumpConfig();    
  }
  if (persist) {
    EEPROM.begin(sizeof(config));  
    EEPROM.put(0, config);
    EEPROM.commit();  
    log("*** config saved to eeprom **");
  }
  if (restartAP) {
    AP_SSID = config.fixture;
    WiFi.softAP(AP_SSID, AP_PASS);
    log("*** AP restarted **");
  }
  if (apply || persist) {
    server.sendHeader("Location", String("/conf"), true);
    server.send (302, "text/plain", "");
    return;
  }

  String body = 
    String("<html>\n") +
    "<head>\n" 
    "<meta charset=\"UTF-8\">\n" 
    "<title>ArtnetRelay</title>\n" 
    "<script>\n" 
    "function clear() {\n" 
    "  document.getElementById('name').value = '';\n" 
    "  return false;\n" 
    "}\n" 
    "function copy(s) {\n" 
    "  var e = s.getInnerHTML().trim();\n" 
    "  document.getElementById('name').value += e;\n" 
    "}\n" 
    "</script>\n" 
    // "<style>\n" 
    // ".e { background-color: red; onclick='alert(this);' }\n"
    // "</style>\n" 
    "</head>\n" 
    "<body style=\"font-family: Arial\"><h3>Artnet Relay</h3>\n" 
    "<form><table>\n";
  
  const char * emojis[] = { "😀","😎","🤩","🥳","🎉","🍺","🍹","🥂","☠️","💀","👻","💋","🌿","🎅","🎄","🎵","🎶", };
  for (int i=0; i<sizeof(emojis)/sizeof(emojis[0]); i++) {
    body += String("<span onclick=\"copy(this)\">") + emojis[i] + "</span>";
  }
    
  body += 
    String("<tr>\n") + 
    "  <td>Name</td>\n" 
    "  <td colspan=\"6\">\n"
    "    <input id=\"name\" type=\"text\" size=\"32\" maxlength=\"32\" name=\"name\" value=\"" + config.fixture + "\"> (" + strlen(config.fixture) + ")\n"
    "  </td>\n"
    "</tr>\n"
    "<tr><td>Universe </td><td><input type=\"text\" maxlength=\"2\"  size=\"3\" name=\"univ\" value=\"" + config.universe + "\"></td></tr>\n"
    "<tr>\n"
    "  <td></td>\n"
    "  <td>channel</td>\n"
    "  <td>gpio    </td>\n"
    "  <td>active </td>\n" 
    "</tr>\n"
    "<tr>\n"
    "  <td>Relay A</td><td><input type=\"text\" maxlength=\"3\" size=\"3\" name=\"chnA\" value=\"" + config.channelA + "\"></td>\n"
    "  <td><input type=\"text\" maxlength=\"2\" size=\"3\" name=\"pinA\" value=\"" + config.pinA + "\"></td>\n"
    "  <td><input type=\"text\" maxlength=\"1\" size=\"3\" name=\"actA\" value=\"" + config.activeA + "\"></td>\n" 
    "</tr>\n"
    "<tr>\n" 
    "  <td>Relay B</td><td><input type=\"text\" maxlength=\"3\" size=\"3\" name=\"chnA\" value=\"" + config.channelB + "\"></td>\n"
    "  <td><input type=\"text\" maxlength=\"2\" size=\"3\" name=\"pinA\" value=\"" + config.pinB + "\"></td>\n"
    "  <td><input type=\"text\" maxlength=\"1\" size=\"3\" name=\"actA\" value=\"" + config.activeB + "\"></td>\n" 
    "</tr>\n"
    "<tr>\n" 
    "  <td>Strobe</td><td><input type=\"text\" maxlength=\"3\" size=\"3\" name=\"chnA\" value=\"" + config.channelStrobe + "\"></td>\n"
    "  <td><input type=\"text\" maxlength=\"2\" size=\"3\" name=\"pinA\" value=\"" + config.pinStrobe + "\"></td>\n"
    "</tr>\n"
    #ifdef WITH_TEMP_MONITORING
    "<tr>\n" 
    "  <td>Cooling</td><td></td>\n"
    "  <td><input type=\"text\" maxlength=\"2\" size=\"3\" name=\"pinA\" value=\"" + config.pinCool + "\"></td>\n"
    "  <td><input type=\"text\" maxlength=\"1\" size=\"3\" name=\"actA\" value=\"" + config.activeCool + "\"></td>\n" 
    "</tr>\n"
    "<tr style=\"height:1ex\"/>\n"     
    "<tr>\n"
    "  <td/><td/>\n"
    "  <td>low </td><td>warn</td><td>high</td>\n"  
    "</tr>\n" 
    "<tr>\n" 
    "  <td>Monitor</td><td><input type=\"text\" maxlength=\"1\" size=\"3\" name=\"moni\" value=\"" + config.monitor + "\"></td>\n"
    "  <td><input type=\"text\" size=\"3\" maxlength=\"3\" name=\"low\"  value=\"" + config.low + "\">°C</td>\n"
    "  <td><input type=\"text\" size=\"3\" maxlength=\"3\" name=\"warn\" value=\"" + config.warn + "\">°C</td>\n"
    "  <td><input type=\"text\" size=\"3\" maxlength=\"3\" name=\"high\" value=\"" + config.high + "\">°C</td>\n"  
    "</tr>\n"
    "</tr>\n" 
    #endif
    "<tr style=\"height:1ex\"/>\n"     
    "<tr>\n" 
    "<td colspan=\"6\">\n"
    "  <input action=\"GET\" type=\"submit\" name=\"submit\" value=\"Apply\"  >\n" 
    "  <input action=\"GET\" type=\"submit\" name=\"submit\" value=\"Persist\">\n"
    "</td>\n" 
    "</tr>\n"
    "</table></form></body>\n"  
    "</html>\n";
  server.send(200, "text/html", body);
}

void handleRoot() {
  digitalWrite(LED_BUILTIN, LOW);
  server.sendHeader("Refresh", String("1;url=/conf"), true);
  server.send(200, "text/plain", 
    String("Hello from ") + AP_SSID
    + " " + VERSION + " " + BUILD
    + "\nDMX universe " + config.universe 
    + "\nChannel A " + config.channelA  
    + "\nPin A" + config.pinA 
    + "\nActive A " + config.activeA 
    + "\nChannel B " + config.channelB
    + "\nPin B" + config.pinB 
    + "\nActive B " + config.activeB
    #ifdef WITH_TEMP_MONITORING
    + "\nMonitore "  + config.monitor
    + "\nLow "  + config.low
    + "\nWarn " + config.warn
    + "\nHigh " + config.high
    + "\nhttp://" + WiFi.localIP().toString()  + "/conf"
    + "\nhttp://" + WiFi.softAPIP().toString() + "/conf"
    #endif
    + "\n"
  );
  digitalWrite(LED_BUILTIN, HIGH);
}
#endif

void log(String msg) {
  Serial.print('['); Serial.print(millis()); Serial.print("] ["); Serial.print(loopCount); Serial.print("] ");
  Serial.println(msg);
  #ifdef WITH_UDP_LOGGING
  if (doLogUDP) {
    UDP.beginPacket("255.255.255.255", UDP_PORT);
    UDP.print("  ");
    UDP.print(msg);
    UDP.endPacket();
  }
  #endif
}
