#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <FS.h>
#include <PubSubClient.h>
#include <TimeLib.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

// AP INFO
#define AP_TIMEOUT 20 // in seconds
#define AP_SSID "AutoConnect"
#define AP_PASS "APp@ssword"

// MQTT Max Attempts
#define MQTT_MAX_ATTEMPTS 5

// LAMP INFO
#define LAMP_NAME "insert_lamp_name"
#define LAMP_INDEX 0

// STATUS LIGHT COLORS
#define WIFI_CONNECTED strip.Color(127, 127, 127)
#define CALIBRATED strip.Color(255, 255, 0)
#define MQTT_CONNECTED strip.Color(0, 255, 0)
#define CONNECTION_ERROR strip.Color(255, 158, 0)

// Pin Mapping
#define LED_PIN 10
#define S_PIN 14
#define R_PIN 12

// Capacitive Sensitivity
// BASELINE_VARIANCE: The higher the number the less the baseline is affected by
// current readings. (was 4)
#define BASELINE_VARIANCE 4
// SENSITIVITY: Integer. Higher is more sensitive (was 8)
#define SENSITIVITY 8
// BASELINE_SENSITIVITY: Integer. A trigger point such that values exceeding this point
// will not affect the baseline. Higher values make the trigger point sooner. (was 16)
#define BASELINE_SENSITIVITY 16
// SAMPLE_SIZE: Number of samples to take for one reading. Higher is more accurate
// but large values cause some latency.(was 32)
#define INIT_SAMPLE_SIZE 512 // For initial calibration
#define LOOP_SAMPLE_SIZE 16
#define SAMPLES_BETWEEN_PIXEL_UPDATES 8
#define LOOPS_TO_FINAL_COLOR 150

// Timing
#define PERIODIC_UPDATE_TIME 5 // seconds
#define COLOR_CHANGE_WINDOW 10 // seconds

//NeoPixel Info
#define LED_COUNT 24
#define LED_BRIGHTNESS 63 // Out of 255

// States
#define ATTACK 1
#define DECAY 2
#define SUSTAIN 3
#define RELEASE1 4
#define RELEASE2 5
#define OFF 6

#define LOCAL_CHANGE 0
#define REMOTE_CHANGE 1

#define END_VALUE 0
#define TIME 1

#define tEVENT_NONE 0
#define tEVENT_TOUCH 1
#define tEVENT_RELEASE 2

const int minMaxColorDiffs[2][2] = {
  {5, 20},  // min/Max if color change last color change from same touch light
  {50, 128} // min/Max if color change last color change from different touch light
};

// END VALUE, TIME
// 160 is approximately 1 second
const long envelopes[6][2] = {
  {0, 0},      // NOT USED
  {255, 30},   // ATTACK
  {200, 240},  // DECAY
  {200, 1000}, // SUSTAIN
  {150, 60},   // RELEASE1
  {0, 48000} // RELEASE2 (65535 is about 6'45")
};

int colors[] = {
  160,   // Teal
  90,  // Magenta
  170, // Blue
  79,  // Orange
  131  // Purple
};

String eventTypes[] = {
  "None",
  "Touch",
  "Release"
};

uint32_t colorWheel(byte WheelPos, byte iBrightness);

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
const char* touchEventName = "touch_event";

char mqtt_server[40];
char mqtt_port[6];
char mqtt_user[40];
char mqtt_pass[40];
WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqtt_server, 40);
WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port, 6);
WiFiManagerParameter custom_mqtt_user("username", "MQTT Username", mqtt_user, 40);
WiFiManagerParameter custom_mqtt_pass("password", "MQTT Password", mqtt_pass, 40);

unsigned char myId = LAMP_INDEX;
unsigned char lastColorChangeDeviceId = -1;

int currentEvent = tEVENT_NONE;
int eventTime = now();
int eventTimePrecision = random(INT_MAX);

int initColor = 0;
int currentColor = 0; // 0 to 255
int finalColor = 0;   // 0 to 255
int lastLocalColorChangeTime = now();

int initBrightness = 0;    // 0 to 255
int currentBrightness = 0; // 0 to 255

unsigned char prevState = OFF;
unsigned char state = OFF;

long loopCount = 0;
long colorLoopCount = 0;
int mqtt_attempt_count = 0; 
int lastPeriodicUpdate = now();

// Timestamps
unsigned long tS;
volatile unsigned long tR;

// Baseline
float tBaseline;

// Flag for saving data
bool shouldSaveConfig = false;

// Callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void ICACHE_RAM_ATTR touchSense();

void callback(char* topic, byte* payload, unsigned int length) {
  String eventData = String((char*)payload);
  int deviceIdEnd = eventData.indexOf(',');
  int deviceId = eventData.substring(0, deviceIdEnd).toInt();
  int eventEnd = eventData.indexOf(',', deviceIdEnd + 1);
  int serverEvent = eventData.substring(deviceIdEnd + 1, eventEnd).toInt();
  int colorEnd = eventData.indexOf(',', eventEnd + 1);
  int serverColor = eventData.substring(eventEnd + 1, colorEnd).toInt();
  int eventTimeEnd = eventData.indexOf(',', colorEnd + 1);
  int serverEventTime = eventData.substring(colorEnd + 1, eventTimeEnd).toInt();
  int serverEventTimePrecision = eventData.substring(eventTimeEnd + 1).toInt();

  if (deviceId == myId) return;
  if (serverEventTime < eventTime) return;
  // Race condition brought colors out of sync
  if (
    serverEventTime == eventTime &&
    serverEventTimePrecision == eventTimePrecision &&
    serverColor != finalColor &&
    myId < deviceId
  ) {
    setColor(serverColor, prevState, deviceId);
    changeState(ATTACK, REMOTE_CHANGE);
    return;
  }
  if (serverEventTime == eventTime && serverEventTimePrecision <= eventTimePrecision) return;

  // Valid remote update
  setEvent(serverEvent, serverEventTime, serverEventTimePrecision);

  if (serverEvent == tEVENT_TOUCH) {
    setColor(serverColor, prevState, deviceId);
    changeState(ATTACK, REMOTE_CHANGE);
  } else {
    changeState(RELEASE1, REMOTE_CHANGE);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(LAMP_NAME, mqtt_user, mqtt_pass)) {
      Serial.println("Connected to the MQTT Server(" + String(mqtt_server) + ":" + String(mqtt_port) + ")");
      client.publish("online", LAMP_NAME);
      client.subscribe(touchEventName);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
    mqtt_attempt_count++;
    if (mqtt_attempt_count >= MQTT_MAX_ATTEMPTS) {
      reconfigConnection();
      mqtt_attempt_count = 0;
    }
  }
  colorCycle(MQTT_CONNECTED, 50);
}

// Called when previous WiFi fails
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}


void setup() {
  Serial.begin(115200);

  strip.begin();
  colorWipe(0, 0);
  strip.setBrightness(LED_BRIGHTNESS);

  pinMode(S_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(R_PIN), touchSense, RISING);

  setupConnection();

  // Touch Calibration
  tBaseline = touchSampling(INIT_SAMPLE_SIZE);
  Serial.println("Touch calibrated");
  colorCycle(CALIBRATED, 50);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting to reconnect...");
    setupConnection();
  }
  
  if (!client.connected()) {
    reconnect();
  }
  
  client.loop();
  
  int touchEvent = touchEventCheck();
  
  if (touchEvent == tEVENT_NONE) {
    // Publish periodic updates to synchronize state
    bool touchedBefore = currentEvent != tEVENT_NONE;
    if (lastPeriodicUpdate < now() - PERIODIC_UPDATE_TIME && touchedBefore) {
      publishTouchEvent(currentEvent, finalColor, eventTime, eventTimePrecision);
      lastPeriodicUpdate = now();
    }
    return;
  }

  setEvent(touchEvent, now(), random(INT_MAX));

  Serial.println(eventTypes[touchEvent]);

  if (touchEvent == tEVENT_TOUCH) {
    int newColor = generateColor(finalColor, prevState, lastColorChangeDeviceId);
    setColor(newColor, prevState, myId);
    changeState(ATTACK, LOCAL_CHANGE);
  }
}

// Util
void readConfig() {
  //read configuration from FS json
  Serial.println("Mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("Mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("Reading config file...");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("Opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        StaticJsonDocument<244> jsonDoc;
        auto err = deserializeJson(jsonDoc, buf.get());
        if (!err){
          serializeJson(jsonDoc, Serial);
          Serial.println("\nParsed json");
          // Read values
          strcpy(mqtt_server, jsonDoc["mqtt_server"]);
          strcpy(mqtt_port, jsonDoc["mqtt_port"]);
          strcpy(mqtt_user, jsonDoc["mqtt_user"]);
          strcpy(mqtt_pass, jsonDoc["mqtt_pass"]);
        } else {
          Serial.println("Failed to load json config");
        }
        configFile.close();
      } else {
        Serial.println("config.json does not exist");
      }
      SPIFFS.end();
    }
  } else {
    Serial.println("Failed to mount FS");
  }
}

void saveConfig() {
  Serial.println("Mounting FS...");
  
  if (SPIFFS.begin()) {
    Serial.println("Saving config");
    StaticJsonDocument<244> jsonDoc;
    jsonDoc["mqtt_server"] = mqtt_server;
    jsonDoc["mqtt_port"] = mqtt_port;
    jsonDoc["mqtt_user"] = mqtt_user;
    jsonDoc["mqtt_pass"] = mqtt_pass;
    
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("Failed to open config file for writing");
    }
    
    serializeJson(jsonDoc, Serial);
    Serial.println();
    
    serializeJson(jsonDoc, configFile);
    
    configFile.close();
    SPIFFS.end();
  } else {
    Serial.println("Failed to mount FS");
  }
}

void setupConnection() {
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConnectTimeout(AP_TIMEOUT);
  
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);

  //Reset saved settings
  //wifiManager.resetSettings();

  wifiManager.autoConnect(AP_SSID, AP_PASS);
  
  postWiFiConnection();
}

void reconfigConnection() {
  colorCycle(CONNECTION_ERROR, 50);
  WiFiManager wifiManager;
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  if (!wifiManager.startConfigPortal(AP_SSID, AP_PASS)) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  postWiFiConnection();
}

void postWiFiConnection() {
  Serial.println("Connected to WiFi(" + WiFi.SSID() + ")");
  
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());

  if (shouldSaveConfig) {
    saveConfig();
  } else {
    readConfig();
  }

  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setCallback(callback);

  colorCycle(WIFI_CONNECTED, 50);
}

void setEvent(int event, int timeOfEvent, int timePrecision) {
  currentEvent = event;
  eventTime = timeOfEvent;
  eventTimePrecision = timePrecision;
}

// Touch
void touchSense() {
  tR = micros();
}

int touchEventCheck() {
  int touchSense;                     // current reading
  static int touchSenseLast = LOW;    // last reading

  static unsigned long touchDebounceTimeLast = 0; // debounce timer
  int touchDebounceTime = 50;                     // debounce time

  static int touchNow = LOW;  // current debounced state
  static int touchLast = LOW; // last debounced state

  int tEvent = tEVENT_NONE;   // default event

  // read touch sensor
  long tReading = touchSampling(LOOP_SAMPLE_SIZE);

  // touch sensor is HIGH if trigger point some threshold above Baseline
  if (tReading > (tBaseline + tBaseline / SENSITIVITY)) {
    touchSense = HIGH;
  } else {
    touchSense = LOW;
  }

  // debounce touch sensor
  // if state changed then reset debounce timer
  if (touchSense != touchSenseLast) {
    touchDebounceTimeLast = millis();
  }
  touchSenseLast = touchSense;

  // accept as a stable sensor reading if the debounce time is exceeded without reset
  if (millis() > touchDebounceTimeLast + touchDebounceTime) {
    touchNow = touchSense;
  }

  // set events based on transitions between readings
  if (!touchLast && touchNow) {
    tEvent = tEVENT_TOUCH;
  }

  if (touchLast && !touchNow) {
    tEvent = tEVENT_RELEASE;
  }

  // update last reading
  touchLast = touchNow;
  return tEvent;
}

long touchSampling(int sampleSize) {
  long tDelay = 0;
  int mSample = 0;
  static int timeout = 10000; // Timeout after 10000 failed readings
  int num_readings = 0;

  for (int i = 0; i < sampleSize; i++) {
    if (!(i % SAMPLES_BETWEEN_PIXEL_UPDATES)) {
      updateState();
    }
    pinMode(R_PIN, OUTPUT); // discharge capacitance at rPin
    digitalWrite(S_PIN, LOW);
    digitalWrite(R_PIN, LOW);
    pinMode(R_PIN, INPUT); // revert to high impedance input
    // timestamp & transition sPin to HIGH and wait for interrupt in a read loop
    num_readings = 0;
    tS = micros();
    tR = tS;
    digitalWrite(S_PIN, HIGH);
    do {
      // wait for transition
      num_readings++;
    } while (digitalRead(R_PIN) == LOW && num_readings < timeout);

    // accumulate the RC delay samples
    // ignore readings when micros() overflows
    if (tR > tS) {
      tDelay = tDelay + (tR - tS);
      mSample++;
    }
  }
   // calculate average RC delay [usec]
  if ((tDelay > 0) && (mSample > 0)) {
    tDelay = tDelay / mSample;
  } else {
    tDelay = 0;     // this is an error condition!
  }

  if (tDelay < (tBaseline + tBaseline / BASELINE_SENSITIVITY)) {
    tBaseline = tBaseline + (tDelay - tBaseline) / BASELINE_VARIANCE;
  }
  return tDelay;
}

void setColor(int color, unsigned char prevState, unsigned char deviceId) {
  lastColorChangeDeviceId = deviceId;
  if (prevState == OFF) currentColor = color;
  initColor = currentColor;
  finalColor = color;
  colorLoopCount = 0;
  Serial.print("get Color From Server Final color: ");
  Serial.print(initColor);
  Serial.print(", ");
  Serial.print(finalColor);
  Serial.print(", ");
}

int generateColor(int currentFinalColor, unsigned char prevState, int lastColorChangeDeviceId) {
  int color = 0;
  int current = now();
  Serial.println("generating color...");
  if (prevState == OFF || lastLocalColorChangeTime < current - COLOR_CHANGE_WINDOW) {
    color = colors[myId];
  } else {
    bool foreignId = (lastColorChangeDeviceId != myId);
    int minChange = minMaxColorDiffs[foreignId][0];
    int maxChange = minMaxColorDiffs[foreignId][1];
    int direction = random(2) * 2 - 1;
    int magnitude = random(minChange, maxChange + 1);
    color = currentFinalColor + direction * magnitude;
    color = (color + 256) % 256;
    // color = 119; // FORCE A COLOR
  }
  lastLocalColorChangeTime = current;
  Serial.print("final color: ");
  Serial.println(finalColor);
  return color;
}

void changeState(unsigned char newState, int remoteChange) {
  prevState = state;
  state = newState;
  initBrightness = currentBrightness;
  loopCount = 0;
  Serial.print("state: ");
  Serial.println(newState);

  if (remoteChange) return;

  if (newState == ATTACK || newState == RELEASE1) {
    publishTouchEvent(currentEvent, finalColor, eventTime, eventTimePrecision);
  }
}

void publishTouchEvent(int event, int color, int time, int timePrecision) {
  String response = String(myId)  + "," +
                    String(event) + "," +
                    String(color) + "," +
                    String(time)  + "," +
                    String(timePrecision);
  char message[response.length()];
  response.toCharArray(message, response.length());
  client.publish(touchEventName, message);
}

void updateState() {
  switch (state) {
    case ATTACK:
      if (loopCount >= envelopes[ATTACK][TIME]) {
        changeState(DECAY, LOCAL_CHANGE);
      }
      break;
    case DECAY:
      if ((loopCount >= envelopes[DECAY][TIME]) || (currentEvent == tEVENT_RELEASE)) {
        changeState(SUSTAIN, LOCAL_CHANGE);
      }
      break;
    case SUSTAIN:
      if ((loopCount >= envelopes[SUSTAIN][TIME]) || (currentEvent == tEVENT_RELEASE)) {
        changeState(RELEASE1, LOCAL_CHANGE);
      }
      break;
    case RELEASE1:
      if (loopCount >= envelopes[RELEASE1][TIME]) {
        changeState(RELEASE2, LOCAL_CHANGE);
      }
      break;
    case RELEASE2:
      if (loopCount >= envelopes[RELEASE2][TIME]) {
        changeState(OFF, LOCAL_CHANGE);
      }
      break;
  }

  currentBrightness = getCurrentBrightness(state, initBrightness, loopCount);
  if (currentColor != finalColor) {
    currentColor = getCurrentColor(finalColor, initColor, colorLoopCount);
  }

  uint32_t colorAndBrightness = colorWheel(currentColor, currentBrightness);
  colorWipe(colorAndBrightness, 0);
  loopCount++;
  colorLoopCount++;
}

// LEDs
int getCurrentBrightness(unsigned char state, int initBrightness, int loopCount) {
  if (state == OFF) return 0;
  int brightnessDistance = envelopes[state][END_VALUE] - initBrightness;
  int brightnessDistanceXElapsedTime = brightnessDistance * loopCount / envelopes[state][TIME];
  return min(255, max(0, initBrightness + brightnessDistanceXElapsedTime));
}

int getCurrentColor(int finalColor, int initColor, int colorLoopCount) {
  if (colorLoopCount > LOOPS_TO_FINAL_COLOR) return finalColor;
  int colorDistance = calcColorChange(initColor, finalColor);
  int colorDistanceXElapsedTime = colorDistance * colorLoopCount / LOOPS_TO_FINAL_COLOR;
  return (256 + initColor + colorDistanceXElapsedTime) % 256;
}

int calcColorChange(int currentColor, int finalColor) {
  int colorChange = finalColor - currentColor;
  int direction = (colorChange < 0) * 2 - 1;
  colorChange += direction * (abs(colorChange) > 127) * 256;
  return colorChange;
}

void colorCycle(uint32_t color, int wait) {
  strip.setBrightness(63);
  colorWipe(color, wait);
  colorWipe(strip.Color(0, 0, 0), wait);
  strip.setBrightness(LED_BRIGHTNESS);
}

void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

uint32_t colorWheel(byte WheelPos, byte iBrightness) {
  float R, G, B;
  float brightness = iBrightness / 255.0;

  if (WheelPos < 85) {
    R = WheelPos * 3;
    G = 255 - WheelPos * 3;
    B = 0;
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    R = 255 - WheelPos * 3;
    G = 0;
    B = WheelPos * 3;
  } else {
    WheelPos -= 170;
    R = 0;
    G = WheelPos * 3;
    B = 255 - WheelPos * 3;
  }
  R = R * brightness + .5;
  G = G * brightness + .5;
  B = B * brightness + .5;
  return strip.Color((byte) R, (byte) G, (byte) B);
}
