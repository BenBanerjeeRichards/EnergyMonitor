#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <LittleFS.h>
#include "Fs.h"
#include <WiFiManager.h>
#include "config.h"
#include "FS.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "Circular.h"
#include "StringUtil.h"

// Refresh string from server
#define REFRESH_SIZE 512

const int UPLOAD_BUFFER_SIZE = 14 * CAPACITY;

const int SERIAL_BAUD = 9600;
const int PIN_PULSE = 14;
const int PIN_D3 = SENSOR_PIN;
const int PIN_LED = 2;  // internal LED
const int PIN_BTN_1 = 12;

// State for tracking pulses
volatile unsigned long lastPulseMicros = -1;
volatile unsigned long lastPulseDurationMicros = -1;
// Erratic measurement tracking
// More than 200ms at high indicates erratic measurements - e.g. sensor not attached to meter
// If this occurs, wait for a number of normal measurements until we resume operations
// Note that these normality measurements are discarded even if we return to OK
enum class SensorState { Ok,
                         Erratic,
                         Disconnected };

const unsigned long MAX_DURATION_MICROS = 200 * 1000;
const unsigned int RETURN_TO_NORMALITY_TRESH = 3;     // Wait for this number of normal measurements until OK
bool hasBeenErratic = false;                          // Have we been erratic in this sync period? Set so we can notify API
int normalityCount = 0;                               // When in erratic mode, how many consequative OK measurements have we had
SensorState sensorState = SensorState::Disconnected;  // Disconnected until proven otherwise

// Button debounce and state
bool btn1Pressed = false;  // becomes true until button press event handled

// State for pulsing LED
const unsigned int PULSE_INTERVAL_MS = 50;
const int STATE_LED_OFF = 0;
const int STATE_LED_ON = 1;
int ledState = STATE_LED_OFF;
unsigned long prevMillis = 0;
unsigned long nextInterval = 1000;

// State for upload syncs
unsigned long lastUploadMs = 0;
char uploadStr[UPLOAD_BUFFER_SIZE] = "";
unsigned long nextUploadMs = 1000 * SYNC_PERIOD_SECONDS;
unsigned int failureCount = 0;

const unsigned int REFRESH_AFTER_SYNC_PERIOD_MS = 2000;
unsigned long nextRefresh = 1000000;

// Update LCD this frequently with percent progress
const unsigned long OTA_LCD_REFRESH_PERIOD_MS = 1000;
unsigned long otaNextUpdateMs = 0;

// Update wifi stength info
const unsigned int WIFI_STATUS_UPDATE_PERIOD_MS = 5000;
unsigned long nextCheckWifiStatusAt = 0;

// Config keys
const char* CFG_SECRET = "secret";
const char* CFG_SENSOR_ID = "sensorId";
const char* CFG_ENDPOINT = "endpoint";
const char* CFG_SYNC_ROUTE = "syncRoute";
const char* CFG_REFRESH_ROUTE = "refreshRoute";
const char* CFG_REFRESH_PERIOD = "refreshPeriod";  // In seconds
const char* CFG_IMPL_PER_KWH = "impulseKwh";

const int CFG_SIZE_SECRET= 256;
const int CFG_SIZE_SENSOR_ID = 16;
const int CFG_SIZE_ENDPOINT = 32;
const int CFG_SIZE_SYNC_ROUTE = 32;
const int CFG_SIZE_REFRESH_ROUTE = 32;
const int CFG_SIZE_REFRESH_PERIOD = 16;
const int CFG_SIZE_IMPLUSE_KWH = 16;

struct config_t {
  char secret[CFG_SIZE_SECRET];
  char sensorId[CFG_SIZE_SENSOR_ID];
  char endpoint[CFG_SIZE_ENDPOINT];
  char syncRoute[CFG_SIZE_SYNC_ROUTE];
  char refreshRoute[CFG_SIZE_REFRESH_ROUTE];
  char refreshPeriodSeconds[CFG_SIZE_REFRESH_PERIOD];
  char implusePerKwh[CFG_SIZE_IMPLUSE_KWH];
};

int refreshPeriodSeconds = 20;
int implusePerKwh = 0;

enum class WifiStrength {
  VeryGood,
  Good,
  Ok,
  Poor,
  Disconnected
};

enum class Error {
  ErraticSensor,
  UploadMem,
  UpdateFailed,
  FileOpenRFail,
  FileOpenWFail,
  FileWriteFail,

  // Config validation errors for each field
  InvalidConfig,
  InvalidConfigSensorId,
  InvalidConfigSecret,
  InvalidConfigEndpoint,
  InvalidConfigSyncRoute,
  InvalidConfigRefreshRoute,
  InvalidConfigImplKwh,
  InvalidConfigRefreshPeriod
};

bool isConfigError(Error error) {
  return error == Error::InvalidConfig || error == Error::InvalidConfigSensorId || error == Error::InvalidConfigSecret 
      || error == Error::InvalidConfigEndpoint || error == Error::InvalidConfigSyncRoute || error == Error::InvalidConfigRefreshRoute
      || error == Error::InvalidConfigImplKwh || error == Error::InvalidConfigRefreshPeriod;
}

// State for rendering to the display
enum class Screen {
  Hello,
  ConnectToAp,       // Instruct user to connect to the AP for setup
  ConnectingToWifi,  // Connecting to a specific (already saved) AP
  CurrentAndDay,     // Show current usage in W and the total used this day
  CurrentAndMonth,   // Same as above, but show the billing period (month)
  Error,             // Display an error code
  Updating,          // OTA update progress
  UpdateOk           // Shown on LCD before restart
};

Screen currentScreen = Screen::Hello;  // Current screen to be dislpayed
bool lcdPendingUpdate = true;          // true when we are pending an update to lcd display
// Actual data to be rendered to the screen. Big struct contains data for all possible screens
// Responsibility for the rest of the code to ensure that the state exists for the currentScreen
struct screen_state_t {
  int buttonCount;  // debug
  // ConnectingToWifi
  char connectingSSID[33];

  // CurrentAndDay
  int currentWatts;
  int todayPounds;
  int todayPence;
  WifiStrength wifiStrength;

  // Month
  int monthPounds;
  int monthPence;

  // Error
  Error error;

  // Updating
  int updatePercent;
};

// Backlight - turn off when inactive
const unsigned int BACKLIGHT_TIMEOUT_MS = 30 * 1000;
bool backlightIsOn = false;
unsigned int turnBacklightOffAt;

struct screen_state_t screenState;
struct config_t config;

// Always 3 digits (EXY)
const char* errorCode(Error error) {
  if (error == Error::UploadMem) {
    return "E11";
  }
  if (error == Error::ErraticSensor) {
    return "E21";
  }
  if (error == Error::UpdateFailed) {
    return "E22";
  }
  if (error == Error::FileOpenRFail) {
    return "E23";
  }
  if (error == Error::FileOpenWFail) {
    return "E24";
  }
  if (error == Error::FileWriteFail) {
    return "E25";
  }
  if (error == Error::InvalidConfig) {
    return "E30";
  }
  if (error == Error::InvalidConfigSecret) {
    return "E31";
  }
  if (error == Error::InvalidConfigSensorId) {
    return "E32";
  }
  if (error == Error::InvalidConfigEndpoint) {
    return "E34";
  }
  if (error == Error::InvalidConfigSyncRoute) {
    return "E35";
  }
  if (error == Error::InvalidConfigRefreshRoute) {
    return "E36";
  }
  if (error == Error::InvalidConfigRefreshPeriod) {
    return "E37";
  }
  if (error == Error::InvalidConfigImplKwh) {
    return "E38";
  }

  return "E??";
}

// Always less than 16 chars
const char* errorMessage(Error error) {
  if (error == Error::ErraticSensor) {
    return "Erratic Sensor";
  }
  if (error == Error::UploadMem) {
    return "Upload memory";
  }
  if (error == Error::UpdateFailed) {
    return "Update failed";
  }
  if (error == Error::FileOpenRFail) {
    return "File open read";
  }
  if (error == Error::FileOpenWFail) {
    return "File open write";
  }
  if (error == Error::FileWriteFail) {
    return "File write";
  }
  if (error == Error::InvalidConfig) {
    return "Invalid config";
  }
  if (error == Error::InvalidConfigSecret) {
    return "Cfg: secret";
  }
  if (error == Error::InvalidConfigSensorId) {
    return "Cfg: sensorId";
  }
  if (error == Error::InvalidConfigEndpoint) {
    return "Cfg: endpoint";
  }
  if (error == Error::InvalidConfigSyncRoute) {
    return "Cfg: syncRoute";
  }
  if (error == Error::InvalidConfigRefreshRoute) {
    return "Cfg: refreshRou";
  }
  if (error == Error::InvalidConfigRefreshPeriod) {
    return "Cfg: refreshPer";
  }
  if (error == Error::InvalidConfigImplKwh) {
    return "Cfg: implKwh";
  }
  return "Unknown";
}

WiFiClient client;

Error error;
boolean activeError = false;
Screen screenRestoreToAfterError;


bool cfg_write(const char* name, char* value);
bool cfg_read(const char* name, char* value, int maxLength);

void setError(Error error) {
  error = error;
  screenRestoreToAfterError = currentScreen;
  screenState.error = error;
  currentScreen = Screen::Error;
  activeError = false;
  lcdPendingUpdate = true;
}

void clearError() {
  activeError = false;
  currentScreen = screenRestoreToAfterError;
  lcdPendingUpdate = true;
}

byte poundLcdChar[8] = {
  0b01111,
  0b01000,
  0b01000,
  0b11110,
  0b01000,
  0b01000,
  0b11111,
  0b00000
};

byte wifiStrenghVeryGoodChar[8]{
  0b00000,
  0b00000,
  0b00001,
  0b00011,
  0b00111,
  0b01111,
  0b11111,
  0b00000
};

byte wifiStrenghGoodChar[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00010,
  0b00110,
  0b01110,
  0b11110,
  0b00000
};

byte wifiStrenghOkChar[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00100,
  0b01100,
  0b11100,
  0b00000
};

byte wifiStrenghPoorChar[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b01000,
  0b11000,
  0b00000
};

const int LCD_CUSTOM_POUND = 0;
const int LCD_CUSTOM_WIFI_VERY_STRONG = 1;
const int LCD_CUSTOM_WIFI_GOOD = 2;
const int LCD_CUSTOM_WIFI_OK = 3;
const int LCD_CUSTOM_WIFI_POOR = 4;



void ICACHE_RAM_ATTR ISR_D3_change();
void ICACHE_RAM_ATTR ISR_D6_high();


struct circular_buffer interval_buffer;
unsigned long lastDurationUs = 0;
bool currentUpdated = false;

struct refresh_result_t {
  // If this is set, then we are sending an update
  char updateUrl[64];
};

struct refresh_result_t refreshResult;
char refreshStr[REFRESH_SIZE];  // String from server for data refresh

WifiStrength getWiFiStrength();
void parseResponse(struct refresh_result_t* response, char* response_str);
bool refresh();

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

void turnOnBacklight() {
  if (!backlightIsOn) {
    lcd.backlight();
    backlightIsOn = true;
  }
  turnBacklightOffAt = millis() + BACKLIGHT_TIMEOUT_MS;
}

void append(int* idx, const char* toAppend) {
  strlcpy(uploadStr + *idx, toAppend, UPLOAD_BUFFER_SIZE);
  *idx += strlen(toAppend);
}

void writeKeyValue(int* idx, const char* key, const char* val) {
  append(idx, key);
  append(idx, "=");
  append(idx, val);
  append(idx, ";");
}

void writeStateToUploadString() {
  char numberStr[15];  // Buffer for conversion from int -> string
  int writeIndex = 0;  // Current location in global uploadStr we are at
  writeKeyValue(&writeIndex, "version", VERSION);
  writeKeyValue(&writeIndex, "sensorId", SENSOR_ID);

  sprintf(numberStr, "%d", WiFi.RSSI());
  writeKeyValue(&writeIndex, "rssi", numberStr);

  const char* sensorStateStr = "ok";
  if (sensorState == SensorState::Ok) {
    sensorStateStr = "ok";
  } else if (sensorState == SensorState::Disconnected) {
    sensorStateStr = "discon";
  } else if (sensorState == SensorState::Erratic || hasBeenErratic) {
    hasBeenErratic = false;  // Reset for next sync period
    sensorStateStr = "erratic";
  }

  writeKeyValue(&writeIndex, "sensor", sensorStateStr);
  sprintf(numberStr, "%d", interval_buffer.size);
  writeKeyValue(&writeIndex, "size", numberStr);
  append(&writeIndex, "values=");
  for (int i = 0; i < interval_buffer.size; i++) {
    sprintf(numberStr, "%d", circular_get(interval_buffer, i));
    append(&writeIndex, numberStr);
    append(&writeIndex, ",");

    if (writeIndex > UPLOAD_BUFFER_SIZE - 20) {
      // If we run out of memory in upload string, then show error and clear buffer
      currentScreen = Screen::Error;
      screenState.error = Error::UploadMem;
      lcdPendingUpdate = true;
      uploadStr[0] = '\0';
      interval_buffer.size = 0;
      return;
    }
  }
}

bool shouldSaveConfig = false;
void onWifiSaveConfig() {
  shouldSaveConfig = true;
}

void onAutoConnectFailed(WiFiManager *_wm) {
  currentScreen = Screen::ConnectToAp;
  lcdPendingUpdate = true;
  lcdRenderLoop();
}

void startWifiPortal(bool autoConnect) {
  WiFiManager wifiManager;
  WiFiManagerParameter param_secret(CFG_SECRET, CFG_SECRET, config.secret, CFG_SIZE_SECRET);
  WiFiManagerParameter param_sensor_id(CFG_SENSOR_ID, CFG_SENSOR_ID, config.sensorId, CFG_SIZE_SENSOR_ID);
  WiFiManagerParameter param_endpoint(CFG_ENDPOINT, CFG_ENDPOINT, config.endpoint, CFG_SIZE_ENDPOINT);
  WiFiManagerParameter param_sync_route(CFG_SYNC_ROUTE, CFG_SYNC_ROUTE, config.syncRoute, CFG_SIZE_SYNC_ROUTE);
  WiFiManagerParameter param_refresh_route(CFG_REFRESH_ROUTE, CFG_REFRESH_ROUTE, config.refreshRoute, CFG_SIZE_REFRESH_ROUTE);
  WiFiManagerParameter param_refresh_period(CFG_REFRESH_PERIOD, CFG_REFRESH_PERIOD, config.refreshPeriodSeconds, CFG_SIZE_REFRESH_PERIOD);
  WiFiManagerParameter param_impluse_kwh(CFG_IMPL_PER_KWH, CFG_IMPL_PER_KWH, config.implusePerKwh, CFG_SIZE_IMPLUSE_KWH);

  if (autoConnect) {
    wifiManager.getWiFiSSID(true).toCharArray(screenState.connectingSSID, 32);
    if (strlen(screenState.connectingSSID) > 0) {
      currentScreen = Screen::ConnectingToWifi;
      Serial.printf("%s\n", screenState.connectingSSID);
      lcdPendingUpdate = true;
      lcdRenderLoop();
    } 
  }

  shouldSaveConfig = false;
  wifiManager.setSaveConfigCallback(onWifiSaveConfig);

  wifiManager.addParameter(&param_secret);
  wifiManager.addParameter(&param_sensor_id);
  wifiManager.addParameter(&param_endpoint);
  wifiManager.addParameter(&param_sync_route);
  wifiManager.addParameter(&param_refresh_route);
  wifiManager.addParameter(&param_refresh_period);
  wifiManager.addParameter(&param_impluse_kwh);
  wifiManager.setConfigPortalTimeout(PORTAL_TIMEOUT_SEC);

  wifiManager.setAPCallback(onAutoConnectFailed);
  // If autoConnect set to true, then we only show portal if connection failed
  // Otherwise, always show the config so that the user can edit the config settings
  if (autoConnect) {
    wifiManager.autoConnect(AP_SSID);
  } else {
    wifiManager.startConfigPortal(AP_SSID);
  }
  Serial.println("Wifi Portal done!");

  if (shouldSaveConfig) {
    strlcpy(config.secret, param_secret.getValue(), CFG_SIZE_SECRET);
    strlcpy(config.sensorId, param_sensor_id.getValue(),CFG_SIZE_SENSOR_ID);
    strlcpy(config.endpoint, param_endpoint.getValue(), CFG_SIZE_ENDPOINT);
    strlcpy(config.syncRoute, param_sync_route.getValue(), CFG_SIZE_SYNC_ROUTE);
    strlcpy(config.refreshRoute, param_refresh_route.getValue(), CFG_SIZE_REFRESH_ROUTE);
    strlcpy(config.refreshPeriodSeconds, param_refresh_period.getValue(), CFG_SIZE_REFRESH_PERIOD);
    strlcpy(config.implusePerKwh, param_impluse_kwh.getValue(), CFG_SIZE_IMPLUSE_KWH);
    Serial.printf("Writing config to flash\n");
    writeConfig(config);
  }

  Error validateError = Error::InvalidConfig;
  if (validateConfig(config, &validateError)) {
    currentScreen = Screen::Hello;
  }

}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Starting setup");

  if (!LittleFS.begin()) {
    LittleFS.format();
  }

  Wire.begin();
  Wire.beginTransmission(LCD_ADDR);
  int error = Wire.endTransmission();
  if (error) {
    Serial.printf(" Failed to find LCD at I2C address %d - error code: %d\n", LCD_ADDR, error);
  }
  lcd.init();
  lcd.clear();
  lcd.backlight();  // Keep on for setup
  lcd.createChar(LCD_CUSTOM_POUND, poundLcdChar);
  lcd.createChar(LCD_CUSTOM_WIFI_VERY_STRONG, wifiStrenghVeryGoodChar);
  lcd.createChar(LCD_CUSTOM_WIFI_GOOD, wifiStrenghGoodChar);
  lcd.createChar(LCD_CUSTOM_WIFI_OK, wifiStrenghOkChar);
  lcd.createChar(LCD_CUSTOM_WIFI_POOR, wifiStrenghPoorChar);

  pinMode(PIN_PULSE, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_D3, INPUT_PULLUP);
  pinMode(PIN_BTN_1, INPUT);
  currentScreen = Screen::Hello;

  config = loadConfig();
  Error validateError;
  if (!validateConfig(config, &validateError)) {
    Serial.println("Invalid config");
    currentScreen = Screen::Error;
    screenState.error = validateError;
    lcdPendingUpdate = true;
  } else {
    startWifiPortal(true);
  }

  lcdPendingUpdate = true;

  digitalWrite(PIN_LED, HIGH);

  attachInterrupt(digitalPinToInterrupt(PIN_D3), ISR_D3_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_1), ISR_D6_high, RISING);

  randomSeed(analogRead(0));
  interval_buffer = circular_init();
  turnOnBacklight();  // Now setup complete, set backlight on timer
  nextUploadMs = 1000 * SYNC_PERIOD_SECONDS;
  Serial.println("Setup completed");
}

bool uploadSamples() {
  noInterrupts();
  unsigned int numToUpload = interval_buffer.size;
  writeStateToUploadString();
  interrupts();
  Serial.printf("Uploading samples %d to %s\n", interval_buffer.size, SYNC_ENDPOINT);

  HTTPClient http;
  http.begin(client, SYNC_ENDPOINT);
  int resp = http.PUT(uploadStr);
  Serial.printf("Upload HTTP response code %d\n", resp);
  http.end();
  int success = resp >= 200 && resp <= 299;
  if (success) {
    // If uploaded ok, then remove from circular buffer by removing count
    // This ensure that any written during upload will still be kept
    noInterrupts();
    const int newSize = interval_buffer.size - numToUpload;
    interval_buffer.size = newSize <= 0 ? 0 : newSize;  // Should not be negative, but good to check
    interrupts();
  }
  return success;
}

bool refresh() {
  HTTPClient http;
  http.begin(client, REFRESH_ENDPOINT);
  int resp = http.GET();
  Serial.printf("Refresh endpoint returned status %d\n", resp);
  if (resp != 200) {
    return false;
  }

  // dynamic string - hopefully not an issue for heap fragmentation
  http.getString().toCharArray(refreshStr, REFRESH_SIZE);
  parseResponse(&refreshResult, refreshStr);

  if (strlen(refreshResult.updateUrl) > 0) {
    performOta();
  }

  return true;
}

void parseResponse(struct refresh_result_t* response, char* response_str) {
  int position = 0;
  while (position < strlen(response_str)) {
    if (startsWith("updateUrl=", response_str + position)) {
      position += strlen("updateUrl=");
      readUntilNewline(response_str, &position, response->updateUrl, 64);
    } else {
      break;
    }
  }
}

int calculateCurrentPowerWatts(int periodMs) {
  if (periodMs == 0) {
    return 0;
  }
  double periodSeconds = (double)periodMs / 1000.0;
  return 1000 * (3600 / periodSeconds) / IMPL_PER_KWH;
}

void loop() {
  lcdRenderLoop();
  unsigned long currentMillis = millis();
  unsigned long diff = currentMillis - prevMillis;
  if (diff >= nextInterval) {
    if (ledState == STATE_LED_ON) {
      digitalWrite(PIN_LED, LOW);
      digitalWrite(PIN_PULSE, LOW);
      nextInterval = random(500, 3000);
      ledState = STATE_LED_OFF;
      Serial.printf("LED off for %d\n", nextInterval + PULSE_INTERVAL_MS);
    } else if (ledState == STATE_LED_OFF) {
      digitalWrite(PIN_LED, HIGH);
      digitalWrite(PIN_PULSE, HIGH);
      nextInterval = PULSE_INTERVAL_MS;
      ledState = STATE_LED_ON;
    }
    prevMillis = millis();
  }

  unsigned long now = millis();
  // TODO upload on buffer nearing capacity
  if (now >= nextUploadMs) {
    if (uploadSamples()) {
      nextUploadMs = now + 1000 * SYNC_PERIOD_SECONDS;
      failureCount = 0;

    } else {
      // On failure, retry sooner - just do multiplicative backoff capped at 5s
      failureCount += 1;
      const int backoffCount = failureCount > 10 ? failureCount : failureCount;
      nextUploadMs = millis() + (backoffCount * 500);
    }
    nextRefresh = (unsigned long)millis() + REFRESH_AFTER_SYNC_PERIOD_MS;
  }

  if (now >= nextRefresh) {
    Serial.println("Refreshing");
    nextRefresh = (unsigned long)millis() + 100000000;
    if (refresh()) {
      Serial.printf("Refresh OK! updateUrl=%s\n", refreshResult.updateUrl);
    } else {
      // TODO
      Serial.printf("Refresh failed!\n");
    }
  }

  if (btn1Pressed) {
    btn1Pressed = false;
    Serial.printf("Button 1 Pressed\n");
    screenState.buttonCount++;
    lcdPendingUpdate = true;
    // Only move screen if backlgiht is on - o/w first press should only turn backlight on
    if (backlightIsOn) {
      if (currentScreen == Screen::Hello) {
        currentScreen = Screen::CurrentAndDay;
      } else if (currentScreen == Screen::CurrentAndDay) {
        currentScreen = Screen::CurrentAndMonth;
      } else if (currentScreen == Screen::CurrentAndMonth) {
        currentScreen = Screen::Hello;
      } else if (currentScreen == Screen::Error) {
        // TODO Need to determine what to do with each error 
        if (isConfigError(screenState.error)) {
          currentScreen = Screen::ConnectToAp;
          lcdPendingUpdate = true;
          lcdRenderLoop();
          startWifiPortal(false);
        }
      }
      screenState.todayPounds = random(0, 300);
      screenState.todayPence = random(0, 99);
      screenState.monthPounds = random(400, 1000);
      screenState.monthPence = random(0, 20);
      lcdPendingUpdate = true;
    }
    turnOnBacklight();
  }

  if (millis() >= nextCheckWifiStatusAt) {
    // Update Wifi signal strength that is shown on LCD
    const WifiStrength newStrength = getWiFiStrength();
    // Check if difference to prevent unneeded lcd updates
    if (newStrength != screenState.wifiStrength) {
      screenState.wifiStrength = newStrength;
      lcdPendingUpdate = true;
    }
    nextCheckWifiStatusAt = millis() + WIFI_STATUS_UPDATE_PERIOD_MS;
  }

  if (backlightIsOn && millis() > turnBacklightOffAt) {
    backlightIsOn = false;
    lcd.noBacklight();
  }

  if (currentUpdated) {
    currentUpdated = false;
    if (interval_buffer.size >= 1) {
      unsigned int newWatts = calculateCurrentPowerWatts(lastDurationUs / 1000);
      if (newWatts != screenState.currentWatts) {
        screenState.currentWatts = newWatts;
        lcdPendingUpdate = true;
      }
    }
  }
}


WifiStrength getWiFiStrength() {
  int rssi = WiFi.RSSI();
  if (rssi >= -70) {
    return WifiStrength::VeryGood;
  } else if (rssi >= -80 && rssi <= -71) {
    return WifiStrength::Good;
  } else if (rssi >= -90 && rssi <= -81) {
    return WifiStrength::Ok;
  } else {
    return WifiStrength::Poor;
  }
}

void ICACHE_RAM_ATTR d3Rising() {
  unsigned long now = micros();
  if (lastPulseMicros == -1) {
    // First pulse since startup
    lastPulseMicros = now;
    return;
  }
  lastPulseDurationMicros = now - lastPulseMicros;
  lastPulseMicros = now;
}


void ICACHE_RAM_ATTR d3Falling() {
  // Check duration pulse was high for
  // If too long, then mark as eratic
  const unsigned long highDuration = micros() - lastPulseMicros;
  if (highDuration <= MAX_DURATION_MICROS || lastPulseMicros == -1) {
    if (sensorState == SensorState::Ok) {
      interval_buffer = circular_add(interval_buffer, lastPulseDurationMicros);
      lastDurationUs = lastPulseDurationMicros;
      currentUpdated = true;
    } else if (sensorState == SensorState::Disconnected) {
      sensorState = SensorState::Ok;
    } else if (sensorState == SensorState::Erratic) {
      normalityCount += 1;
      if (normalityCount >= RETURN_TO_NORMALITY_TRESH) {
        normalityCount = 0;
        sensorState = SensorState::Ok;
      }
    }
  } else {
    normalityCount = 0;
    sensorState = SensorState::Ok;
    hasBeenErratic = true;
  }
}

void ICACHE_RAM_ATTR ISR_D3_change() {
  if (digitalRead(PIN_D3)) {
    d3Rising();
  } else {
    d3Falling();
  }
}


void ICACHE_RAM_ATTR ISR_D6_high() {
  btn1Pressed = true;
}

void lcdConnectingToWifi() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Connecting to");
  lcd.setCursor(6, 1);
  lcd.print("WiFi");
}


void lcdConnectingToWifi(char* ssid) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Connecting to");

  clampString(ssid, 16);
  char bottomLine[17];
  centerString(bottomLine, 16, ssid);
  lcd.setCursor(0, 1);
  lcd.print(bottomLine);
}

void lcdHello() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("     Updated!");

  lcd.setCursor(0, 1);
  char bottom[16];
  int x = 0;
  intToString(bottom, 16, &x, screenState.buttonCount, "--", 10);
  lcd.print(bottom);
}

int specialCharForWifiStrength(WifiStrength strength) {
  if (strength == WifiStrength::VeryGood) {
    return LCD_CUSTOM_WIFI_VERY_STRONG;
  } else if (strength == WifiStrength::Good) {
    return LCD_CUSTOM_WIFI_GOOD;
  } else if (strength == WifiStrength::Ok) {
    return LCD_CUSTOM_WIFI_OK;
  } else if (strength == WifiStrength::Poor) {
    return LCD_CUSTOM_WIFI_POOR;
  }
  // TODO disconnected
  return 1;
}

void lcdCurrentUsage(WifiStrength strength, const char* periodString, int watts, int todayPounds, int todayPence) {
  char topLine[16];
  char finalTopLine[17];
  int position = 0;
  strlcpy(topLine + position, "Now ", 15);
  position += 4;
  intToString(topLine, 15, &position, watts, ">99999", 5);
  strlcpy(topLine + position, "W", 15);
  centerString(finalTopLine, 15, topLine);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(specialCharForWifiStrength(strength));
  lcd.print(finalTopLine);

  char bottomLine[17];
  char finalBottomLine[17];
  strcpy(bottomLine, periodString);
  position = strlen(periodString);
  strcpy(bottomLine + position, " \243");
  position += 2;
  intToString(bottomLine, 16, &position, todayPounds, "---", 4);
  const char* sepStr = todayPence < 10 ? ".0" : ".";
  strcpy(bottomLine + position, sepStr);
  position += strlen(sepStr);
  intToString(bottomLine, 16, &position, todayPence, "--", 2);
  centerString(finalBottomLine, 16, bottomLine);
  lcdWriteWithSpecials(finalBottomLine, 0, 1);
}

void lcdConnectToAP() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connect to WiFi");
  lcd.setCursor(0, 1);

  char bottomLine[17];
  char finalBottomLine[17];
  strncpy(bottomLine, AP_SSID, 16);
  centerString(finalBottomLine, 16, bottomLine);
  lcd.setCursor(0, 1);
  lcd.print(finalBottomLine);
}

void lcdError(Error error) {
  char topLine[17];
  strcpy(topLine, "ERROR           ");
  strncpy(topLine + 13, errorCode(error), 3);
  char bottomLine[17];
  strncpy(bottomLine, errorMessage(error), 16);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(topLine);
  lcd.setCursor(0, 1);
  lcd.print(bottomLine);
}

void lcdUpdating(int updatePercent) {
  char topLine[17];
  strcpy(topLine, "   Updating...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(topLine);
  char numStr[3];
  int position = 0;
  char bottomLine[17];
  intToString(bottomLine, 16, &position, updatePercent, "??", 3);
  strcpy(bottomLine + position, "%");
  char finalBottomLine[17];
  centerString(finalBottomLine, 16, bottomLine);
  lcd.setCursor(0, 1);
  lcd.print(finalBottomLine);
}

void lcdWriteWithSpecials(char* string, int startCol, int row) {
  char tmp[2];
  for (int col = startCol; col < strlen(string) + startCol; col++) {
    lcd.setCursor(col, row);
    if (string[col] == '\243') {
      lcd.write(LCD_CUSTOM_POUND);
    } else {
      strncpy(tmp, string + col, 1);
      tmp[1] = '\0';
      lcd.print(tmp);
    }
  }
}

void lcdUpdateOk() {
  char line[17];
  strcpy(line, "    Update OK");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line);
  strcpy(line, "  Restarting...");
  lcd.setCursor(0, 1);
  lcd.print(line);
}


void lcdRenderLoop() {
  if (!lcdPendingUpdate) {
    return;
  }
  switch (currentScreen) {
    case Screen::Hello:
      lcdHello();
      break;
    case Screen::CurrentAndDay:
      lcdCurrentUsage(screenState.wifiStrength, "Day", screenState.currentWatts, screenState.todayPounds, screenState.todayPence);
      break;
    case Screen::CurrentAndMonth:
      lcdCurrentUsage(screenState.wifiStrength, "Month", screenState.currentWatts, screenState.monthPounds, screenState.monthPence);
      break;
    case Screen::ConnectingToWifi:
      lcdConnectingToWifi(screenState.connectingSSID);
      break;
    case Screen::ConnectToAp:
      lcdConnectToAP();
      break;
    case Screen::Error:
      lcdError(screenState.error);
      break;
    case Screen::Updating:
      lcdUpdating(screenState.updatePercent);
      break;
    case Screen::UpdateOk:
      lcdUpdateOk();
      break;
  }
  lcdPendingUpdate = false;
}

void otaStarted() {
  Serial.println("OTA started");
  otaNextUpdateMs = millis();
}

void otaFinished() {
  Serial.println("OTA completed");
  currentScreen = Screen::UpdateOk;
  lcdPendingUpdate = true;
  lcdRenderLoop();
}

void otaProgress(int cur, int total) {
  Serial.printf("OTA update at %d of %d bytes...\n", cur, total);
  if (total != 0) {
    currentScreen = Screen::Updating;
    screenState.updatePercent = 100 * (1.0 * cur / total);

    // Don't update the lcd too often - otherwise not legible due to slow refresh time
    if (screenState.updatePercent > 95 || millis() > otaNextUpdateMs) {
      lcdPendingUpdate = true;
      lcdRenderLoop();  // Have to call this as ota blocks main loop
      otaNextUpdateMs = millis() + OTA_LCD_REFRESH_PERIOD_MS;
    }
  }
}

void otaFailed(int err) {
  Serial.printf("OTA update fatal error code %d\n", err);
  currentScreen = Screen::Error;
  screenState.error = Error::UpdateFailed;
  lcdPendingUpdate = true;
}


void performOta() {
  LittleFS.end();
  if (strlen(refreshResult.updateUrl) <= 0) {
    Serial.println("Skipping OTA update as no updateUrl provided");
    return;
  }

  ESPhttpUpdate.onStart(otaStarted);
  ESPhttpUpdate.onEnd(otaFinished);
  ESPhttpUpdate.onProgress(otaProgress);
  ESPhttpUpdate.onError(otaFailed);
  ESPhttpUpdate.rebootOnUpdate(false);

  HTTPUpdateResult otaRet = ESPhttpUpdate.update(client, refreshResult.updateUrl);
  if (otaRet == HTTP_UPDATE_OK) {
    delay(500);  // Give time to read LCD
    ESP.restart();
  } else {
    Serial.println("OTA failed");
  }
}

bool cfg_write(const char* name, char* value) {
  File file = LittleFS.open(name, "w+");
  if (!file) {
    currentScreen = Screen::Error;
    screenState.error = Error::FileOpenWFail;
    lcdPendingUpdate = true;
    return false;
  }
  int numWritten = file.print(value);
  Serial.printf("Wrote %d\n", numWritten);

  if (numWritten != strlen(value)) {
    currentScreen = Screen::Error;
    screenState.error = Error::FileWriteFail;
    lcdPendingUpdate = true;
    return false;
  }
  file.close();
  Serial.printf("Wrote %s to %s\n", value, name);
  return true;
}

bool cfg_read(const char* name, char* value, int maxLen) {
  File file = LittleFS.open(name, "r");
  if (!file) {
    currentScreen = Screen::Error;
    screenState.error = Error::FileOpenRFail;
    lcdPendingUpdate = true;
    value[0] = '\0';
    return false;
  }
  int i = 0;
  for (; i < maxLen - 1; i++) {
    if (!file.available()) {
      break;
    } else {
      value[i] = file.read();
    }
  }
  value[i] = '\0';
  Serial.printf("Read %s from %s\n", value, name);
  return true;
}

// To avoid having to do much parsing, each config is stored in a separate
// file, named the same as its CFG_* name
struct config_t loadConfig() {
  struct config_t config;
  cfg_read(CFG_SECRET, config.secret, CFG_SIZE_SECRET);
  cfg_read(CFG_SENSOR_ID, config.sensorId, CFG_SIZE_SENSOR_ID);
  cfg_read(CFG_ENDPOINT, config.endpoint, CFG_SIZE_ENDPOINT);
  cfg_read(CFG_SYNC_ROUTE, config.syncRoute, CFG_SIZE_SYNC_ROUTE);
  cfg_read(CFG_REFRESH_ROUTE, config.refreshRoute, CFG_SIZE_REFRESH_ROUTE);
  cfg_read(CFG_REFRESH_PERIOD, config.refreshPeriodSeconds, CFG_SIZE_REFRESH_PERIOD);
  cfg_read(CFG_IMPL_PER_KWH, config.implusePerKwh, CFG_SIZE_IMPLUSE_KWH);

  char* tmp;
  refreshPeriodSeconds = strtol(config.refreshPeriodSeconds, &tmp, 10);
  implusePerKwh = strtol(config.implusePerKwh, &tmp, 10);
  return config;
}

void writeConfig(struct config_t config) {
  Serial.printf("Writing %d %d\n", refreshPeriodSeconds, implusePerKwh);
  Error validateError = Error::InvalidConfig;
  if (!validateConfig(config, &validateError)) {
    currentScreen = Screen::Error;
    screenState.error = validateError;
    lcdPendingUpdate = true;
  }
  cfg_write(CFG_SECRET, config.secret);
  cfg_write(CFG_SENSOR_ID, config.sensorId);
  cfg_write(CFG_ENDPOINT, config.endpoint);
  cfg_write(CFG_SYNC_ROUTE, config.syncRoute);
  cfg_write(CFG_REFRESH_ROUTE, config.refreshRoute);
  cfg_write(CFG_REFRESH_PERIOD, config.refreshPeriodSeconds);
  cfg_write(CFG_IMPL_PER_KWH, config.implusePerKwh);
  char* tmp;
  refreshPeriodSeconds = strtol(config.refreshPeriodSeconds, &tmp, 10);
  implusePerKwh = strtol(config.implusePerKwh, &tmp, 10);
}

bool validateConfig(struct config_t config, Error* error) {
  if (strlen(config.secret) <= 0) {
    *error = Error::InvalidConfigSecret;
    return false;
  }
  if (strlen(config.sensorId) <= 0) {
    *error = Error::InvalidConfigSensorId;
    return false;
  }
  if (strlen(config.endpoint) <= 0) {
    *error = Error::InvalidConfigEndpoint;
    return false;
  }
  if (strlen(config.syncRoute) <= 0) {
    *error = Error::InvalidConfigSyncRoute;
    return false;
  }
  if (strlen(config.refreshRoute) <= 0) {
    *error = Error::InvalidConfigRefreshRoute;
    return false;
  }
  if (refreshPeriodSeconds > 3600) {
    *error = Error::InvalidConfigRefreshPeriod;
    return false;
  }

  if (implusePerKwh <= 0) {
    *error = Error::InvalidConfigImplKwh;
    return false;
  }
  return true;
}