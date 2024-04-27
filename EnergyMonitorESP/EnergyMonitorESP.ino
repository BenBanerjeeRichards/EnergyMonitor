#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiManager.h>
#include "config.h"
#include "FS.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "Circular.h"
#include "StringUtil.h"

const int UPLOAD_BUFFER_SIZE = 6 * CAPACITY;


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
enum class SensorState {Ok, Erratic, Disconnected}; 

const unsigned long MAX_DURATION_MICROS = 200 * 1000;
const unsigned int RETURN_TO_NORMALITY_TRESH = 3; // Wait for this number of normal measurements until OK
bool hasBeenErratic = false;                      // Have we been erratic in this sync period? Set so we can notify API
int normalityCount = 0;                           // When in erratic mode, how many consequative OK measurements have we had
SensorState sensorState = SensorState::Disconnected;       // Disconnected until proven otherwise

// Button debounce and state 
const int DEBOUNCE_THRESH_MS = 150;        // Any inputs in this period are ignored 
bool btn1Pressed = false;                           // becomes true until button press event handled 
volatile unsigned long btn1LastPressedMs = 0;       // For software debounce

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

// Update wifi stength info 
const unsigned int WIFI_STATUS_UPDATE_PERIOD_MS = 5000;
unsigned long nextCheckWifiStatusAt = 0;

enum class WifiStrength {
  VeryGood, Good, Ok, Poor, Disconnected
};

// State for rendering to the display 
enum class Screen { 
  Hello, 
  ConnectToAp,        // Instruct user to connect to the AP for setup
  ConnectingToWifi,   // Connecting to a specific (already saved) AP
  CurrentAndDay,      // Show current usage in W and the total used this day 
  CurrentAndMonth     // Same as above, but show the billing period (month)
};

Screen currentScreen = Screen::Hello;   // Current screen to be dislpayed
bool lcdPendingUpdate = true;              // true when we are pending an update to lcd display
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

};
// Backlight - turn off when inactive
const unsigned int BACKLIGHT_TIMEOUT_MS = 30 * 1000;
bool backlightIsOn = false;
unsigned int turnBacklightOffAt;

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

byte wifiStrenghVeryGoodChar[8] {
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


struct screen_state_t screenState;

void ICACHE_RAM_ATTR ISR_D3_change();
void ICACHE_RAM_ATTR ISR_D6_high();


struct circular_buffer interval_buffer;
WifiStrength getWiFiStrength();

LiquidCrystal_I2C lcd(0x27,16,2); 

void turnOnBacklight() {
  if (!backlightIsOn) {
    lcd.backlight();
    backlightIsOn = true;
  }
  turnBacklightOffAt = millis() + BACKLIGHT_TIMEOUT_MS;
}

void append(int *idx, const char* toAppend) {
    strlcpy(uploadStr+*idx, toAppend, UPLOAD_BUFFER_SIZE);
    *idx += strlen(toAppend);
}

void writeKeyValue(int* idx, const char* key, const char* val) {
    append(idx, key);
    append(idx, "=");
    append(idx, val);
    append(idx, ";");
}

void writeStateToUploadString() {
    char numberStr[15];     // Buffer for conversion from int -> string
    int writeIndex = 0;   // Current location in global uploadStr we are at 
    writeKeyValue(&writeIndex,"version", VERSION);
    writeKeyValue(&writeIndex,"tenantId", TENANT_ID);

    sprintf(numberStr, "%d", WiFi.RSSI());
    writeKeyValue(&writeIndex,"rssi", numberStr);

    const char* sensorStateStr = "ok";
    if (sensorState == SensorState::Ok) {
      sensorStateStr = "ok";
    } else if (sensorState == SensorState::Disconnected) {
      sensorStateStr = "discon";
    } else if (sensorState == SensorState::Erratic || hasBeenErratic) {
      hasBeenErratic = false; // Reset for next sync period
      sensorStateStr = "erratic";
    }

    writeKeyValue(&writeIndex,"sensor", sensorStateStr);
    sprintf(numberStr, "%d", interval_buffer.size);
    writeKeyValue(&writeIndex,"size", numberStr);
    append(&writeIndex, "values=");
    for (int i =0; i < interval_buffer.size; i++) {
        sprintf(numberStr, "%d", circular_get(interval_buffer, i));
        append(&writeIndex, numberStr);
        append(&writeIndex, ",");
    }
}



void setup() {
  Serial.begin(SERIAL_BAUD);

  Serial.printf("Setting up I2C display\n");
  Wire.begin();
  Wire.beginTransmission(0x27);
  int error = Wire.endTransmission();
  Serial.printf("Wire Error %d\n", error);
  lcd.init();
  lcd.clear();         
  lcd.backlight(); // Keep on for setup
  lcd.createChar(LCD_CUSTOM_POUND, poundLcdChar);
  lcd.createChar(LCD_CUSTOM_WIFI_VERY_STRONG, wifiStrenghVeryGoodChar);
  lcd.createChar(LCD_CUSTOM_WIFI_GOOD, wifiStrenghGoodChar);
  lcd.createChar(LCD_CUSTOM_WIFI_OK, wifiStrenghOkChar);
  lcd.createChar(LCD_CUSTOM_WIFI_POOR, wifiStrenghPoorChar);

  pinMode(PIN_PULSE, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_D3, INPUT_PULLUP);
  pinMode(PIN_BTN_1, INPUT_PULLUP);

  Serial.printf("Wifi Manager\n");
  WiFiManager wifiManager;

  Serial.println(wifiManager.getWiFiSSID(true));
  wifiManager.getWiFiSSID(true).toCharArray(screenState.connectingSSID, 32);
  if (strlen(screenState.connectingSSID) > 0) {
    // Connecting to existing
    currentScreen = Screen::ConnectingToWifi;
    Serial.printf("%s\n", screenState.connectingSSID);
    lcdPendingUpdate = true;
    lcdRenderLoop();  // Do manually here as outside main loop
  } else {
    // TODO 
  }

  wifiManager.setConfigPortalTimeout(600);
  wifiManager.autoConnect(AP_SSID, AP_PASS);
  currentScreen = Screen::Hello;
  lcdPendingUpdate = true;

  Serial.write("Connected\n");

  digitalWrite(PIN_LED, HIGH);

  attachInterrupt(digitalPinToInterrupt(PIN_D3), ISR_D3_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_1), ISR_D6_high, FALLING);

  randomSeed(analogRead(0));
  interval_buffer = circular_init();
  turnOnBacklight();  // Now setup complete, set backlight on timer
}

bool uploadSamples() {
  noInterrupts();
  unsigned int numToUpload = interval_buffer.size;
  writeStateToUploadString();
  interrupts();

  Serial.printf("Uploading samples %d\n", interval_buffer.size);
       
  WiFiClient client;
  HTTPClient http;
  http.begin(client, SYNC_ENDPOINT);
  int resp = http.PUT(uploadStr);
  Serial.printf("Upload HTTP response code %d\n", resp);
  http.end();
  int success =  resp >= 200 && resp <= 299;
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

void loop() {
  lcdRenderLoop();
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis >= nextInterval) {
    if (ledState == STATE_LED_ON) {
      digitalWrite(PIN_LED, LOW);
      digitalWrite(PIN_PULSE, LOW);
      nextInterval = random(500, 2000);
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
  if (interval_buffer.size > 0.6 * CAPACITY || now >= nextUploadMs) {
    if (uploadSamples()) {
      nextUploadMs = now + 1000 * SYNC_PERIOD_SECONDS;
      failureCount = 0;
      
    } else {
      // On failure, retry sooner - just do multiplicative backoff capped at 5s
      failureCount += 1;
      const int backoffCount = failureCount > 10 ? failureCount : failureCount;
      nextUploadMs = millis() + (backoffCount * 500);
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
      }
      screenState.currentWatts = random(100, 10000);
      screenState.todayPounds = random(0, 300);
      screenState.todayPence = random(0, 99);
      screenState.monthPounds = random(400, 1000);
      screenState.monthPence = random(0, 20);
      lcdPendingUpdate = true;
    }
    turnOnBacklight();
  }

  if (millis() >= nextCheckWifiStatusAt) {
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
  unsigned long now = millis();
  if (now - btn1LastPressedMs > DEBOUNCE_THRESH_MS) {
    btn1Pressed = true;
    btn1LastPressedMs = now;  // Time debounce from trigger
  }
}

void lcdConnectingToWifi() {
  lcd.clear();
  lcd.setCursor(1,0);   
  lcd.print("Connecting to");
  lcd.setCursor(6,1);   
  lcd.print("WiFi");
}


void lcdConnectingToWifi(char* ssid) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(" Connecting to");

  clampString(ssid, 16);
  char bottomLine[17];
  centerString(bottomLine, 16, ssid);
  lcd.setCursor(0,1);
  lcd.print(bottomLine);
}

void lcdHello() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("     Hello");

  lcd.setCursor(0, 1);
  char bottom[16];
  int x =0 ;
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
  }\
  // TODO disconnected
  return 1;
}

void lcdCurrentUsage(WifiStrength strength, const char* periodString, int watts, int todayPounds, int todayPence) {
  char topLine[16];
  char finalTopLine[17];
  int position = 0;
  strlcpy(topLine+position, "Now ", 15);
  position += 4;
  intToString(topLine, 15, &position, watts, ">99999", 5);
  strlcpy(topLine+position, "W", 15);
  centerString(finalTopLine, 15, topLine);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.write(specialCharForWifiStrength(strength));
  lcd.print(finalTopLine);

  char bottomLine[17];
  char finalBottomLine[17];
  strcpy(bottomLine, periodString);
  position = strlen(periodString);
  strcpy(bottomLine+position, " \243");
  position += 2;
  intToString(bottomLine, 16, &position, todayPounds, "---", 4);
  const char* sepStr = todayPence < 10 ? ".0" : ".";
  strcpy(bottomLine+position, sepStr);
  position += strlen(sepStr);
  intToString(bottomLine, 16, &position, todayPence, "--", 2);
  centerString(finalBottomLine, 16, bottomLine);
  Serial.printf("%s|%s\n", bottomLine, finalBottomLine);
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

void lcdWriteWithSpecials(char* string, int startCol, int row) {
  char tmp[2];
  for (int col = startCol; col < strlen(string)+startCol; col++) {
    lcd.setCursor(col, row);
    if (string[col] == '\243') {
      lcd.write(LCD_CUSTOM_POUND);
    } else {
      strncpy(tmp, string+col, 1);
      tmp[1] = '\0';
      lcd.print(tmp);
    }
  }

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

  }
  lcdPendingUpdate = false;
}
