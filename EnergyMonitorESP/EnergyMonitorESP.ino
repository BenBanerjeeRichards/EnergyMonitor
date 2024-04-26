#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiManager.h>
#include "config.h"
#include "FS.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

const int CAPACITY = 100;
const int UPLOAD_BUFFER_SIZE = 6 * CAPACITY;

int mod(int a, int b) {
    int r = a % b;
    return r < 0 ? r + b : r;
}

struct circular_buffer {
    volatile unsigned int buffer[CAPACITY];
    volatile unsigned int writeIndex;
    volatile unsigned int size;
};

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
volatile const int DEBOUNCE_THRESH_MS = 100;        // Any inputs in this period are ignored 
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

void ICACHE_RAM_ATTR ISR_D3_change();
void ICACHE_RAM_ATTR ISR_D6_high();


struct circular_buffer interval_buffer;


struct circular_buffer circular_init() {
    struct circular_buffer buf;
    buf.writeIndex = 0;
    buf.size = 0;
    return buf;
}

struct circular_buffer circular_add(struct circular_buffer buf, unsigned int item) {
    buf.buffer[buf.writeIndex] = item;
    buf.writeIndex = mod((buf.writeIndex + 1), CAPACITY);

    if (buf.size < CAPACITY) {
        buf.size++;
    }
    return buf;
}


// Get items where index 0 is latest item added to buffer and CAPACITY-1 is the last item
unsigned int circular_get(struct circular_buffer buf, int idx) {
    return buf.buffer[mod((buf.writeIndex - (idx + 1)),CAPACITY)];
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


LiquidCrystal_I2C lcd(0x27,16,2); 

void setup() {
  Serial.begin(SERIAL_BAUD);

  Serial.printf("Setting up I2C display\n");
  Wire.begin();
  Wire.beginTransmission(0x27);
  int error = Wire.endTransmission();
  Serial.printf("Wire Error %d\n", error);
  lcd.init();
  lcd.clear();         
  lcd.backlight(); 
  

  pinMode(PIN_PULSE, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_D3, INPUT_PULLUP);
  pinMode(PIN_BTN_1, INPUT);

  lcdCurrentUsage(320, 10, 10);
  // lcdConnectingToWifi();
  Serial.printf("Wifi Manager\n");
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(600);
  wifiManager.autoConnect(AP_SSID, AP_PASS);


  Serial.write("Connected\n");

  digitalWrite(PIN_LED, HIGH);

  attachInterrupt(digitalPinToInterrupt(PIN_D3), ISR_D3_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_1), ISR_D6_high, RISING);

  randomSeed(analogRead(0));
  interval_buffer = circular_init();
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
    // Debounce - don't register as new press
  if (now - btn1LastPressedMs > DEBOUNCE_THRESH_MS) {
    btn1LastPressedMs = now;
    btn1Pressed = true;
  }
}

void lcdConnectingToWifi() {
  lcd.clear();
  lcd.setCursor(1,0);   
  lcd.print("Connecting to");
  lcd.setCursor(6,1);   
  lcd.print("WiFi");
}

void clampString(char* string, int max) {
  if (strlen(string) > max) {
    // Two dots to save on space
    string[max-2] = '.';
    string[max-1] = '.';
    string[max] = '\0';
  }
}

int intToString(char* string, int stringSize, int* stringOffset, int number, const char* fallback, int maxNumLength) {
    int sizeNeeded = snprintf(string+*stringOffset, stringSize, "%d", number);
    if (sizeNeeded > maxNumLength) {
        strcpy(string+*stringOffset, fallback);
        *stringOffset += strlen(fallback);
        return 1;
    } else {
        *stringOffset += sizeNeeded;
        return 0;
    }
}

void centerString(char* outputString, int width, char* stringToCenter) {
  int inputLen = strlen(stringToCenter);
  int diff = width - inputLen;
  if (diff < 0) {
    return;
  }
  int leftPad = diff / 2;
  for (int i = 0; i < leftPad; i++) {
    strlcpy(outputString+i, " ", width);
  }
  strlcpy(outputString+leftPad, stringToCenter, width);
}

void lcdCurrentUsage(int watts, int todayPounds, int todayPence) {
  char wattsStr[6]; 
  char poundsStr[5]; 
  char penceStr[2];

  // poundsStr = "--";
  // penceStr = "--";
  char topLine[16];
  char finalTopLine[16];
  int position = 0;
  strlcpy(topLine+position, "Now ", 16);
  position += 4;
  intToString(topLine, 16, &position, watts, ">999", 3);
  strlcpy(topLine+position, "W", 16);
  centerString(finalTopLine, 16, topLine);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(finalTopLine);

  lcd.setCursor(0, 1);
  char s[20];
  strcpy(s, "abcdefghijklmnop");
  clampString(s, 16);
  lcd.print(s);
}