#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include "config.h"

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
const int PIN_D1 = 5;
const int PIN_D3 = 0;
const int PIN_LED = 2;  // internal LED
const unsigned int PULSE_INTERVAL_MS = 50;

// State machine for pulsing LED
const int STATE_LED_OFF = 0;
const int STATE_LED_ON = 1;
int ledState = STATE_LED_OFF;

unsigned long prevMillis = 0;
unsigned long nextInterval = 1000;
unsigned long lastUploadMs = 0;
char uploadStr[UPLOAD_BUFFER_SIZE] = "";
unsigned long nextUploadMs = 1000 * SYNC_PERIOD_SECONDS;
unsigned int failureCount = 0;

volatile unsigned long lastPulseMicros = -1;

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
    
    sprintf(numberStr, "%d", interval_buffer.size);
    writeKeyValue(&writeIndex,"size", numberStr);
    append(&writeIndex, "values=");
    for (int i =0; i < interval_buffer.size; i++) {
        sprintf(numberStr, "%d", circular_get(interval_buffer, i));
        append(&writeIndex, numberStr);
        append(&writeIndex, ",");
    }
}


void ICACHE_RAM_ATTR ISR_D3_high();

void setup() {
  pinMode(PIN_D1, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_D3, INPUT_PULLUP);
  Serial.begin(SERIAL_BAUD);

  // Connect to network to verify wifi credentials 
  Serial.write("Connecting to network..\n");
  WiFi.hostname("EnergyMonitor");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    wifiLedPattern();
  }
  Serial.write("Connected\n");
  digitalWrite(PIN_LED, HIGH);

  attachInterrupt(digitalPinToInterrupt(PIN_D3), ISR_D3_high, RISING);
  randomSeed(analogRead(0));
  interval_buffer = circular_init();
}

void wifiLedPattern() {
    digitalWrite(PIN_LED, LOW);
    delay(50);
    digitalWrite(PIN_LED, HIGH);
    delay(100);
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
      digitalWrite(PIN_D1, LOW);
      nextInterval = random(500, 2000);
      ledState = STATE_LED_OFF;
      Serial.printf("LED off for %d\n", nextInterval + PULSE_INTERVAL_MS);
    } else if (ledState == STATE_LED_OFF) {
      digitalWrite(PIN_LED, HIGH);
      digitalWrite(PIN_D1, HIGH);
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
}


void ICACHE_RAM_ATTR ISR_D3_high() {
  unsigned long now = micros();
  if (lastPulseMicros == -1) {
    // First pulse since startup
    lastPulseMicros = now;
    return;
  }
  int duration = now - lastPulseMicros;
  lastPulseMicros = now;
  interval_buffer = circular_add(interval_buffer, duration);
}
