#define VERSION "v0.0.1"
#define SENSOR_ID "dev"
#define SYNC_PERIOD_SECONDS 2 // TODO set this!
#define SENSOR_PIN 13
#define AP_SSID "EnergyMonitor"
#define LCD_ADDR 0x27
#define IMPL_PER_KWH 1000
#define PORTAL_TIMEOUT_SEC 600
// Test mode - this just flashes the LED on pin 14 to check a second device
#define TEST_MODE false
// Send requests to local endpoints (defined below) instead of cloud run
#define LOCAL_SERVER true

#if (LOCAL_SERVER == true)
#undef SYNC_ENDPOINT
#undef REFRESH_ENDPOINT
#endif 

#ifndef SYNC_ENDPOINT
#define SYNC_ENDPOINT "http://192.168.1.84:8090/v1/sync"
#endif 

#ifndef REFRESH_ENDPOINT
#define REFRESH_ENDPOINT "http://192.168.1.84:8090/v1/refresh"
#endif
