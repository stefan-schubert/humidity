#define PJON_PACKET_MAX_LENGTH 32

#include <config.h>
#include <fonts.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include "SoftwareSerial.h"
#include "Wire.h"
#include "PJON.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ST7735.h"
#include "U8g2_for_Adafruit_GFX.h"
#include "PubSubClient.h"

// DEBUG flag
// #define DEBUG

// pin config
#define RX_PIN D1
#define TX_PIN D4
#define ID 45
#define BUZZER_PIN D6
#define BUTTON_PIN D2
#define TFT_RST -1
#define TFT_CS D8
#define TFT_DC D3
#define TFT_LED D0

// colors
#define GREEN 0x34C0
#define YELLOW 0xFE60
#define RED 0xC980
#define BLUE 0x5E1B

// ubidots
#define SERVER "industrial.api.ubidots.com"
#define SERVER_PORT 8883

// constants
const unsigned int MAX_HUMIDITY = 70;                  // 70.00%
const unsigned int WARN_HUMIDITY = 50;                 // 50.00 %
const unsigned long MAX_AGE = 60000;                   // 1min
const unsigned long MAX_AGE_CLOUD = 300000;            // 5min
const unsigned long DEBOUNCE = 20;                     // 20ms
const unsigned long MAX_DISPLAY_ACTIVE = 600000;       // 10min
const unsigned long MAX_DISCONNECT_TIME = 600000;      // 10min
const unsigned long MAX_MQTT_DISCONNECT_TIME = 300000; // 5min
const uint8_t MAX_WIRES = 3;                           // max 3 sensor pairs
const unsigned int WARN_ANGLE = 110 + 320 * ((float)WARN_HUMIDITY / 100.0f);
const unsigned int ERROR_ANGLE = 110 + 320 * ((float)MAX_HUMIDITY / 100.0f);
// ca file
static const char *ca1 PROGMEM = "-----BEGIN CERTIFICATE-----\nMIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\nTzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\ncmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\nWhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\nZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\nMTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\nh77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\nA5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\nT8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\nB5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\nB5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\nKBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\nOlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\njh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\nqHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\nrU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\nHRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\nhkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\nubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\nNFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\nORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\nTkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\njNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\noyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\nmRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\nemyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n-----END CERTIFICATE-----\n";
static const char *ca2 PROGMEM = "-----BEGIN CERTIFICATE-----\nMIIDSjCCAjKgAwIBAgIQRK+wgNajJ7qJMDmGLvhAazANBgkqhkiG9w0BAQUFADA/\nMSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\nDkRTVCBSb290IENBIFgzMB4XDTAwMDkzMDIxMTIxOVoXDTIxMDkzMDE0MDExNVow\nPzEkMCIGA1UEChMbRGlnaXRhbCBTaWduYXR1cmUgVHJ1c3QgQ28uMRcwFQYDVQQD\nEw5EU1QgUm9vdCBDQSBYMzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEB\nAN+v6ZdQCINXtMxiZfaQguzH0yxrMMpb7NnDfcdAwRgUi+DoM3ZJKuM/IUmTrE4O\nrz5Iy2Xu/NMhD2XSKtkyj4zl93ewEnu1lcCJo6m67XMuegwGMoOifooUMM0RoOEq\nOLl5CjH9UL2AZd+3UWODyOKIYepLYYHsUmu5ouJLGiifSKOeDNoJjj4XLh7dIN9b\nxiqKqy69cK3FCxolkHRyxXtqqzTWMIn/5WgTe1QLyNau7Fqckh49ZLOMxt+/yUFw\n7BZy1SbsOFU5Q9D8/RhcQPGX69Wam40dutolucbY38EVAjqr2m7xPi71XAicPNaD\naeQQmxkqtilX4+U9m5/wAl0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNV\nHQ8BAf8EBAMCAQYwHQYDVR0OBBYEFMSnsaR7LHH62+FLkHX/xBVghYkQMA0GCSqG\nSIb3DQEBBQUAA4IBAQCjGiybFwBcqR7uKGY3Or+Dxz9LwwmglSBd49lZRNI+DT69\nikugdB/OEIKcdBodfpga3csTS7MgROSR6cz8faXbauX+5v3gTt23ADq1cEmv8uXr\nAvHRAosZy5Q6XkjEGB5YGV8eAlrwDPGxrancWYaLbumR9YbK+rlmM6pZW87ipxZz\nR8srzJmwN0jP41ZL9c8PDHIyh8bwRLtTcm1D9SZImlJnt1ir/md2cXjbDaJWFBM5\nJDGFoqgCWjBH4d1QB7wCCZAA62RjYJsWvIjJEubSfZGL+T0yjWW06XyxV3bqxbYo\nOb8VZRzI9neWagqNdwvYkQsEjgfbKbYK7p2CNTUQ\n-----END CERTIFICATE-----\n";
BearSSL::X509List x509;

// display
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
U8G2_FOR_ADAFRUIT_GFX u8g2;
// serial port for PJON
SoftwareSerial port;
PJON<ThroughSerialAsync> bus(ID);
// secure wifi connection client
WiFiClientSecure client;
// client
PubSubClient mqtt(SERVER, SERVER_PORT, client);

// PJON data structure
struct sensor
{
  uint16_t humidity;
  uint16_t temperature;
  bool connected_0;
  bool connected_1;
};
struct sensor_record
{
  sensor sensors[MAX_WIRES];
};
// status of system
enum status
{
  ERROR,
  WARN,
  NORMAL,
  OUTDATED,
  NONE,
  INIT
};
// cache data structure
struct cache_entry
{
  bool fresh;
  bool value_available;
  uint16_t humidity;
  double temperature;
};
struct cache
{
  cache_entry entries[MAX_WIRES];
};

// timers
unsigned long last_time_button_pressed = millis();
unsigned long last_time_payload_recieved = millis();
unsigned long last_time_display_active = millis();
unsigned long last_time_data_submitted = millis();
unsigned long last_time_tried_to_reconnect = 0;
// falgs
volatile bool data_cleared = false;
volatile bool buttonPressed = false;
volatile bool buzzer_active = false;
volatile bool error_active = false;
volatile bool display_active = true;
// status
status last_status = INIT;
// record
sensor_record record = {.sensors = {{.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}, {.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}, {.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}}};
// cache
cache cache_instance = {.entries = {{.fresh = true}, {.fresh = true}, {.fresh = true}}};

/**
 * Toggles display on or off.
 * 
 * @param on_off display state, true == on
 */
void toggleDisplay(bool on_off)
{
  last_time_display_active = millis();
  if (display_active == on_off)
  {
    return;
  }
  display_active = on_off;
  tft.enableDisplay(on_off);
  digitalWrite(TFT_LED, on_off ? HIGH : LOW);
}

/**
 * Draws an arc on given coordinate.
 */
void drawArc(uint16_t x, uint16_t y, int start_angle, int end_angle, uint16_t radius, uint16_t thinkness, uint16_t color)
{
  for (int i = start_angle; i <= end_angle; i++)
  {
    double rad_inner = i * DEG_TO_RAD;
    double rad_outer0 = (i == start_angle ? i : i - 1) * DEG_TO_RAD;
    double rad_outer1 = (i == end_angle ? i : i + 1) * DEG_TO_RAD;
    uint16_t outer_radius = radius + thinkness;
    double x0 = (cos(rad_inner) * radius) + x;
    double y0 = (sin(rad_inner) * radius) + y;
    double x1 = (cos(rad_outer0) * outer_radius) + x;
    double y1 = (sin(rad_outer0) * outer_radius) + y;
    double x2 = (cos(rad_outer1) * outer_radius) + x;
    double y2 = (sin(rad_outer1) * outer_radius) + y;
    tft.drawTriangle(x0, y0, x1, y1, x2, y2, color);
  }
}

/**
 * Draws a gauge at specified coordinate.
 */
void drawGauge(uint16_t x, uint16_t y, sensor *sensor, cache_entry *cache)
{
  // normal 0-50, warn 51-70, error 71-100
  uint16_t h = round((double)sensor->humidity / 100.0);
  h = h < 100 ? h : 100;
  double t = (double)(sensor->temperature / 10) / 10.0;
  t = t < 80.0 ? t : 80.0;
  bool value_available = sensor->connected_0 || sensor->connected_0;
  bool force_update = !cache->value_available && cache->value_available != value_available;
  bool force_reset = !value_available && value_available != cache->value_available;

  // 360-40 = 320
  // draw humidity
  if (cache->fresh || force_update || (h != cache->humidity && value_available))
  {
    uint16_t degrees = round(h * 3.2) + 110;
    if (h > 0 && h <= WARN_HUMIDITY)
    {
      drawArc(x, y, 110, degrees, 26, 4, GREEN);
    }
    else if (h > WARN_HUMIDITY && h <= MAX_HUMIDITY)
    {
      drawArc(x, y, 110, WARN_ANGLE, 26, 4, GREEN);
      drawArc(x, y, WARN_ANGLE + 1, degrees, 26, 4, YELLOW);
    }
    else if (h > MAX_HUMIDITY)
    {
      drawArc(x, y, 110, WARN_ANGLE, 26, 4, GREEN);
      drawArc(x, y, WARN_ANGLE + 1, ERROR_ANGLE, 26, 4, YELLOW);
      drawArc(x, y, ERROR_ANGLE + 1, degrees, 26, 4, RED);
    }
    drawArc(x, y, degrees + (degrees > 0 && degrees < 430 ? 1 : 0), 430, 26, 4, ST7735_BLACK);
    drawArc(x, y, ERROR_ANGLE, ERROR_ANGLE, 25, 7, ST7735_WHITE);
  }
  // reset gauge
  if (force_reset)
  {
    drawArc(x, y, 110, 430, 26, 4, ST7735_BLACK);
    drawArc(x, y, ERROR_ANGLE, ERROR_ANGLE, 25, 7, ST7735_WHITE);
  }

  // draw temperature
  if (cache->fresh || force_update || (t != cache->temperature && value_available))
  {
    uint16_t degrees = round(t * 4) + 110;
    drawArc(x, y, 110, degrees, 22, 2, BLUE);
    drawArc(x, y, degrees + (degrees > 0 && degrees < 430 ? 1 : 0), 430, 22, 2, ST7735_BLACK);
  }
  // reset gauge
  if (force_reset)
  {
    drawArc(x, y, 110, 430, 22, 2, ST7735_BLACK);
  }

  // write numbers
  if (cache->fresh || force_update || force_reset || h != cache->humidity || t != cache->temperature)
  {
    tft.fillCircle(x, y, 21, ST7735_BLACK);
    u8g2.setForegroundColor(ST7735_WHITE);
    char h_string[4] = "--%";
    if (h > 0 && value_available)
    {
      sprintf(h_string, "%3d%%", h);
    }
    u8g2.setFont(dejavu_14);
    u8g2.drawUTF8(x - u8g2.getUTF8Width(h_string) / 2, y, h_string);

    u8g2.setForegroundColor(ST7735_WHITE);
    char t_string[8] = "--°C";
    if (t > 0 && value_available)
    {
      sprintf(t_string, "%3.1f°C", t);
    }
    u8g2.setFont(dejavu_8);
    u8g2.drawUTF8(x - u8g2.getUTF8Width(t_string) / 2, y + 12, t_string);
  }

  // draw max value
  if (cache->fresh)
  {
    drawArc(x, y, 110, 333, 25, 0, ST7735_WHITE);
  }

  // set values in cache
  cache->value_available = value_available;
  cache->fresh = false;
  cache->humidity = h;
  cache->temperature = t;
}

/**
 * Draws a status as icon.
 */
void drawStatus(uint16_t x, uint16_t y, status s, status *last_status)
{
  status last = *last_status;
  if (last != s)
  {
    switch (s)
    {
    case ERROR:
      tft.fillCircle(x, y, 30, RED);
      tft.fillCircle(x, y, 26, ST7735_BLACK);
      tft.drawCircle(x, y, 26, ST7735_WHITE);
      u8g2.setFont(open_iconic_symbols_4x);
      u8g2.setForegroundColor(BLUE);
      u8g2.drawGlyph(x - 16, y + 16, 67);
      break;
    case WARN:
      tft.fillCircle(x, y, 30, YELLOW);
      tft.fillCircle(x, y, 26, ST7735_BLACK);
      tft.drawCircle(x, y, 26, ST7735_WHITE);
      u8g2.setFont(open_iconic_symbols_4x);
      u8g2.setForegroundColor(YELLOW);
      u8g2.drawGlyph(x - 14, y + 14, 66);
      break;
    case NORMAL:
      tft.fillCircle(x, y, 30, GREEN);
      tft.fillCircle(x, y, 26, ST7735_BLACK);
      tft.drawCircle(x, y, 26, ST7735_WHITE);
      u8g2.setFont(open_iconic_symbols_4x);
      u8g2.setForegroundColor(GREEN);
      u8g2.drawGlyph(x - 16, y + 16, 65);
      break;
    case OUTDATED:
      tft.fillCircle(x, y, 30, ST7735_BLACK);
      tft.drawCircle(x, y, 26, ST7735_WHITE);
      u8g2.setFont(open_iconic_symbols_4x);
      u8g2.setForegroundColor(ST7735_WHITE);
      u8g2.drawGlyph(x - 16, y + 16, 64);
      break;
    default:
      tft.fillCircle(x, y, 30, ST7735_BLACK);
      tft.drawCircle(x, y, 26, ST7735_WHITE);
      break;
    }
  }
  *last_status = s;
}

/**
 * Checks record for new status by checking values against configured threshold.
 */
void checkStatusAndDrawValues(sensor_record *record)
{
  last_time_payload_recieved = millis();
  if (data_cleared)
  {
    toggleDisplay(true);
  }
  data_cleared = false;
  uint16_t max_humidity = 0;
  for (uint8_t i = 0; i < MAX_WIRES; i++)
  {
    if (record->sensors[i].humidity > max_humidity)
    {
      max_humidity = record->sensors[i].humidity;
    }
  }
  if (max_humidity > MAX_HUMIDITY * 100 && !error_active)
  {
#ifdef DEBUG
    Serial.println("Humidity to high: Turn on the buzzer!");
#endif
    toggleDisplay(true);
    error_active = true;
    buzzer_active = true;
    tone(BUZZER_PIN, 1480.0);
    drawStatus(96, 32, ERROR, &last_status);
  }
  else if (max_humidity > WARN_HUMIDITY * 100 && max_humidity <= MAX_HUMIDITY * 100)
  {
    noTone(BUZZER_PIN);
    drawStatus(96, 32, WARN, &last_status);
  }
  else if (max_humidity > 0 && max_humidity <= WARN_HUMIDITY * 100)
  {
    noTone(BUZZER_PIN);
    drawStatus(96, 32, NORMAL, &last_status);
  }
  if (error_active && max_humidity <= MAX_HUMIDITY * 100)
  {
    error_active = false;
  }
  drawGauge(32, 32, &record->sensors[0], &cache_instance.entries[0]);
  drawGauge(32, 96, &record->sensors[1], &cache_instance.entries[1]);
  drawGauge(96, 96, &record->sensors[2], &cache_instance.entries[2]);
}

/**
 * Connects MQTT
 */
bool connectMQTT()
{
  // Stop if already connected.
  if (mqtt.connected())
  {
    return true;
  }

#ifdef DEBUG
  Serial.println("Connecting to MQTT... ");
#endif

  bool result = mqtt.connect("humidity_device", TOKEN, "");
  if (!result)
  {
#ifdef DEBUG
    Serial.println(mqtt.state());
    Serial.println("MQTT not connected!");
#endif
    mqtt.disconnect();
    // set connection status
    tft.fillRect(0, 0, 4, 4, YELLOW);
    return false;
  }

#ifdef DEBUG
  Serial.println("MQTT connected!");
#endif

  // set connection status
  tft.fillRect(0, 0, 4, 4, GREEN);
  return true;
}

/**
 * Submits values to mqtt broker.
 */
void submitValues(sensor_record *record)
{
  long current_millis = millis();
  if ((current_millis - last_time_data_submitted) > MAX_AGE_CLOUD && mqtt.connected() && mqtt.state() == MQTT_CONNECTED)
  {
    last_time_data_submitted = current_millis;
#ifdef DEBUG
    Serial.println("Submit data!");
#endif

    // build topic and payload
    char payload[50];
    char *payload_ptr = payload;
    bool has_payload = false;
    for (uint8_t i = 0; i < MAX_WIRES; i++)
    {
      if (record->sensors[i].connected_0 || record->sensors[i].connected_1)
      {
        payload_ptr += sprintf(payload_ptr, "%s\"h%d\":%3.2f,\"t%d\":%3.2f", has_payload ? "," : "", i, record->sensors[i].humidity / 100.0, i, record->sensors[i].temperature / 100.0);
        has_payload = true;
      }
    }
    if (has_payload)
    {
      char wrapped[50];
      sprintf(wrapped, "{%s}", payload);
#ifdef DEBUG
      Serial.println(wrapped);
#endif
      // publish data
      mqtt.publish(TOPIC, wrapped);
    }
#ifdef DEBUG
    Serial.printf("Free space: [%d]\n", ESP.getFreeHeap());
#endif
  }
#ifdef DEBUG
  if (mqtt.state() != MQTT_CONNECTED)
  {
    Serial.println("MQTT Connection lost!");
  }
#endif
}

/**
 * Payload handler for PJON.
 */
void payloadHandler(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info)
{
  memcpy(&record, payload, sizeof(record));
  // handler data
  checkStatusAndDrawValues(&record);
  // send data to the cloud
  submitValues(&record);
#ifdef DEBUG
  for (uint8_t i = 0; i < MAX_WIRES; i++)
  {
    Serial.printf("Recieved: hum %u; temp %u; s0 %u; s1 %u\n", record.sensors[i].humidity, record.sensors[i].temperature, record.sensors[i].connected_0, record.sensors[i].connected_1);
  }
#endif
}

/**
 * Error handler for PJON.
 */
void errorHandler(uint8_t code, uint16_t data, void *custom_pointer)
{
  if (code == PJON_CONNECTION_LOST)
  {
    digitalWrite(LED_BUILTIN, HIGH);
#ifdef DEBUG
    Serial.println("Error!");
#endif
  }
}

/**
 * Button interupt handler. 
 * 
 * Turns on display, if display is off and now error is present.
 * If error is present, button pressed deactivates buzzer.
 */
void ICACHE_RAM_ATTR buttonPress()
{
  if ((millis() - last_time_button_pressed) > DEBOUNCE)
  {
    if (buttonPressed == false)
    {
      buttonPressed = true;
      if (!buzzer_active)
      {
        toggleDisplay(!display_active);
      }
      buzzer_active = false;
      noTone(BUZZER_PIN);
    }
    else
    {
      buttonPressed = false;
    }
    last_time_button_pressed = millis();
  }
}

/**
 * Init wifi.
 */
bool initWifi()
{
  WiFi.mode(WIFI_STA);
#ifdef DEBUG
  Serial.printf("Stored SSID: [%s]\n", WiFi.SSID().c_str());
#endif
  if (strlen(WiFi.SSID().c_str()) > 0)
  {
    WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str());
  }
  else
  {
    WiFi.begin(DEFAULT_WLAN_SSID, DEFAULT_WLAN_KEY);
  }
  uint8_t count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 100)
  {
    delay(500);
    count++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    return true;
  }
  else
  {
    delay(10000);
    // start wps
    if (WiFi.beginWPSConfig() && WiFi.SSID().length() > 0)
    {
      return true;
    }
    else
    {
      return false;
#ifdef DEBUG
      Serial.println("WPS failed!");
#endif
    }
  }
}

/**
 * Setup system.
 */
void setup()
{
  delay(2000);
#ifdef DEBUG
  Serial.begin(9600);
  Serial.setDebugOutput(true);
#endif

  // setup TFT backlight pin
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);

  // init TFT
  tft.initR(INITR_144GREENTAB);
  // use u8g2
  u8g2.begin(tft);
  u8g2.setForegroundColor(ST7735_WHITE);
  tft.setRotation(tft.getRotation() + 1);
  tft.fillScreen(ST7735_BLACK);

  // draw status and gauges
  drawStatus(96, 32, NONE, &last_status);
  drawGauge(32, 32, &record.sensors[0], &cache_instance.entries[0]);
  drawGauge(32, 96, &record.sensors[0], &cache_instance.entries[1]);
  drawGauge(96, 96, &record.sensors[0], &cache_instance.entries[2]);

  // start software serial port
  port.begin(9600, SWSERIAL_8N1, RX_PIN, TX_PIN, false);

  // setup PJON
  bus.strategy.set_serial(&port);
  // set handlers
  bus.set_error(errorHandler);
  bus.set_receiver(payloadHandler);
  // start bus
  bus.begin();

  // setup button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // connect to interrupt
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPress, CHANGE);

  // init wifi
  bool connection_status = initWifi();
  // set connection status
  tft.fillRect(0, 0, 4, 4, connection_status ? GREEN : RED);
  if (connection_status)
  {
    // init mqtt
    configTime(1 * 3600, 0, "de.pool.ntp.org");
    delay(2000);
    x509.append(ca1);
    x509.append(ca2);
    client.setTrustAnchors(&x509);
    connectMQTT();
  }

#ifdef DEBUG
  Serial.println("Init complete!");
#endif
}

/**
 * Event loop.
 */
void loop()
{
  // trigger PJON handlers
  bus.update();
  bus.receive();
  // trigger mqtt handlers
  mqtt.loop();

  unsigned long current_millis = millis();
  // turn on display and show cleared status if no new data has been received
  if (!data_cleared && (current_millis - last_time_payload_recieved) > MAX_AGE)
  {
    toggleDisplay(true);
    drawStatus(96, 32, OUTDATED, &last_status);
    for (uint8_t i = 0; i < MAX_WIRES; i++)
    {
      record.sensors[i].connected_0 = false;
      record.sensors[i].connected_1 = false;
    }
    drawGauge(32, 32, &record.sensors[0], &cache_instance.entries[0]);
    drawGauge(32, 96, &record.sensors[1], &cache_instance.entries[1]);
    drawGauge(96, 96, &record.sensors[2], &cache_instance.entries[2]);
    data_cleared = true;
  }

  // turn off backlight if no error is present, backlight is on and display timeout is reached
  if (!error_active && display_active && (current_millis - last_time_display_active) > MAX_DISPLAY_ACTIVE)
  {
    toggleDisplay(false);
  }

  // check connection status
  if (!mqtt.connected())
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      // connect to mqtt broker
      if ((current_millis - last_time_tried_to_reconnect) > MAX_MQTT_DISCONNECT_TIME)
      {
        last_time_tried_to_reconnect = current_millis;
        if (connectMQTT())
        {
          last_time_tried_to_reconnect = 0;
        }
      }
      else
      {
        // set connection status
        tft.fillRect(0, 0, 4, 4, YELLOW);
      }
    }
    else
    {
      // set connection status
      tft.fillRect(0, 0, 4, 4, RED);
    }
  }

  // restart machine, if we are not connected
  if (WiFi.status() != WL_CONNECTED && (current_millis - last_time_data_submitted) > MAX_DISCONNECT_TIME)
  {
    WiFi.forceSleepBegin();
    wdt_reset();
    ESP.restart();
    while (1)
    {
      wdt_reset();
    }
  }
}