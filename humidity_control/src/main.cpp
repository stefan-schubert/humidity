#define PJON_PACKET_MAX_LENGTH 100

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "SoftwareSerial.h"
#include "Wire.h"
#include "PJON.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ST7735.h"
#include "U8g2_for_Adafruit_GFX.h"
#include <Ubidots.h>
#include <config.h>

// DEBUG flag
#define DEBUG

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

// constants
const unsigned int MAX_HUMIDITY = 70;            // 70.00%
const unsigned int WARN_HUMIDITY = 50;           // 50.00 %
const unsigned long MAX_AGE = 10000;             // 10s
const unsigned long DEBOUNCE = 20;               // 20ms
const unsigned long MAX_DISPLAY_ACTIVE = 600000; // 10min
const uint8_t MAX_WIRES = 3;                     // max 3 sensor pairs
const unsigned int WARN_ANGLE = 110 + 320 * ((float)WARN_HUMIDITY / 100.0f);
const unsigned int ERROR_ANGLE = 110 + 320 * ((float)MAX_HUMIDITY / 100.0f);

// display
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
U8G2_FOR_ADAFRUIT_GFX u8g2;
// serial port for PJON
SoftwareSerial port;
PJON<ThroughSerialAsync> bus(ID);
Ubidots ubidots((char *)TOKEN, UBI_INDUSTRIAL, UBI_UDP);

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
    u8g2.setFont(u8g2_font_helvR14_tr);
    u8g2.drawUTF8(x - u8g2.getUTF8Width(h_string) / 2, y, h_string);

    u8g2.setForegroundColor(ST7735_WHITE);
    char t_string[8] = "--°C";
    if (t > 0 && value_available)
    {
      sprintf(t_string, "%3.1f°C", t);
    }
    u8g2.setFont(u8g2_font_helvR08_tr);
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
      u8g2.setFont(u8g2_font_open_iconic_thing_4x_t);
      u8g2.setForegroundColor(BLUE);
      u8g2.drawGlyph(x - 16, y + 16, 72);
      break;
    case WARN:
      tft.fillCircle(x, y, 30, YELLOW);
      tft.fillCircle(x, y, 26, ST7735_BLACK);
      tft.drawCircle(x, y, 26, ST7735_WHITE);
      u8g2.setFont(u8g2_font_open_iconic_embedded_4x_t);
      u8g2.setForegroundColor(YELLOW);
      u8g2.drawGlyph(x - 16, y + 16, 65);
      break;
    case NORMAL:
      tft.fillCircle(x, y, 30, GREEN);
      tft.fillCircle(x, y, 26, ST7735_BLACK);
      tft.drawCircle(x, y, 26, ST7735_WHITE);
      u8g2.setFont(u8g2_font_open_iconic_www_4x_t);
      u8g2.setForegroundColor(GREEN);
      u8g2.drawGlyph(x - 16, y + 16, 73);
      break;
    case OUTDATED:
      tft.fillCircle(x, y, 30, ST7735_BLACK);
      tft.drawCircle(x, y, 26, ST7735_WHITE);
      u8g2.setFont(u8g2_font_open_iconic_www_4x_t);
      u8g2.setForegroundColor(ST7735_WHITE);
      u8g2.drawGlyph(x - 16, y + 16, 74);
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
 * Publishes values to ubidots.
 */
void submitValues(sensor_record *record)
{
  ubidots.add("humidity_0", record->sensors[0].humidity / 100.0);
  ubidots.add("temperature_0", record->sensors[0].temperature / 100.0);
  ubidots.add("humidity_1", record->sensors[1].humidity / 100.0);
  ubidots.add("temperature_1", record->sensors[1].temperature / 100.0);
  ubidots.add("humidity_2", record->sensors[2].humidity / 100.0);
  ubidots.add("temperature_2", record->sensors[2].temperature / 100.0);
  ubidots.send((char *)LABEL);
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
  port.begin(9600, SWSERIAL_8N1, RX_PIN, TX_PIN, false, 256);

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
  while (WiFi.status() == WL_DISCONNECTED && count < 10)
  {
    delay(1000);
    tft.fillRect(count * 4, 0, 2, 2, ST7735_WHITE);
    count++;
  }
  // reset count
  tft.fillRect(0, 0, 18, 2, ST7735_BLACK);

  if (WiFi.status() == WL_CONNECTED)
  {
    tft.fillRect(0, 0, 4, 4, GREEN);
  }
  else
  {
    delay(10000);
    // start wps
    if (WiFi.beginWPSConfig() && WiFi.SSID().length() > 0)
    {
      tft.fillRect(0, 0, 4, 4, GREEN);
    }
    else
    {
      tft.fillRect(0, 0, 4, 4, RED);
#ifdef DEBUG
      Serial.println("WPS failed!");
#endif
    }
  }

  // ubidots init
#ifdef DEBUG
  ubidots.setDebug(true);
#endif

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
}