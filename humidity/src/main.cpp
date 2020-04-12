#define PJON_PACKET_MAX_LENGTH 100

#include <Arduino.h>
#include "brzo_i2c.h"
#include <BME280_BRZO.h>
#include "SoftwareSerial.h"
#include "PJON.h"

// DEBUG flag
#define NO_DEBUG

// pin config
#define SDA D2
#define SCL D1
#define RX_PIN D3
#define TX_PIN D6

// multiplexer address
#define TCAADDR 0x70
// PJON id
#define ID 44

// constants
// check interval to read sensor values
const unsigned long CHECK_INTERVAL = 5000;
// max sensor wires connected to multiplexer
// each wire has 2 sensors
const uint8_t MAX_WIRES = 3;
// last measured time
unsigned long lastTime = millis();
// serial port for PJON
SoftwareSerial port;
// PJON instance
PJON<ThroughSerialAsync> bus(ID);

// BME280 sensor instances with settings
BME280I2C_BRZO::Settings settings_bme_0;
BME280I2C_BRZO bme_0;
BME280I2C_BRZO::Settings settings_bme_1;
BME280I2C_BRZO bme_1;

// sensor struct
struct sensor
{
  uint16_t humidity;
  uint16_t temperature;
  bool connected_0;
  bool connected_1;
};
// complete record
struct sensor_record
{
  sensor sensors[MAX_WIRES];
};
// prefill record instance
sensor_record record = {.sensors = {{.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}, {.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}, {.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}}};

/**
 * Selects a sensor wire.
 */
void tca_select(uint8_t i)
{
  if (i > 7)
    return;

  uint8_t buffer[1];
  brzo_i2c_start_transaction(TCAADDR, 400);
  buffer[0] = 1 << i;
  brzo_i2c_write(buffer, 1, false);
  uint8_t result = brzo_i2c_end_transaction();

#ifdef DEBUG
  if (result != 0)
  {
    Serial.printf("tca select result: %d\n", result);
  }
#endif
}

/**
 * Detects the first or second sensors on given wire index.
 * 
 * If sensor was already detected, the sensor is only validated and not initialized.
 */
void detect_sensor(uint8_t index, bool first)
{
  if (first)
  {
    record.sensors[index].connected_0 = record.sensors[index].connected_0 ? bme_0.sensorPresent() : bme_0.begin();
#ifdef DEBUG
    if (!record.sensors[index].connected_0)
    {
      Serial.printf("Could not find BME280 sensor (0) for connector #%d\n", index);
    }
#endif
  }
  else
  {
    record.sensors[index].connected_1 = record.sensors[index].connected_1 ? bme_1.sensorPresent() : bme_1.begin();
#ifdef DEBUG
    if (!record.sensors[index].connected_1)
    {
      Serial.printf("Could not find BME280 sensor (1) for connector #%d\n", index);
    }
#endif
  }
}

/**
 * Reads a complete sensor record and sends them to peer.
 */
void read_and_send_record()
{
  for (uint8_t i = 0; i < MAX_WIRES; i++)
  {
    // select wire
    tca_select(i);

    uint8_t count = 0;
    record.sensors[i].humidity = 0;
    record.sensors[i].temperature = 0;

    // detect sensor before reading
    detect_sensor(i, true);
    if (record.sensors[i].connected_0)
    {
      record.sensors[i].humidity += bme_0.hum() * 100;
      record.sensors[i].temperature += bme_0.temp() * 100;
      count++;
    }
    // detect sensor before reading
    detect_sensor(i, false);
    if (record.sensors[i].connected_1)
    {
      record.sensors[i].humidity += bme_1.hum() * 100;
      record.sensors[i].temperature += bme_1.temp() * 100;
      count++;
    }
    // calculate average value for transmission, if 2 sensors are readed for ech wire
    if (count > 0)
    {
      record.sensors[i].humidity = record.sensors[i].humidity / count;
      record.sensors[i].temperature = record.sensors[i].temperature / count;
    }
  }

#ifdef DEBUG
  for (uint8_t i = 0; i < MAX_WIRES; i++)
  {
    Serial.printf("Payload: hum %u; temp %u; s0 %u; s1 %u\n", record.sensors[i].humidity, record.sensors[i].temperature, record.sensors[i].connected_0, record.sensors[i].connected_1);
  }
#endif

  // send dara
  bus.send(45, &record, sizeof(record));
}

// error handler for PJON
void error_handler(uint8_t code, uint16_t data, void *custom_pointer)
{
#ifdef DEBUG
  Serial.printf("PJON error: %d\n", code);
#endif
}

/**
 * Setup system.
 */
void setup()
{
  delay(2000);
#ifdef DEBUG
  Serial.begin(9600, SERIAL_8N1);
  Serial.println("\nInit.");
#endif

  // setup I2C bus with 2ms timeout
  brzo_i2c_setup(SDA, SCL, 2000);

  // setup BME280 instances
  // set correct address for different instances
  settings_bme_0.bme280Addr = BME280I2C::I2CAddr_0x76;
  bme_0.setSettings(settings_bme_0);
  settings_bme_1.bme280Addr = BME280I2C::I2CAddr_0x77;
  bme_1.setSettings(settings_bme_1);

  // start software serial port
  port.begin(9600, SWSERIAL_8N1, RX_PIN, TX_PIN, false, 256);

  // setup PJON with serial port
  bus.strategy.set_serial(&port);
  // set error handler
  bus.set_error(error_handler);
  // start bus
  bus.begin();

  // detect sensor initially
  for (uint8_t i = 0; i < MAX_WIRES; i++)
  {
    tca_select(i);
    detect_sensor(i, true);
    detect_sensor(i, false);
  }

#ifdef DEBUG
  Serial.println("\nInit done.");
#endif
}

/**
 * Event loop.
 */
void loop()
{
  if ((millis() - lastTime) > CHECK_INTERVAL)
  {
    // read sensors after interval is over
    read_and_send_record();
    // store last submission
    lastTime = millis();
  }
  // trigger PJON handlers
  bus.update();
  bus.receive();
}