#define PJON_PACKET_MAX_LENGTH 100

#include <Arduino.h>
#include "brzo_i2c.h"
#include <BME280_BRZO.h>
#include "SoftwareSerial.h"
#include "PJON.h"

#define DEBUG

#define SDA D2
#define SCL D1
#define TCAADDR 0x70

#define RX_PIN D3
#define TX_PIN D6
#define ID 44

const unsigned long CHECK_INTERVAL = 5000;
unsigned long lastTime = millis();
SoftwareSerial port;
PJON<ThroughSerialAsync> bus(ID);

const uint8_t MAX_WIRES = 3;
BME280I2C_BRZO::Settings settings_bme_0;
BME280I2C_BRZO bme_0;
BME280I2C_BRZO::Settings settings_bme_1;
BME280I2C_BRZO bme_1;

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
sensor_record record = {.sensors = {{.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}, {.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}, {.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}}};

void tca_select(uint8_t i)
{
  // return; // TODO remove
  if (i > 7)
    return;

  uint8_t buffer[1];
  brzo_i2c_start_transaction(TCAADDR, 400);
  buffer[0] = 1 << i;
  brzo_i2c_write(buffer, 1, false);
  uint8_t result = brzo_i2c_end_transaction();

  if (result != 0)
  {
#ifdef DEBUG
    Serial.printf("tca select result: %d\n", result);
#endif
  }
}

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

void read_and_send_record()
{
  for (uint8_t i = 0; i < MAX_WIRES; i++) // TODO MAX_WIRES
  {
    tca_select(i);

    uint8_t count = 0;
    record.sensors[i].humidity = 0;
    record.sensors[i].temperature = 0;

    detect_sensor(i, true);
    if (record.sensors[i].connected_0)
    {
      record.sensors[i].humidity += bme_0.hum() * 100;
      record.sensors[i].temperature += bme_0.temp() * 100;
      count++;
    }
    detect_sensor(i, false);
    if (record.sensors[i].connected_1)
    {
      record.sensors[i].humidity += bme_1.hum() * 100;
      record.sensors[i].temperature += bme_1.temp() * 100;
      count++;
    }
    if (count > 0)
    {
      record.sensors[i].humidity = record.sensors[i].humidity / count;
      record.sensors[i].temperature = record.sensors[i].temperature / count;
    }
  }

#ifdef DEBUG
  for (uint8_t i = 0; i < MAX_WIRES; i++) // TODO MAX_WIRES
  {
    Serial.printf("Payload: hum %u; temp %u; s0 %u; s1 %u\n", record.sensors[i].humidity, record.sensors[i].temperature, record.sensors[i].connected_0, record.sensors[i].connected_1);
  }
#endif
  bus.send(45, &record, sizeof(record));
}

void error_handler(uint8_t code, uint16_t data, void *custom_pointer)
{
#ifdef DEBUG
  Serial.printf("PJON error: %d\n", code);
#endif
}

void setup()
{
  delay(5000);
#ifdef DEBUG
  Serial.begin(9600, SERIAL_8N1);
  Serial.println("\nInit.");
#endif

  brzo_i2c_setup(SDA, SCL, 2000);

  settings_bme_0.bme280Addr = BME280I2C::I2CAddr_0x76;
  bme_0.setSettings(settings_bme_0);
  settings_bme_1.bme280Addr = BME280I2C::I2CAddr_0x77;
  bme_1.setSettings(settings_bme_1);

  port.begin(9600, SWSERIAL_8N1, RX_PIN, TX_PIN, false, 256);

  bus.strategy.set_serial(&port);
  bus.set_error(error_handler);
  bus.begin();

  for (uint8_t i = 0; i < MAX_WIRES; i++) // TODO MAX_WIRES
  {
    tca_select(i);
    detect_sensor(i, true);
    detect_sensor(i, false);
  }

#ifdef DEBUG
  Serial.println("\nInit done.");
#endif
}

void loop()
{
  if ((millis() - lastTime) > CHECK_INTERVAL)
  {
    read_and_send_record();
    lastTime = millis();
  }
  bus.update();
  bus.receive();
}