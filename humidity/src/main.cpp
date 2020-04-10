#define PJON_PACKET_MAX_LENGTH 100

#include <Arduino.h>
#include "Wire.h"
#include "twi.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "SoftwareSerial.h"
#include "PJON.h"

#define DEBUG

#define TCAADDR 0x70
#define SENSOR_0 0x76
#define SENSOR_1 0x77

#define RX_PIN D3
#define TX_PIN D6
#define ID 44

const unsigned long CHECK_INTERVAL = 5000;
unsigned long lastTime = millis();
SoftwareSerial port;
PJON<ThroughSerialAsync> bus(ID);

const uint8_t MAX_WIRES = 3;
Adafruit_BME280 bme_0;
Adafruit_BME280 bme_1;
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
sensor_record record = {.sensors = {{.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}, {.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}, {.humidity = 0, .temperature = 0, .connected_0 = false, .connected_1 = false}}};;

void tca_select(uint8_t i)
{
  // return; // TODO remove
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  uint8_t result = Wire.endTransmission();
  if (result != 0)
  {
#ifdef DEBUG
    Serial.printf("tca select result: %d\n", result);
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

    if (record.sensors[i].connected_0)
    {
      record.sensors[i].humidity += bme_0.readHumidity() * 100;
      record.sensors[i].temperature += bme_0.readTemperature() * 100;
      count++;
    }
    if (record.sensors[i].connected_1)
    {
      record.sensors[i].humidity += bme_1.readHumidity() * 100;
      record.sensors[i].temperature += bme_1.readTemperature() * 100;
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

// /**
//  * This routine turns off the I2C bus and clears it
//  * on return SCA and SCL pins are tri-state inputs.
//  * You need to call Wire.begin() after this to re-enable I2C
//  * This routine does NOT use the Wire library at all.
//  *
//  * returns 0 if bus cleared
//  *         1 if SCL held low.
//  *         2 if SDA held low by slave clock stretch for > 2sec
//  *         3 if SDA held low after 20 clocks.
//  */
// int I2C_ClearBus()
// {
//   pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
//   pinMode(SCL, INPUT_PULLUP);

//   delay(2500); // Wait 2.5 secs. This is strictly only necessary on the first power
//   // up of the DS3231 module to allow it to initialize properly,
//   // but is also assists in reliable programming of FioV3 boards as it gives the
//   // IDE a chance to start uploaded the program
//   // before existing sketch confuses the IDE by sending Serial data.

//   boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
//   if (SCL_LOW)
//   {           //If it is held low Arduno cannot become the I2C master.
//     return 1; //I2C bus error. Could not clear SCL clock line held low
//   }

//   boolean SDA_LOW = (digitalRead(SDA) == LOW); // vi. Check SDA input.
//   int clockCount = 20;                         // > 2x9 clock

//   while (SDA_LOW && (clockCount > 0))
//   { //  vii. If SDA is Low,
//     clockCount--;
//     // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
//     pinMode(SCL, INPUT);        // release SCL pullup so that when made output it will be LOW
//     pinMode(SCL, OUTPUT);       // then clock SCL Low
//     delayMicroseconds(10);      //  for >5uS
//     pinMode(SCL, INPUT);        // release SCL LOW
//     pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
//     // do not force high as slave may be holding it low for clock stretching.
//     delayMicroseconds(10); //  for >5uS
//     // The >5uS is so that even the slowest I2C devices are handled.
//     SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
//     int counter = 20;
//     while (SCL_LOW && (counter > 0))
//     { //  loop waiting for SCL to become High only wait 2sec.
//       counter--;
//       delay(100);
//       SCL_LOW = (digitalRead(SCL) == LOW);
//     }
//     if (SCL_LOW)
//     {           // still low after 2 sec error
//       return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
//     }
//     SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
//   }
//   if (SDA_LOW)
//   {           // still low
//     return 3; // I2C bus error. Could not clear. SDA data line held low
//   }

//   // else pull SDA line low for Start or Repeated Start
//   pinMode(SDA, INPUT);  // remove pullup.
//   pinMode(SDA, OUTPUT); // and then make it LOW i.e. send an I2C Start or Repeated start control.
//   // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
//   /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
//   delayMicroseconds(10);      // wait >5uS
//   pinMode(SDA, INPUT);        // remove output low
//   pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
//   delayMicroseconds(10);      // x. wait >5uS
//   pinMode(SDA, INPUT);        // and reset pins as tri-state inputs which is the default state on reset
//   pinMode(SCL, INPUT);
//   return 0; // all ok
// }

void setup()
{
  delay(5000);
#ifdef DEBUG
  Serial.begin(9600, SERIAL_8N1);
  Serial.println("\nInit.");
#endif

  // Wire.setClock(10000);
  // Wire.setClockStretchLimit(300000L);

  // if (digitalRead(D1) == HIGH && digitalRead(D2) == LOW)
  // {
  //   Serial.println("reset");
  //   pinMode(D1, OUTPUT); // is connected to SCL
  //   digitalWrite(D1, LOW);
  //   delay(2000);        //maybe too long
  //   pinMode(D1, INPUT); // reset pin
  //   delay(50);
  //   ESP.reset();
  // }
  //Serial.println(I2C_ClearBus());

  Wire.begin(D2, D1);

#ifdef DEBUG
  Serial.printf("Wire status: %d\n", Wire.status());
#endif

  port.begin(9600, SWSERIAL_8N1, RX_PIN, TX_PIN, false, 256);

  bus.strategy.set_serial(&port);
  bus.set_error(error_handler);
  bus.begin();

  for (uint8_t i = 0; i < MAX_WIRES; i++) // TODO MAX_WIRES
  {
    tca_select(i);
    record.sensors[i].connected_0 = true;
    if (!bme_0.begin(SENSOR_0, &Wire))
    {
#ifdef DEBUG
      Serial.printf("Could not find BME280 sensor (0) for connector #%d\n", i);
#endif
      record.sensors[i].connected_0 = false;
    }
    record.sensors[i].connected_1 = true;
    if (!bme_1.begin(SENSOR_1, &Wire))
    {
#ifdef DEBUG
      Serial.printf("Could not find BME280 sensor (1) for connector #%d\n", i);
#endif
      record.sensors[i].connected_1 = false;
    }
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