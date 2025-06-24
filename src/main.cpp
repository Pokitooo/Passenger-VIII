#include <Arduino.h>
#include <lib_xcore>
#include <STM32FreeRTOS.h>

#include <Wire.h>
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_v3.h>
#include <TCA9548.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>

#include "SdFat.h"
#include "RadioLib.h"

#include "Arduino_Extended.h"

#define UBLOX_CUSTOM_MAX_WAIT (250u)
#define SPI_SPEED_SD_MHZ (20)

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

// Pins defination
#define PIN_SPI_MOSI1 PA7
#define PIN_SPI_MISO1 PA6
#define PIN_SPI_SCK1 PA5

// NSS
#define PIN_NSS_ICM PA15
#define PIN_NSS_SD PA4

// lORA PIN DEF
#define LORA_NSS PA1
#define LORA_BUSY PA0
#define LORA_NRST PA3

// i2c
TwoWire i2c(PB7, PB6);
SFE_UBLOX_GNSS m10s;
Adafruit_BME280 bme;
Adafruit_MS8607 ms8607[2];
TCA9548 tca(0x70, &i2c);

// SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);

// SD
SdSpiConfig sd_config(PIN_NSS_SD, SHARED_SPI, SD_SCK_MHZ(25), &spi1);
SdExFat sd = {};
ExFile file = {};

// LoRa
volatile bool tx_flag = false;
String s;

SPISettings lora_spi_settings(18'000'000, MSBFIRST, SPI_MODE0);

constexpr struct
{
  float center_freq = 921.000'000f; // MHz
  float bandwidth = 125.f;          // kHz
  uint8_t spreading_factor = 12;    // SF: 6 to 12
  uint8_t coding_rate = 8;          // CR: 5 to 8
  uint8_t sync_word = 0x12;         // Private SX1262
  int8_t power = 22;                // up to 22 dBm for SX1262
  uint16_t preamble_length = 16;
} params;

SX1262 lora = new Module(LORA_NSS, RADIOLIB_NC, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

struct Data
{
  // 40 bits
  uint32_t timestamp;
  uint8_t counter;

  // 160 bits
  double gps_latitude;
  double gps_longitude;
  float gps_altitude;

  // 96 bits
  float temp;
  float humid;
  float press;

  struct MS8607Data
  {
    float pres{};
    float alt{};
    float temp{};
    float humid{};
  } msData[2]{};
};

// Status
struct Status
{
  bool tca9548;
  bool m10s;
  bool ms8607[2];
} status{};

Data data;
SemaphoreHandle_t i2cMutex;

// Communication data
String constructed_data;

extern void read_m10s(void *);

extern void read_bme(void *);

extern void read_ms(void *);

extern void construct_data(void *);

extern void send_data(void *);

extern void save_data(void *);

extern void print_data(void *);

extern void set_tx_flag();

void setup()
{
  Serial.begin(460800);
  delay(2000);

  i2c.begin();
  i2c.setClock(300000u);

  spi1.begin();

  i2cMutex = xSemaphoreCreateMutex();

  // variable
  static bool state;

  // SD
  state = sd.begin(sd_config);
  Serial.printf("SD CARD: %s\n", state ? "SUCCESS" : "FAILED");
  if (!state)
    while (true)
      ;
  if (file = sd.open("data.csv", O_RDWR | O_CREAT | O_AT_END | O_APPEND))
  {
    Serial.println("File open Success!");
  }
  else
  {
    Serial.println("File open failed!");
  }

  // LoRa
  int16_t ls = lora.begin(params.center_freq,
                          params.bandwidth,
                          params.spreading_factor,
                          params.coding_rate,
                          params.sync_word,
                          params.power,
                          params.preamble_length);
  state = state || lora.explicitHeader();
  state = state || lora.setCRC(true);
  state = state || lora.autoLDRO();

  if (ls == RADIOLIB_ERR_NONE)
  {
    Serial.println("SX1262 initialized successfully!");
  }
  else
  {
    Serial.print("Initialization failed! Error: ");
    Serial.println(ls);
    while (true)
      ;
  }

  s.reserve(256);

  // m10s (0x42)
  status.m10s = m10s.begin(i2c);
  if (status.m10s)
  {
    m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setNavigationFrequency(25, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
  }

  // bme280 (0x77)
  if (!bme.begin(0x77, &i2c))
  {
    Serial.println("Could not find a valid BME280 sensor");
  }

  // ms8607 (0x76, 0x40)
  status.tca9548 = tca.begin(0b00000000);
  Serial.printf("TCA9548 %s\n", status.tca9548 ? "SUCCESS" : "FAILED");
  for (size_t i = 0; i < 2; ++i)
  {
    tca.enableChannel(i);
    tca.selectChannel(i);
    if (tca.isConnected(0x76) && tca.isConnected(0x40))
    {
      status.ms8607[i] = ms8607[i].begin(&i2c);
      ms8607[i].setHumidityResolution(MS8607_HUMIDITY_RESOLUTION_OSR_10b);
      ms8607[i].setPressureResolution(MS8607_PRESSURE_RESOLUTION_OSR_4096);
      Serial.printf("MS8607 EXT %d %s\n", i, status.ms8607[i] ? "SUCCESS" : "FAILED");
    }
    else
      Serial.printf("MS8607 EXT %d NO DEVICE FOUND\n", i);
  }

  xTaskCreate(read_m10s, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_bme, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_ms, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(construct_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(send_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(save_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(print_data, "", 1024, nullptr, 2, nullptr);
  vTaskStartScheduler();
}

void read_m10s(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
      if (m10s.getPVT())
      {
        data.timestamp = m10s.getUnixEpoch(UBLOX_CUSTOM_MAX_WAIT);
        data.gps_latitude = static_cast<double>(m10s.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_longitude = static_cast<double>(m10s.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_altitude = static_cast<float>(m10s.getAltitudeMSL(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
      }
      xSemaphoreGive(i2cMutex);
    }
    DELAY(500);
  }
}

void read_bme(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
      data.temp = bme.readTemperature();
      data.humid = bme.readHumidity();
      data.press = bme.readPressure() / 100.0F;
      xSemaphoreGive(i2cMutex);
    }
    DELAY(200);
  }
}

void read_ms(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
      for (size_t i = 0; i = 2; ++i)
      {
        if (status.ms8607[i])
        {
          tca.selectChannel(i);
          sensors_event_t temp, pressure, humidity;

          ms8607[i].getEvent(&pressure, &temp, &humidity);
          data.msData[i].pres = pressure.pressure;
          data.msData[i].alt = pressure.altitude;
          data.msData[i].temp = temp.temperature;
          data.msData[i].humid = humidity.relative_humidity;
        }
      }
      xSemaphoreGive(i2cMutex);
    }
    DELAY(200);
  }
}

void construct_data(void *)
{
  for (;;)
  {
    constructed_data = "";
    csv_stream_crlf(constructed_data)
        << "<10>"
        << data.counter
        << data.timestamp
        << String(data.gps_latitude, 6)
        << String(data.gps_longitude, 6)
        << String(data.gps_altitude, 4)
        << data.temp
        << data.humid
        << data.press
        << data.msData[0].pres
        << data.msData[0].alt
        << data.msData[0].temp
        << data.msData[0].humid
        << data.msData[1].pres
        << data.msData[1].alt
        << data.msData[1].temp
        << data.msData[1].humid;
    DELAY(1000);
  }
}

void send_data(void *)
{
  for (;;)
  {
    static int16_t state;
    static uint32_t t0, t;

    if (tx_flag)
    {
      tx_flag = false;
      t = millis();

      if (state == RADIOLIB_ERR_NONE)
      {
        Serial.println("Transmission successful!");
        Serial.printf("Used %d ms\n", t - t0);
      }
      else
      {
        Serial.print("Transmission failed! Error: ");
        Serial.println(state);
      }
    }
    else
    {
      lora.startTransmit(constructed_data.c_str());
      t0 = millis();
    }
    DELAY(2000);
    ++data.counter;
  }
}

void save_data(void *)
{
  for (;;)
  {
    file.println(constructed_data);
    file.flush();
    Serial.println("Data written and flushed.");
    DELAY(1000);
  }
}

void print_data(void *)
{
  for (;;)
  {
    Serial.println("----- Data -----");

    Serial.print("Timestamp: ");
    Serial.println(data.timestamp);
    Serial.print("Counter: ");
    Serial.println(data.counter);

    Serial.print("GPS Latitude: ");
    Serial.println(data.gps_latitude, 8);
    Serial.print("GPS Longitude: ");
    Serial.println(data.gps_longitude, 8);
    Serial.print("GPS Altitude: ");
    Serial.println(data.gps_altitude);

    Serial.print("Temp: ");
    Serial.println(data.temp);
    Serial.print("Humid: ");
    Serial.println(data.humid);
    Serial.print("Press: ");
    Serial.println(data.press);

    Serial.println("----------------");

    Serial.print(constructed_data);
    DELAY(1000);
  }
}

void set_tx_flag()
{
  tx_flag = true;
  digitalToggle(PB5);
}

void loop()
{
  DELAY(1);
}
