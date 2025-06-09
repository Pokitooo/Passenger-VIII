#include <arduino.h>
#include <RadioLib.h>

// SPI
SPIClass spi1(PA7, PA6, PA5);

// LoRa data
volatile bool tx_flag = false;
String s;

// LoRa
SPISettings lora_spi_settings(18'000'000, MSBFIRST, SPI_MODE0);

// LoRa Parameters
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
// SX1262 pin connections (adjust pins for your board)
#define LORA_NSS PB8
#define LORA_BUSY PB5
#define LORA_NRST PB9

// Initialize SX1262 module
SX1262 lora = new Module(LORA_NSS, RADIOLIB_NC, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

void set_tx_flag()
{
    tx_flag = true;
    digitalToggle(PB5);
}

void setup()
{
    static bool state;
    pinMode(PB5, OUTPUT);
    Serial.begin();
    spi1.begin();

    delay(2000);
    Serial.println("Hi!");

    // int16_t ls = lora.begin();

    int16_t ls = lora.begin(params.center_freq,
                            params.bandwidth,
                            params.spreading_factor,
                            params.coding_rate,
                            params.sync_word,
                            params.power,
                            params.preamble_length
    );

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
}

void loop()
{

  String str;
  int state = lora.receive(str);
  Serial.println(str);

  
  if (state == RADIOLIB_ERR_NONE) {
    // packet was successfully received
    Serial.println(F("success!"));

    // print the data of the packet
    Serial.print(F("[SX1262] Data:\t\t"));
    Serial.println(str);

    // print the RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("[SX1262] RSSI:\t\t"));
    Serial.print(lora.getRSSI());
    Serial.println(F(" dBm"));

    // print the SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("[SX1262] SNR:\t\t"));
    Serial.print(lora.getSNR());
    Serial.println(F(" dB"));

    // print frequency error
    Serial.print(F("[SX1262] Frequency error:\t"));
    Serial.print(lora.getFrequencyError());
    Serial.println(F(" Hz"));

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));

  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  
}