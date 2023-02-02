// An example of the SoftSpiDriver template class.
// This example is for an old Adafruit Data Logging Shield on a Mega.
// Software SPI is required on Mega since this shield connects to pins 10-13.
// This example will also run on an Uno and other boards using software SPI.
//
#include "SdFat.h"
#undef SPI_DRIVER_SELECT
#define SPI_DRIVER_SELECT 2
#if SPI_DRIVER_SELECT == 2  // Must be set in SdFat/SdFatConfig.h

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 1
//
// Chip select may be constant or RAM variable.
const uint8_t SD_CS_PIN = 1;
//
// Pin numbers in templates must be constants.
const uint8_t SOFT_MISO_PIN = 3;
const uint8_t SOFT_MOSI_PIN = 0;
const uint8_t SOFT_SCK_PIN  = 2;

// SdFat software SPI template
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
// Speed argument is ignored for software SPI.
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)
#else  // ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
#endif  // ENABLE_DEDICATED_SPI

#if SD_FAT_TYPE == 0
SdFat sd;
SdFile file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

#define DATAFILE "data.txt"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 10
#define BME_MISO 8
#define BME_MOSI 7
#define BME_CS 9

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
unsigned long delayTime;

void setup() {
  Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial) {
    yield();
  }
  Serial.println("Type any character to start");
  while (!Serial.available()) {
    yield();
  }

  unsigned status;
  status = bme.begin();
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt();
  }

    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
        delayTime = 1000;

  // O_WRITE: write-only to the file
  // O_APPEND: always append to the file
  // O_CREAT: create the file if needed
  // O_SYNC: sync all buffered data after each write (to prevent data loss)
//  if (!file.open(DATAFILE, O_WRITE | O_APPEND | O_CREAT | O_SYNC)) {
//    sd.errorHalt(F("file open failed"));
//  }
  
  Serial.println(F("Done."));
}
//------------------------------------------------------------------------------
void loop() {
    printValues();
    logVerboseValues();
    delay(delayTime);
}

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}


void logVerboseValues() {
    if (!file.open(DATAFILE, O_WRITE | O_APPEND | O_CREAT | O_SYNC)) {
      sd.errorHalt(F("file open failed"));
    }
    file.print("Temperature = ");
    file.print(bme.readTemperature());
    file.println(" °C");

    file.print("Pressure = ");

    file.print(bme.readPressure() / 100.0F);
    file.println(" hPa");

    file.print("Approx. Altitude = ");
    file.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    file.println(" m");

    file.print("Humidity = ");
    file.print(bme.readHumidity());
    file.println(" %");

    file.println();
    file.close();
}


void logValues() {
    file.print("t");
    file.print(bme.readTemperature());

    file.print("p");
    file.print(bme.readPressure() / 100.0F);

    file.print("a");
    file.print(bme.readAltitude(SEALEVELPRESSURE_HPA));

    file.print("h");
    file.print(bme.readHumidity());

    file.println();
}
#else  // SPI_DRIVER_SELECT
#error SPI_DRIVER_SELECT must be two in SdFat/SdFatConfig.h
#endif  //SPI_DRIVER_SELECT
