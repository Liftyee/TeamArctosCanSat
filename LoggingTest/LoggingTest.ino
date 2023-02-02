// CanSat Board Test
// Reminder to edit SdFatConfig.h Config

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

#define RED 20
#define GRN 19
#define BLU 18

#define ON_OFF 13
#define PRIME 12
#define ADC 26

#define S_ERROR 255,0,0
#define S_NORMAL 0,255,0
#define S_WAIT 0,0,255

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

long long lastSync;

void setup() {
  showStatus(S_WAIT);
  Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial && !digitalRead(PRIME)) {
    yield();
  }
  Serial.println("Press PRIME to start");
  while (!Serial.available() && !digitalRead(PRIME)) {
    yield();
  }

  unsigned status;
  status = bme.begin();
  if (!sd.begin(SD_CONFIG)) {
    showStatus(S_ERROR);
    sd.initErrorHalt();
  }

  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      showStatus(S_ERROR);
      while (1) delay(10);
  }
  delayTime = 1000;

  // O_WRITE: write-only to the file
  // O_APPEND: always append to the file
  // O_CREAT: create the file if needed
  // O_SYNC: sync all buffered data after each write (to prevent data loss)
  if (!file.open(DATAFILE, O_WRITE | O_APPEND | O_CREAT | O_SYNC)) {
    showStatus(S_ERROR);
    sd.errorHalt(F("file open failed"));
  }
  
  Serial.println(F("Done."));
  showStatus(S_NORMAL);
  lastSync = millis();
}
//------------------------------------------------------------------------------

void loop() {
    printValues();
    //logVerboseValues();
    logValues();
    checkSync();
    checkEnd();
    delay(delayTime);
}

void checkEnd() {
  if (digitalRead(ON_OFF)) {
    file.close();
    Serial.println("Closing up files");
    showStatus(S_WAIT);
    while (1) {
      yield();
    }
  }
}

#define SYNC_INTERVAL 5000
void checkSync() {
  if (millis()-lastSync > SYNC_INTERVAL) {
    file.close();
    if (!file.open(DATAFILE, O_WRITE | O_APPEND | O_CREAT | O_SYNC)) {
      showStatus(S_ERROR);
      sd.errorHalt(F("file open failed"));
    }
    Serial.println("Reopened file");
    lastSync = millis();
  }
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
      showStatus(S_ERROR);
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

void showStatus(byte r, byte g, byte b) {
  analogWrite(RED, 255-r);
  analogWrite(GRN, 255-g);
  analogWrite(BLU, 255-b);
}
#else  // SPI_DRIVER_SELECT
#error SPI_DRIVER_SELECT must be two in SdFat/SdFatConfig.h
#endif  //SPI_DRIVER_SELECT
