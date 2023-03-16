/***************************************************************************
  Canlet Flight Software
  
  Victor Liu @ Team Arctos, 2023
 ***************************************************************************/

#include "SdFat.h"
#include "sdios.h"
#include "BufferedPrint.h"

#define DEBUG
#define pf(s) file.printField(s,',')
#define spd(s) Serial.print(s);Serial.print(",")

#define MSL_ALT_CONST 1013.25

#define I2C_SDA 20
#define I2C_SCL 21
#define STATUS_L 17

#define LSM_CS 1
#define LSM_SCK 2
#define LSM_MISO 4
#define LSM_MOSI 7
#define LSM_INT1 3
#define LSM_INT2 0

#define LORA_CS 10
#define LORA_RST 11
#define LORA_IRQ 8
#define LORA_FREQ 433920000

#define SD_MISO 12
#define SD_MOSI 15
#define SD_SCK 14
#define SD_CS 13

#define FORCE_SENS 29

#define BMP_FLAG 0b10000000
#define SHT_FLAG 0b01000000
#define IMU_FLAG 0b00100000
#define VEML_FLAG 0b00010000
#define SD_FLAG 0b00001000
#define LORA_FLAG 0b00000100
byte sensFlags; // flags: BMP280, SHTC3, IMU, VEML, SD, LoRa

#define fileName "data.csv"

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

#define SD_FAT_TYPE 1

#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SD_CS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

#define SPI_CLOCK SD_SCK_MHZ(10)
//#undef ENABLE_DEDICATED_SPI
//#define ENABLE_DEDICATED_SPI 0
// Try to select the best SD card configuration.
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK, &SPI1)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK, &SPI1)
#endif  // HAS_SDIO_CLASS

// Set PRE_ALLOCATE true to pre-allocate file clusters.
const bool PRE_ALLOCATE = true;

// Set SKIP_FIRST_LATENCY true if the first read/write to the SD can
// be avoid by writing a file header or reading the first record.
const bool SKIP_FIRST_LATENCY = true;

// Size of read/write.
const size_t BUF_SIZE = 512;

// File size in MB where MB = 1,000,000 bytes.
const uint32_t FILE_SIZE_MB = 5;

// Write pass count.
const uint8_t WRITE_COUNT = 2;

// Read pass count.
const uint8_t READ_COUNT = 2;
//==============================================================================
// End of configuration constants.
//------------------------------------------------------------------------------
// File size in bytes.
const uint32_t FILE_SIZE = 1000000UL*FILE_SIZE_MB;

// Insure 4-byte alignment.
uint32_t buf32[(BUF_SIZE + 3)/4];
uint8_t* buf = (uint8_t*)buf32;

#if SD_FAT_TYPE == 0
SdFat sd;
typedef File file_t;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
SdExFat sd;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
SdFs sd;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

//------------------------------------------------------------------------------
// Store error strings in flash to save RAM.
#define error(s) sd.errorHalt(&Serial, F(s))
void clearSerialInput() {
  uint32_t m = micros();
  do {
    if (Serial.read() >= 0) {
      m = micros();
    }
  } while (micros() - m < 10000);
}
//------------------------------------------------------------------------------

BufferedPrint<file_t, 64> file;
file_t baseFile;
void initSD() {
  SPI1.setRX(SD_MISO);
  SPI1.setTX(SD_MOSI);
  SPI1.setSCK(SD_SCK);
  SPI1.setCS(SD_CS);

  Serial.print("Dedicated SPI is "); Serial.println(ENABLE_DEDICATED_SPI);
  if (!sd.begin(SD_CONFIG)) {
    #ifdef DEBUG
      sd.initErrorPrint(&Serial);
    #endif
    Serial.print("SD init failed");
    return;
  }

  if (sd.fatType() == FAT_TYPE_EXFAT) {
    Serial.println(F("Type is exFAT"));
  } else {
    Serial.println(int(sd.fatType()));
  }

  Serial.println(sd.card()->sectorCount()*512E-9);

  if (!baseFile.open(fileName, O_RDWR | O_CREAT | O_APPEND | O_SYNC)) {
    Serial.println("Couldn't open file!");
    return;
  }

  file.begin(&baseFile);

  Serial.println("SD init success!");
  sensFlags |= SD_FLAG;
}

void initLoRa() {
  // NOTE: Requires SPI pins to already be set! 
  LoRa.setSPI(SPI1);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Starting LoRa failed!");
    return;
  }  
  sensFlags |= LORA_FLAG;
}

#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C

#include "Adafruit_VEML7700.h"
Adafruit_VEML7700 veml = Adafruit_VEML7700();

// Actually needs Liftyee/GXHTC3 library! 
#include "Adafruit_SHTC3.h"

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

#include <Adafruit_LSM6DSOX.h>


Adafruit_LSM6DSOX sox;

void initIMU() {
  SPI.setRX(LSM_MISO);
  SPI.setTX(LSM_MOSI);
  SPI.setSCK(LSM_SCK);
  SPI.setCS(LSM_CS);
  if (!sox.begin_SPI(LSM_CS)) {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSOX chip");
    return;
  }
  Serial.println("LSM6DSOX Found!");

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

  sox.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  sox.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
  sensFlags |= IMU_FLAG;
}

void initBMP() {
  Serial.println(F("finding BMP280..."));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  Wire.setSCL(21);
  Wire.setSDA(20);
  status = bmp.begin(BMP280_ADDRESS_ALT);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    return;
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  Serial.println("BMP280 found!");
  sensFlags |= BMP_FLAG;
}

void initSHT() {
  Serial.println("SHTC3 test");
  if (! shtc3.begin()) {
    Serial.println("Couldn't find SHTC3");
    return;
  }
  Serial.println("Found SHTC3 sensor");
  sensFlags |= SHT_FLAG;
}

void initVEML() {
  if (!veml.begin()) {
    Serial.println("VEML Sensor not found");
  }
  Serial.println("VEML Sensor found");
  sensFlags |= VEML_FLAG;
}

long startMillis;
void setup() {
  sensFlags = 0;
  pinMode(STATUS_L, OUTPUT);
  digitalWrite(STATUS_L, LOW);
  Serial.begin(115200);
  delay(2000);
  Serial.print("Waking up...");
  initSD();
  Serial.print("SD done...");
  initBMP();
  Serial.print("BMP done...");
  initSHT();
  Serial.print("SHT done...");
  //initSD();
  //delay(2000);
  //initIMU();
  startMillis = millis();
  Serial.print("starting reading from sensor...");
}

long long i = 0;

int forceV;
void loop() {
  digitalWrite(STATUS_L, !digitalRead(STATUS_L));
  Serial.print("reading..."); Serial.print(sensFlags, BIN);
  getBMPData();
  getSHTData();
//  if (sensFlags & IMU_FLAG) {
//    getIMUData();
//  }
//  if (sensFlags & VEML_FLAG) {
//    getVEMLData();
//  }
  forceV = analogRead(FORCE_SENS);
  delay(100);

  printShortData();
  if (sensFlags | SD_FLAG) {
    Serial.print("writing...");
    logData();     
  }
  if (sensFlags | LORA_FLAG) {
    transmitData();
  }
}
sensors_event_t humidity, temp, accel, gyro, itemp;
float btemp, pressure, altitude, lux;


void getBMPData() {
  btemp = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(MSL_ALT_CONST);
}

void getIMUData() {
  sox.getEvent(&accel, &gyro, &itemp);
}

void getSHTData() {
  shtc3.getEvent(&humidity, &temp);
}

void getVEMLData() {
  // to read lux using automatic method, specify VEML_LUX_AUTO
  lux = veml.readLux(VEML_LUX_AUTO);

  Serial.println("------------------------------------");
  Serial.print("Lux = "); Serial.println(lux);
  Serial.println("Settings used for reading:");
  Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }
  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }  
}

void logData() {
  pf(millis());
  pf(temp.temperature);
  pf(humidity.relative_humidity);
  pf(pressure);
  pf(altitude);
  pf(lux);
  pf(accel.acceleration.x);
  pf(accel.acceleration.y);
  pf(accel.acceleration.z);
  pf(gyro.gyro.x);
  pf(gyro.gyro.y);
  pf(gyro.gyro.z); 
  pf(forceV);

  file.println();
  file.sync();
}

void transmitData() {
  LoRa.beginPacket();
  LoRa.print(temp.temperature); LoRa.print(",");
  LoRa.print(pressure); LoRa.print(",");
  LoRa.print(altitude); LoRa.print(",");
  LoRa.print(sensFlags);
  LoRa.endPacket();
}

void printShortData(){
  spd(millis());
  spd(temp.temperature);
  spd(humidity.relative_humidity);
  spd(pressure);
  spd(altitude);
  spd(lux);
  spd(accel.acceleration.x);
  spd(accel.acceleration.y);
  spd(accel.acceleration.z);
  spd(gyro.gyro.x);
  spd(gyro.gyro.y);
  spd(gyro.gyro.z); 
  spd(forceV);
}

void printData() {
    shtc3.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  
    Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();

      //  /* Get a new normalized sensor event */
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t itemp;
    sox.getEvent(&accel, &gyro, &itemp);

    Serial.print("\t\tTemperature ");
    Serial.print(itemp.temperature);
    Serial.println(" deg C");

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");

    /* Display the results (rotation is measured in rad/s) */
    Serial.print("\t\tGyro X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" radians/s ");
    Serial.println();
}
