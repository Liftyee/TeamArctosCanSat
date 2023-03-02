#include <SPI.h>
#include <LoRa.h>

// WORKING 26/02/2023h
int counter = 0;

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define I2C_SDA 20
#define I2C_SCL 21
#define STATUS_L 17

Adafruit_BMP280 bmp; // I2C

#include "Adafruit_SHTC3.h"

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

void initBMP() {
  Serial.println(F("BMP280 init"));
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
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void setup() {
  Serial.begin(115200);
  //while (!Serial);
  delay(2000);
  initBMP();

  Serial.println("SHTC3 init");
  if (! shtc3.begin()) {
    Serial.println("Couldn't find SHTC3");
    while (1) delay(1);
  }
  Serial.println("Found SHTC3 sensor");
  Serial.println("LoRa Sender");
  pinMode(17, OUTPUT);
  SPI1.setTX(15);
  SPI1.setRX(12);
  SPI1.setSCK(14);
  LoRa.setSPI(SPI1);
  LoRa.setPins(10, 11, 8);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  pinMode(25, OUTPUT);
}

char buf[128];
void loop() {

  sensors_event_t humidity, temp;
  
  shtc3.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  digitalWrite(17, LOW);
  Serial.print("Sending packet: ");
  sprintf(buf, "1,%lu,%6.2f,%7.2f,%7.2f,%4.2f", millis(), bmp.readTemperature(), bmp.readPressure(), bmp.readAltitude(1026), humidity.relative_humidity);
  Serial.println(buf);

  // send packet
  LoRa.beginPacket();
  LoRa.print(buf);
  LoRa.endPacket();

  counter++;
  digitalWrite(17, HIGH);
  delay(500);
}
