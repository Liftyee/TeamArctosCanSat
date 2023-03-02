#include <SPI.h>
#include <LoRa.h>

int counter = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");
  SPI1.setTX(15);
  SPI1.setRX(12);
  SPI1.setSCK(14);
  LoRa.setSPI(SPI1);
  LoRa.setPins(16, 17, 6);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  pinMode(25, OUTPUT);
}

void loop() {
  digitalWrite(25, HIGH);
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;
  digitalWrite(25, LOW);
  delay(500);
}
