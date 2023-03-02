#include <SPI.h>
#include <LoRa.h>

// WORKING 26/02/2023h
int counter = 0;


void setup() {
  Serial.begin(9600);
  //while (!Serial);
  delay(2000);
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

void loop() {
  digitalWrite(17, LOW);
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;
  digitalWrite(17, HIGH);
  delay(500);
}
