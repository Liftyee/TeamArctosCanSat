/***************************************************************************
  Main Can Flight Software
  
  Victor Liu @ Team Arctos, 2023
 ***************************************************************************/

#define spd(s) Serial.print(s);Serial.print(",")

#define MSL_ALT_CONST 1026

#define I2C_SDA 20
#define I2C_SCL 21
#define STATUS_L 25

#define LSM_CS 1
#define LSM_SCK 2
#define LSM_MISO 4
#define LSM_MOSI 7
#define LSM_INT1 3
#define LSM_INT2 0

#define LORA_CS 16
#define LORA_RST 17
#define LORA_IRQ 6
#define LORA_FREQ 433920000

#define SD_MISO 12
#define SD_MOSI 15
#define SD_SCK 14
#define SD_CS 13

#define RED_L 5
#define GRN_L 25
#define BLU_L 26

#define ESP_CTL 24
#define RELEASE_SERVO 28

#include <Servo.h>
Servo rServo;

#include <Wire.h>
#include <Adafruit_BMP280.h>

#define LOCKED_POS 120
#define WIGGLE_POS 70
#define OPEN_POS 0
#define DESCENT_THR 10

Adafruit_BMP280 bmp;
float maxAlt;
bool deployed;
void setup() {
  rServo.attach(RELEASE_SERVO);
  Serial.begin(115200);
  delay(2000);
  Serial.print("Secondary Board - Canlet Release Ready");

  rServo.write(LOCKED_POS);
  Wire.setSCL(21);
  Wire.setSDA(20);
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  pinMode(RED_L, OUTPUT);
  digitalWrite(RED_L, HIGH);
  pinMode(GRN_L, OUTPUT);
  if (!status) {
    Serial.println(F("Could not find BMP280 sensor!"));
    //pinMode(RED_L, OUTPUT);
    digitalWrite(RED_L, LOW);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  digitalWrite(GRN_L, LOW);
  float curAlt = bmp.readAltitude(MSL_ALT_CONST);
  maxAlt = curAlt;
  deployed = false;
}

void loop() {
  // get the current altitude
  float curAlt = bmp.readAltitude(MSL_ALT_CONST);
  
  Serial.print(curAlt); Serial.print(", max="); Serial.println(maxAlt);
  digitalWrite(GRN_L, !digitalRead(GRN_L));
  if (!deployed) {
    if (curAlt > maxAlt) {
      maxAlt = curAlt;
    }
    if (curAlt < maxAlt-DESCENT_THR) {
      // we are descending, deploy!
      rServo.write(OPEN_POS);
      deployed = true;
    }
  } else {
      // already deployed, jiggle the last ones out
      rServo.write(OPEN_POS);
      delay(3000);
      rServo.write(WIGGLE_POS);
      delay(3000);
      digitalWrite(GRN_L, LOW); // blink the LED
  }
  delay(100);
}
