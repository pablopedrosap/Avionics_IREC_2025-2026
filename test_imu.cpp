#include <SPI.h>

#define PIN_SCLK 16
#define PIN_MISO 14
#define PIN_MOSI 15
#define PIN_CS_IMU 19

#define MPU_WHO_AM_I 0x75

SPIClass spi(IM_SPI);

uint8_t spiRead(uint8_t reg) {
  // Read = reg | 0x80
  uint8_t tx = reg | 0x80;
  digitalWrite(PIN_CS_IMU, LOW);
  spi.transfer(tx);
  uint8_t val = spi.transfer(0x00);
  digitalWrite(PIN_CS_IMU, HIGH);
  return val;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Testing MPU-9250 ===");

  pinMode(PIN_CS_IMU, OUTPUT);
  digitalWrite(PIN_CS_IMU, HIGH);

  spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS_IMU);

  delay(50);

  uint8_t who = spiRead(MPU_WHO_AM_I);
  Serial.printf("MPU-9250 WHO_AM_I = 0x%02X\n", who);

  if (who == 0x71 || who == 0x73)
    Serial.println("MPU-9250 DETECTED");
  else
    Serial.println("MPU-9250 NOT DETECTED");
}

void loop() {}
