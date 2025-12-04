#include <SPI.h>

#define PIN_SCLK 16
#define PIN_MISO 14
#define PIN_MOSI 15
#define PIN_CS_BARO 20

// MS5611 commands
#define MS5611_CMD_RESET 0x1E
#define MS5611_PROM_READ_BASE 0xA0  // A0, A2, A4... AE

SPIClass spi2(MS_SPI);

uint16_t readPROM(uint8_t addr) {
  digitalWrite(PIN_CS_BARO, LOW);
  spi2.transfer(addr);
  uint16_t msb = spi2.transfer(0x00);
  uint16_t lsb = spi2.transfer(0x00);
  digitalWrite(PIN_CS_BARO, HIGH);
  return (msb << 8) | lsb;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Testing MS5611 ===");

  pinMode(PIN_CS_BARO, OUTPUT);
  digitalWrite(PIN_CS_BARO, HIGH);

  spi2.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS_BARO);

  delay(10);

  // Send RESET command
  digitalWrite(PIN_CS_BARO, LOW);
  spi2.transfer(MS5611_CMD_RESET);
  digitalWrite(PIN_CS_BARO, HIGH);
  delay(3); // reset time

  Serial.println("Reading PROM:");
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t addr = MS5611_PROM_READ_BASE + (i * 2);
    uint16_t val = readPROM(addr);
    Serial.printf("PROM[%d] = 0x%04X\n", i, val);
  }
}

void loop() {}
