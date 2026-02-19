#include <SPI.h>

// ===== MPU9250 Setup =====
#define PIN_CS_MPU 9  // CS pin for MPU9250

// MPU9250 registers
#define MPU9250_WHO_AM_I      0x75
#define MPU9250_PWR_MGMT_1    0x6B
#define MPU9250_ACCEL_XOUT_H  0x3B
#define MPU9250_GYRO_XOUT_H   0x43

SPISettings mpuSPISettings(1000000, MSBFIRST, SPI_MODE0); // 1 MHz safe

uint8_t mpuRead(uint8_t reg) {
  SPI.beginTransaction(mpuSPISettings);
  digitalWrite(PIN_CS_MPU, LOW);
  SPI.transfer(reg | 0x80); // MSB=1 for read
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(PIN_CS_MPU, HIGH);
  SPI.endTransaction();
  return val;
}

void mpuWrite(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(mpuSPISettings);
  digitalWrite(PIN_CS_MPU, LOW);
  SPI.transfer(reg & 0x7F); // MSB=0 for write
  SPI.transfer(val);
  digitalWrite(PIN_CS_MPU, HIGH);
  SPI.endTransaction();
}

void mpuReadBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
  SPI.beginTransaction(mpuSPISettings);
  digitalWrite(PIN_CS_MPU, LOW);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_CS_MPU, HIGH);
  SPI.endTransaction();
}

// ===== MS5611 Setup =====
#define PIN_CS_BARO 10
#define MS5611_CMD_RESET 0x1E
#define MS5611_PROM_READ_BASE 0xA0

uint16_t PROM[8];  // store calibration coefficients

SPISettings baroSPISettings(250000, MSBFIRST, SPI_MODE0); // MS5611 SPI

uint16_t readPROM(uint8_t addr) {
  SPI.beginTransaction(baroSPISettings);
  digitalWrite(PIN_CS_BARO, LOW);
  SPI.transfer(addr);
  uint16_t msb = SPI.transfer(0x00);
  uint16_t lsb = SPI.transfer(0x00);
  digitalWrite(PIN_CS_BARO, HIGH);
  SPI.endTransaction();
  return (msb << 8) | lsb;
}

uint32_t readADC() {
  SPI.beginTransaction(baroSPISettings);
  digitalWrite(PIN_CS_BARO, LOW);
  SPI.transfer(0x00); // CMD_ADC_READ
  uint32_t b1 = SPI.transfer(0);
  uint32_t b2 = SPI.transfer(0);
  uint32_t b3 = SPI.transfer(0);
  digitalWrite(PIN_CS_BARO, HIGH);
  SPI.endTransaction();
  return (b1 << 16) | (b2 << 8) | b3;
}

// Conversion commands
#define CMD_CONVERT_D1_OSR4096 0x48
#define CMD_CONVERT_D2_OSR4096 0x58

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(500);

  // ===== MPU9250 init =====
  pinMode(PIN_CS_MPU, OUTPUT);
  digitalWrite(PIN_CS_MPU, HIGH);
  SPI.begin();

  mpuWrite(MPU9250_PWR_MGMT_1, 0x00); // Wake up MPU
  delay(100);

  uint8_t id = mpuRead(MPU9250_WHO_AM_I);
  Serial.print("MPU9250 WHO_AM_I = 0x");
  Serial.println(id, HEX);

  // ===== MS5611 init =====
  pinMode(PIN_CS_BARO, OUTPUT);
  digitalWrite(PIN_CS_BARO, HIGH);

  // RESET the MS5611
  SPI.beginTransaction(baroSPISettings);
  digitalWrite(PIN_CS_BARO, LOW);
  SPI.transfer(MS5611_CMD_RESET);
  digitalWrite(PIN_CS_BARO, HIGH);
  SPI.endTransaction();
  delay(3);

  // Read PROM coefficients
  Serial.println("Reading MS5611 PROM:");
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t addr = MS5611_PROM_READ_BASE + (i * 2);
    PROM[i] = readPROM(addr);
    Serial.printf("PROM[%d] = 0x%04X\n", i, PROM[i]);
  }
}

void loop() {
  // ===== MPU9250 Data =====
  uint8_t buf[14];
  mpuReadBytes(MPU9250_ACCEL_XOUT_H, buf, 14);

  int16_t ax = (buf[0] << 8) | buf[1];
  int16_t ay = (buf[2] << 8) | buf[3];
  int16_t az = (buf[4] << 8) | buf[5];
  int16_t temp_mpu = (buf[6] << 8) | buf[7];
  int16_t gx = (buf[8] << 8) | buf[9];
  int16_t gy = (buf[10] << 8) | buf[11];
  int16_t gz = (buf[12] << 8) | buf[13];

  // Convert MPU9250 temperature to °C
  float temp_mpu_c = temp_mpu / 333.87 + 21.0;

  // ===== MS5611 Data =====
  // Pressure
  SPI.beginTransaction(baroSPISettings);
  digitalWrite(PIN_CS_BARO, LOW);
  SPI.transfer(CMD_CONVERT_D1_OSR4096);
  digitalWrite(PIN_CS_BARO, HIGH);
  SPI.endTransaction();
  delay(10);
  uint32_t D1 = readADC();

  // Temperature
  SPI.beginTransaction(baroSPISettings);
  digitalWrite(PIN_CS_BARO, LOW);
  SPI.transfer(CMD_CONVERT_D2_OSR4096);
  digitalWrite(PIN_CS_BARO, HIGH);
  SPI.endTransaction();
  delay(10);
  uint32_t D2 = readADC();

  // First-order temperature calculation
  float dT = (float)D2 - ((float)PROM[5] * 256.0f);
  float TEMP = 2000.0f + dT * (float)PROM[6] / 8388608.0f; // TEMP in 0.01 °C
  float OFF  = (float)PROM[2] * 65536.0f + ((float)PROM[4] * dT) / 128.0f;
  float SENS = (float)PROM[1] * 32768.0f + ((float)PROM[3] * dT) / 256.0f;

  // Second-order compensation
  float T2=0, OFF2=0, SENS2=0;
  if (TEMP < 2000) {
    T2 = (dT * dT) / 2147483648.0f;
    OFF2 = 5.0f * ((TEMP - 2000.0f) * (TEMP - 2000.0f)) / 2.0f;
    SENS2 = 5.0f * ((TEMP - 2000.0f) * (TEMP - 2000.0f)) / 4.0f;
    if (TEMP < -1500) {
      OFF2  += 7.0f * ((TEMP + 1500.0f) * (TEMP + 1500.0f));
      SENS2 += 11.0f * ((TEMP + 1500.0f) * (TEMP + 1500.0f)) / 2.0f;
    }
  }
  TEMP -= T2;
  OFF  -= OFF2;
  SENS -= SENS2;
  float P = ((D1 * SENS) / 2097152.0f - OFF) / 32768.0f;
  float P_mbar = P / 100.0f;

  // ===== Print all data =====
  Serial.printf("MPU Acc: %6d %6d %6d | Gyro: %6d %6d %6d | Temp: %.2f C | ",
                ax, ay, az, gx, gy, gz, temp_mpu_c);
  Serial.printf("MS5611 T=%.2f C P=%.2f mbar\n", TEMP / 100.0f, P_mbar);

  delay(500);
}
