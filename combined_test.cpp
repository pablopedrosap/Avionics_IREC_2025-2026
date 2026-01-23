/*
  FLIGHT COMPUTER (STARTER / TEST) — MPU9250 + MS5611
  Dual-deploy style flight-state logic (PAD -> BOOST -> COAST -> APOGEE -> DROGUE -> MAIN -> LANDED)

  Key points:
  - 200 Hz fixed loop
  - Non-blocking MS5611 conversions (pressure @ 100 Hz, temp @ 4 Hz)
  - AGL altitude uses calibrated ground pressure P0 (so your altitude is stable)
  - Pyro functions are STUBS (prints only). Do NOT fly this as-is.

  Wiring:
  - MPU9250 SPI CS: PIN_CS_MPU
  - MS5611  SPI CS: PIN_CS_BARO
  - MOSI/MISO/SCK shared
*/

#include <SPI.h>
#include <math.h>

// ====================== Configuration ======================
#define LOOP_RATE_HZ            200
#define LOOP_PERIOD_US          (1000000UL / LOOP_RATE_HZ)

#define BARO_PRESSURE_RATE_HZ   100     // pressure updates per second
#define BARO_TEMP_RATE_HZ       4       // temperature updates per second

#define PRINT_RATE_HZ           2
#define PRINT_PERIOD_MS         (1000UL / PRINT_RATE_HZ)

// ====================== Flight Parameters ======================
#define LAUNCH_ACCEL_THRESHOLD_G      2.5f
#define LAUNCH_MIN_SAMPLES            8

#define BURNOUT_ACCEL_THRESHOLD_G     1.2f
#define BURNOUT_MIN_VELOCITY_MPS      10.0f

#define APOGEE_VEL_BAND_MPS           10.0f
#define APOGEE_CONFIRM_SAMPLES        10

#define MAIN_DEPLOY_ALTITUDE_FT       1500.0f

#define LANDING_VEL_MPS               2.0f
#define LANDING_CONFIRM_SAMPLES       200
#define LANDING_MAX_ALT_FT            100.0f

// ====================== Pins ======================
#define PIN_CS_MPU   9
#define PIN_CS_BARO  10

// #define PIN_DROGUE_PYRO  X
// #define PIN_MAIN_PYRO    X

// ====================== MPU9250 Registers ======================
#define MPU9250_WHO_AM_I      0x75
#define MPU9250_PWR_MGMT_1    0x6B
#define MPU9250_ACCEL_CONFIG  0x1C
#define MPU9250_GYRO_CONFIG   0x1B
#define MPU9250_ACCEL_XOUT_H  0x3B

// ====================== MS5611 Commands ======================
#define MS5611_CMD_RESET       0x1E
#define MS5611_PROM_READ_BASE  0xA0
#define MS5611_CMD_ADC_READ    0x00

// OSR1024 for speed (~2.3 ms typical)
#define CMD_CONVERT_D1_OSR1024  0x44
#define CMD_CONVERT_D2_OSR1024  0x54
#define BARO_CONVERSION_US      2500UL

// ====================== SPI Settings ======================
SPISettings mpuSPISettings(8000000, MSBFIRST, SPI_MODE0);
SPISettings baroSPISettings(4000000, MSBFIRST, SPI_MODE0);

// ====================== Flight State Machine ======================
enum FlightState {
  STATE_PAD,
  STATE_BOOST,
  STATE_COAST,
  STATE_APOGEE,
  STATE_DROGUE_DESCENT,
  STATE_MAIN_DESCENT,
  STATE_LANDED
};
FlightState flightState = STATE_PAD;
const char* stateNames[] = {"PAD", "BOOST", "COAST", "APOGEE", "DROGUE", "MAIN", "LANDED"};

// ====================== Baro State Machine ======================
enum BaroState { BARO_IDLE, BARO_CONVERTING_PRESSURE, BARO_CONVERTING_TEMP };
BaroState baroState = BARO_IDLE;

uint32_t baroConversionStartUs = 0;
bool baroNewPressure = false;

uint32_t lastBaroPressureStartUs = 0;
uint32_t lastBaroTempStartUs = 0;
const uint32_t BARO_PRESSURE_PERIOD_US = 1000000UL / BARO_PRESSURE_RATE_HZ;
const uint32_t BARO_TEMP_PERIOD_US     = 1000000UL / BARO_TEMP_RATE_HZ;

// ====================== MS5611 Calibration / Data ======================
uint16_t PROM[8] = {0};

uint32_t D1_pressure = 0;
uint32_t D2_temperature = 0;

float pressure_Pa = 101325.0f;
float temperature_C = 25.0f;

float groundPressure_Pa = 101325.0f; // calibrated on pad

// ====================== IMU Data ======================
int16_t ax_raw = 0, ay_raw = 0, az_raw = 0;
int16_t gx_raw = 0, gy_raw = 0, gz_raw = 0;

float accel_g = 1.0f;
float accel_vertical_g = 1.0f;

// ====================== Derived Flight Data ======================
float altitude_AGL_m = 0.0f;
float altitude_AGL_ft = 0.0f;

float velocity_m_s = 0.0f;
float maxAltitude_ft = 0.0f;

float prevAltitude_m = 0.0f;
uint32_t prevAltitudeTimeUs = 0;

// ====================== Counters ======================
int apogeeConfirmCount = 0;
int landingConfirmCount = 0;
int launchConfirmCount = 0;

// ====================== Timing ======================
uint32_t lastLoopUs = 0;
uint32_t loopCount = 0;
uint32_t lastPrintMs = 0;
uint32_t flightStartMs = 0;

// =====================================================
// MPU9250 SPI helpers
// =====================================================
void mpuWrite(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(mpuSPISettings);
  digitalWrite(PIN_CS_MPU, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(PIN_CS_MPU, HIGH);
  SPI.endTransaction();
}

uint8_t mpuRead(uint8_t reg) {
  SPI.beginTransaction(mpuSPISettings);
  digitalWrite(PIN_CS_MPU, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(PIN_CS_MPU, HIGH);
  SPI.endTransaction();
  return val;
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

// =====================================================
// MS5611 SPI helpers
// =====================================================
void baroSendCommand(uint8_t cmd) {
  SPI.beginTransaction(baroSPISettings);
  digitalWrite(PIN_CS_BARO, LOW);
  SPI.transfer(cmd);
  digitalWrite(PIN_CS_BARO, HIGH);
  SPI.endTransaction();
}

uint16_t baroReadPROM(uint8_t addr) {
  SPI.beginTransaction(baroSPISettings);
  digitalWrite(PIN_CS_BARO, LOW);
  SPI.transfer(addr);
  uint16_t msb = SPI.transfer(0x00);
  uint16_t lsb = SPI.transfer(0x00);
  digitalWrite(PIN_CS_BARO, HIGH);
  SPI.endTransaction();
  return (msb << 8) | lsb;
}

uint32_t baroReadADC() {
  SPI.beginTransaction(baroSPISettings);
  digitalWrite(PIN_CS_BARO, LOW);
  SPI.transfer(MS5611_CMD_ADC_READ);
  uint32_t b1 = SPI.transfer(0);
  uint32_t b2 = SPI.transfer(0);
  uint32_t b3 = SPI.transfer(0);
  digitalWrite(PIN_CS_BARO, HIGH);
  SPI.endTransaction();
  return (b1 << 16) | (b2 << 8) | b3;
}

// =====================================================
// MS5611 compensation
// =====================================================
void calculatePressure() {
  if (D1_pressure == 0 || D2_temperature == 0) return;

  float dT   = (float)D2_temperature - ((float)PROM[5] * 256.0f);
  float TEMP = 2000.0f + dT * (float)PROM[6] / 8388608.0f; // 0.01C
  float OFF  = (float)PROM[2] * 65536.0f + ((float)PROM[4] * dT) / 128.0f;
  float SENS = (float)PROM[1] * 32768.0f + ((float)PROM[3] * dT) / 256.0f;

  // Second order (mainly for low temps)
  float T2 = 0.0f, OFF2 = 0.0f, SENS2 = 0.0f;
  if (TEMP < 2000.0f) {
    T2 = (dT * dT) / 2147483648.0f;
    float td = TEMP - 2000.0f;
    OFF2  = 5.0f * td * td / 2.0f;
    SENS2 = 5.0f * td * td / 4.0f;
    if (TEMP < -1500.0f) {
      float td2 = TEMP + 1500.0f;
      OFF2  += 7.0f  * td2 * td2;
      SENS2 += 11.0f * td2 * td2 / 2.0f;
    }
  }
  TEMP -= T2;
  OFF  -= OFF2;
  SENS -= SENS2;

  float P = ((D1_pressure * SENS) / 2097152.0f - OFF) / 32768.0f; // Pa
  pressure_Pa = P;
  temperature_C = TEMP / 100.0f;
}

// =====================================================
// Baro non-blocking scheduler + state machine
// =====================================================
void baroUpdate() {
  uint32_t now = micros();

  // Start conversions (when idle) on schedule
  if (baroState == BARO_IDLE) {
    // Temp has priority if it's due
    if ((now - lastBaroTempStartUs) >= BARO_TEMP_PERIOD_US) {
      lastBaroTempStartUs = now;
      baroSendCommand(CMD_CONVERT_D2_OSR1024);
      baroState = BARO_CONVERTING_TEMP;
      baroConversionStartUs = now;
      return;
    }

    // Pressure schedule
    if ((now - lastBaroPressureStartUs) >= BARO_PRESSURE_PERIOD_US) {
      lastBaroPressureStartUs = now;
      baroSendCommand(CMD_CONVERT_D1_OSR1024);
      baroState = BARO_CONVERTING_PRESSURE;
      baroConversionStartUs = now;
      return;
    }
  }

  // Finish conversions when ready
  if (baroState == BARO_CONVERTING_PRESSURE) {
    if ((now - baroConversionStartUs) >= BARO_CONVERSION_US) {
      D1_pressure = baroReadADC();
      baroState = BARO_IDLE;
      calculatePressure();
      baroNewPressure = true;
    }
  } else if (baroState == BARO_CONVERTING_TEMP) {
    if ((now - baroConversionStartUs) >= BARO_CONVERSION_US) {
      D2_temperature = baroReadADC();
      baroState = BARO_IDLE;
      // Next pressure calculation will use this updated temp
    }
  }
}

// =====================================================
// Altitude + velocity (AGL from P relative to ground P0)
// =====================================================
static inline float pressureToAltitudeAGL_m(float P, float P0) {
  if (P <= 0.0f || P0 <= 0.0f) return 0.0f;
  return 44330.0f * (1.0f - powf(P / P0, 0.1903f));
}

void updateAltitudeAndVelocity() {
  if (!baroNewPressure) return;
  baroNewPressure = false;

  uint32_t now = micros();

  altitude_AGL_m  = pressureToAltitudeAGL_m(pressure_Pa, groundPressure_Pa);
  altitude_AGL_ft = altitude_AGL_m * 3.28084f;

  if (altitude_AGL_ft > maxAltitude_ft) maxAltitude_ft = altitude_AGL_ft;

  if (prevAltitudeTimeUs > 0) {
    float dt = (now - prevAltitudeTimeUs) / 1000000.0f;
    if (dt > 0.002f) {
      float newVel = (altitude_AGL_m - prevAltitude_m) / dt;
      velocity_m_s = 0.85f * velocity_m_s + 0.15f * newVel;
    }
  }
  prevAltitude_m = altitude_AGL_m;
  prevAltitudeTimeUs = now;
}

// =====================================================
// IMU read
// =====================================================
void readIMU() {
  uint8_t buf[14];
  mpuReadBytes(MPU9250_ACCEL_XOUT_H, buf, 14);

  ax_raw = (buf[0] << 8) | buf[1];
  ay_raw = (buf[2] << 8) | buf[3];
  az_raw = (buf[4] << 8) | buf[5];

  gx_raw = (buf[8] << 8) | buf[9];
  gy_raw = (buf[10] << 8) | buf[11];
  gz_raw = (buf[12] << 8) | buf[13];

  // +/-8g => 4096 LSB/g
  float ax_g = ax_raw / 4096.0f;
  float ay_g = ay_raw / 4096.0f;
  float az_g = az_raw / 4096.0f;

  accel_g = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  accel_vertical_g = az_g; // assumes Z aligned-ish to vertical on pad
}

// =====================================================
// Pyro stubs (prints only)
// =====================================================
void deployDrogue() {
  Serial.println("!!! DROGUE DEPLOY COMMAND (STUB) !!!");
}

void deployMain() {
  Serial.println("!!! MAIN DEPLOY COMMAND (STUB) !!!");
}

// =====================================================
// Flight state machine
// =====================================================
void updateFlightState() {
  switch (flightState) {
    case STATE_PAD: {
      if (accel_g > LAUNCH_ACCEL_THRESHOLD_G) {
        launchConfirmCount++;
        if (launchConfirmCount >= LAUNCH_MIN_SAMPLES) {
          flightState = STATE_BOOST;
          flightStartMs = millis();
          Serial.println(">>> LAUNCH DETECTED <<<");
        }
      } else {
        launchConfirmCount = 0;
      }
    } break;

    case STATE_BOOST: {
      if (accel_g < BURNOUT_ACCEL_THRESHOLD_G && velocity_m_s > BURNOUT_MIN_VELOCITY_MPS) {
        flightState = STATE_COAST;
        Serial.println(">>> BURNOUT - COASTING <<<");
      }
    } break;

    case STATE_COAST: {
      // Don’t even consider apogee until we’ve clearly climbed
      if (maxAltitude_ft > 300.0f) {
        if (fabsf(velocity_m_s) < APOGEE_VEL_BAND_MPS) {
          apogeeConfirmCount++;
          if (apogeeConfirmCount >= APOGEE_CONFIRM_SAMPLES) {
            flightState = STATE_APOGEE;
            Serial.println(">>> APOGEE DETECTED <<<");
            Serial.printf(">>> MAX ALTITUDE: %.0f ft AGL <<<\n", maxAltitude_ft);
            deployDrogue();
          }
        } else {
          apogeeConfirmCount = 0;
        }
      }
    } break;

    case STATE_APOGEE:
      flightState = STATE_DROGUE_DESCENT;
      break;

    case STATE_DROGUE_DESCENT: {
      if (altitude_AGL_ft <= MAIN_DEPLOY_ALTITUDE_FT) {
        flightState = STATE_MAIN_DESCENT;
        Serial.printf(">>> MAIN DEPLOY at %.0f ft AGL <<<\n", altitude_AGL_ft);
        deployMain();
      }
    } break;

    case STATE_MAIN_DESCENT: {
      if (fabsf(velocity_m_s) < LANDING_VEL_MPS && altitude_AGL_ft < LANDING_MAX_ALT_FT) {
        landingConfirmCount++;
        if (landingConfirmCount >= LANDING_CONFIRM_SAMPLES) {
          flightState = STATE_LANDED;
          Serial.println(">>> LANDED <<<");
        }
      } else {
        landingConfirmCount = 0;
      }
    } break;

    case STATE_LANDED:
      break;
  }
}

// =====================================================
// Ground calibration (blocking, only on pad)
// =====================================================
void calibrateGround() {
  Serial.println("Calibrating ground pressure P0... keep rocket still.");

  // Ensure temp is valid
  baroSendCommand(CMD_CONVERT_D2_OSR1024);
  delay(3);
  D2_temperature = baroReadADC();

  const int N = 60;
  float sumP = 0.0f;

  for (int i = 0; i < N; i++) {
    baroSendCommand(CMD_CONVERT_D1_OSR1024);
    delay(3);
    D1_pressure = baroReadADC();
    calculatePressure();
    sumP += pressure_Pa;
    delay(10);
  }

  groundPressure_Pa = sumP / (float)N;

  // Reset filters/baselines
  altitude_AGL_m = 0.0f;
  altitude_AGL_ft = 0.0f;
  velocity_m_s = 0.0f;
  maxAltitude_ft = 0.0f;
  prevAltitude_m = 0.0f;
  prevAltitudeTimeUs = 0;

  Serial.printf("Ground P0: %.1f Pa\n", groundPressure_Pa);
  Serial.println("Calibration complete.\n");
}

// =====================================================
// Setup
// =====================================================
void setup() {
  // High baud reduces blocking time from printing
  Serial.begin(921600);
  while (!Serial) { delay(10); }
  delay(100);

  Serial.println("========================================");
  Serial.println("  FLIGHT COMPUTER - STARTER (DUAL DEPLOY)");
  Serial.println("  MPU9250 + MS5611 | 30,000 ft class");
  Serial.println("========================================");
  Serial.printf("Loop: %d Hz | BaroP: %d Hz | BaroT: %d Hz\n",
                LOOP_RATE_HZ, BARO_PRESSURE_RATE_HZ, BARO_TEMP_RATE_HZ);

  // SPI + CS pins
  pinMode(PIN_CS_MPU, OUTPUT);
  pinMode(PIN_CS_BARO, OUTPUT);
  digitalWrite(PIN_CS_MPU, HIGH);
  digitalWrite(PIN_CS_BARO, HIGH);
  SPI.begin();

  // ===== MPU init =====
  mpuWrite(MPU9250_PWR_MGMT_1, 0x00);
  delay(100);
  mpuWrite(MPU9250_ACCEL_CONFIG, 0x10); // +/-8g
  mpuWrite(MPU9250_GYRO_CONFIG,  0x10); // +/-1000 dps

  uint8_t id = mpuRead(MPU9250_WHO_AM_I);
  Serial.printf("MPU9250 WHO_AM_I: 0x%02X %s\n",
                id, (id == 0x71 || id == 0x73) ? "(OK)" : "(?)");

  // ===== MS5611 init =====
  baroSendCommand(MS5611_CMD_RESET);
  delay(5);

  for (uint8_t i = 0; i < 8; i++) {
    PROM[i] = baroReadPROM(MS5611_PROM_READ_BASE + (i * 2));
  }
  Serial.println("MS5611 PROM read OK.");

  // Prime with one temp + one pressure so calculations are valid
  baroSendCommand(CMD_CONVERT_D2_OSR1024);
  delay(3);
  D2_temperature = baroReadADC();

  baroSendCommand(CMD_CONVERT_D1_OSR1024);
  delay(3);
  D1_pressure = baroReadADC();
  calculatePressure();

  // Ground calibration for P0
  calibrateGround();

  // Initialize baro scheduling so it starts immediately
  uint32_t now = micros();
  lastBaroPressureStartUs = now - BARO_PRESSURE_PERIOD_US;
  lastBaroTempStartUs     = now - BARO_TEMP_PERIOD_US;

  Serial.println("STATE: PAD — waiting for launch...\n");

  lastLoopUs = micros();
  lastPrintMs = millis();
}

// =====================================================
// Main loop
// =====================================================
void loop() {
  uint32_t nowUs = micros();

  // Fixed-rate loop (catch up if needed)
  while ((uint32_t)(nowUs - lastLoopUs) >= LOOP_PERIOD_US) {
    lastLoopUs += LOOP_PERIOD_US;
    loopCount++;

    // Fast sensor reads
    readIMU();

    // Non-blocking baro update
    baroUpdate();

    // Update altitude/velocity only when new pressure arrives
    updateAltitudeAndVelocity();

    // Flight logic
    updateFlightState();

    nowUs = micros();
  }

  // Slow telemetry printing (kept outside the fixed loop)
  if ((millis() - lastPrintMs) >= PRINT_PERIOD_MS) {
    lastPrintMs += PRINT_PERIOD_MS;

    float actualHz = loopCount * (1000.0f / (float)PRINT_PERIOD_MS);
    loopCount = 0;

    float tPlus = (flightState > STATE_PAD) ? (millis() - flightStartMs) / 1000.0f : 0.0f;

    Serial.printf("[%s] T+%.1fs | Alt: %.0f ft | Vel: %.1f m/s | Acc: %.2f g | Max: %.0f ft | %.0f Hz | P: %.0f Pa\n",
                  stateNames[flightState],
                  tPlus,
                  altitude_AGL_ft,
                  velocity_m_s,
                  accel_g,
                  maxAltitude_ft,
                  actualHz,
                  pressure_Pa);
  }
}
