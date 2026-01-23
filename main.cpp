#include <Arduino.h>
#include "logic.h"

State ST = SAFE;
uint32_t t_burnout = 0;
float v_hist[256]; int vh_n = 0;

void setup() { Serial.begin(115200); }

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 20) return;
  last = millis();

  Sample s = readSensors();
  bool sepC = sepConfirm();
  bool armed = interlocksOK();

  if (vh_n < 256) v_hist[vh_n++] = s.v_alt;
  else { memmove(v_hist, v_hist + 1, sizeof(v_hist) - sizeof(float)); v_hist[255] = s.v_alt; }

  int stage = getStage(ST);
  
  switch (ST) {
    case SAFE:   if (armed) ST = ARMED; break;
    case ARMED:  if (liftoff_ok(s, stage)) ST = ASCENT; break;
    case ASCENT: if (burnout_ok(s, stage)) { t_burnout = s.t_ms; ST = COAST; } break;
    case COAST: {
      bool timeOK = sep_time_ok(s.t_ms, t_burnout);
      bool altOK  = sep_alt_ok(s, stage);
      bool confOK = sep_confirm_ok(sepC);
      if (vote2of3(timeOK, altOK, confOK)) { setEnableS2(true); ST = S2_ARMED; }
      if (apogee_ok(v_hist, vh_n, stage)) ST = APOGEE;
    } break;
    case APOGEE: pyroFire(1, 60); ST = DROGUE; break;
    case DROGUE: if (main_ok(s, stage)) { pyroFire(2, 80); ST = MAIN; } break;
    case MAIN:   if (s.alt < 50) ST = RECOVERY; break;
    default: break;
  }
}
