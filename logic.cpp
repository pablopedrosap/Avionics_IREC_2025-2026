#pragma once
#include <Arduino.h>

// Units: t_ms= time in ms, alt= altitude in m, v_alt= vertical velocity in m/s, a_mag= acceleration inm/s^2
struct Sample { uint32_t t_ms; float alt, v_alt, a_mag; };
enum State { SAFE, ARMED, ASCENT, COAST, S2_ARMED, APOGEE, DROGUE, MAIN, RECOVERY };

// Stage-specific constraints
struct StageConstraints {
  float liftoff_accel;      // m/s^2
  float burnout_accel;       // m/s^2
  float sep_alt_min;         // m
  float sep_alt_max;         // m
  float apogee_v_threshold;  // m/s
  float main_alt;            // m
  float main_v_descend;      // m/s
};

const StageConstraints S1 = {
  .liftoff_accel = 15.0f,      // ~1.5g (S1 max: 16.2g from OpenRocket)
  .burnout_accel = 5.0f,       // ~0.5g 
  .sep_alt_min = 150.0f,       // m (safety margin, OpenRocket target: 200m)
  .sep_alt_max = 300.0f,       // m (safety margin, OpenRocket target: 200m)
  .apogee_v_threshold = 2.0f,  // m/s 
  .main_alt = 310.0f,          // m (safety margin, OpenRocket target: 304.8m / 1000ft)
  .main_v_descend = -5.0f      // m/s 
};

const StageConstraints S2 = {
  .liftoff_accel = 15.0f,      // ~1.5g (S2 max: 12.1g from OpenRocket)
  .burnout_accel = 5.0f,       // ~0.5g 
  .apogee_v_threshold = 2.0f,  // m/s 
  .main_alt = 310.0f,          // m (safety margin, OpenRocket target: 304.8m / 1000ft)
  .main_v_descend = -5.0f      // m/s 
};

// Get current stage (1 or 2) based on state
int getStage(State st) {
  return (st >= S2_ARMED) ? 2 : 1;
}

Sample readSensors() { return {millis(), 0, 0, 0}; }
bool sepConfirm() { return false; }
bool interlocksOK() { return true; }
void pyroFire(int ch, uint32_t ms) {}
void setEnableS2(bool en) {}

// Advika writes these:
/* Flight parameters (OpenRocket): 
    S1 apogee 4596ft (1402m), velocity max 1037ft/s (316m/s), acceleration max 16.2g (159m/s^2).
    S2 apogee 12368ft (3774m), velocity max 865ft/s (264m/s), acceleration max 12.1g (119m/s^2). */

// Liftoff: accel > threshold (stage-aware)
bool liftoff_ok(const Sample& s, int stage) {
  const StageConstraints& c = (stage == 1) ? S1 : S2;
  return s.a_mag > c.liftoff_accel;
}

// Burnout: accel < threshold (stage-aware)
bool burnout_ok(const Sample& s, int stage) {
  const StageConstraints& c = (stage == 1) ? S1 : S2;
  return s.a_mag < c.burnout_accel;
}

// Separation time gate (ms after burnout)
bool sep_time_ok(uint32_t now_ms, uint32_t t_burnout) {
  if (t_burnout == 0) return false;  // No burnout time recorded yet
  uint32_t elapsed = now_ms - t_burnout;
  return elapsed >= 500 && elapsed <= 3000;  // 0.5-3 seconds window (units: milliseconds)
}

// Separation altitude gate (stage-aware, only used for S1)
bool sep_alt_ok(const Sample& s, int stage) {
  const StageConstraints& c = (stage == 1) ? S1 : S2;
  return s.alt > c.sep_alt_min && s.alt < c.sep_alt_max;
}

bool sep_confirm_ok(bool sepC) { return sepC; }

// Apogee: velocity trending to <= ~0 (stage-aware)
bool apogee_ok(const float* v_hist, int n, int stage) {
  if (n < 3) return false;
  
  const StageConstraints& c = (stage == 1) ? S1 : S2;
  int lookback = (n < 10) ? n : 10;
  
  bool had_positive = false;
  for (int i = n - lookback; i < n - 1; i++) {
    if (v_hist[i] > c.apogee_v_threshold) {
      had_positive = true;
      break;
    }
  }
  
  float current_v = v_hist[n - 1];
  return had_positive && current_v <= c.apogee_v_threshold;
}

// Main: below threshold and descending (stage-aware)
bool main_ok(const Sample& s, int stage) {
  const StageConstraints& c = (stage == 1) ? S1 : S2;
  return s.alt < c.main_alt && s.v_alt < c.main_v_descend;
}
