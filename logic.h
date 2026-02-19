#pragma once

#include <Arduino.h>

// Units: t_ms = time in ms, alt = altitude in m, v_alt = vertical velocity in m/s, a_mag = acceleration magnitude in m/s^2
struct Sample {
  uint32_t t_ms;
  float alt;
  float v_alt;
  float a_mag;
};

// High-level flight states.
enum State { SAFE, ARMED, ASCENT, COAST, S2_ARMED, APOGEE, DROGUE, MAIN, RECOVERY };

// Stage-specific constraints
struct StageConstraints {
  float liftoff_accel;       // m/s^2
  float burnout_accel;       // m/s^2
  float sep_alt_min;         // m
  float sep_alt_max;         // m
  float apogee_v_threshold;  // m/s
  float main_alt;            // m
  float main_v_descend;      // m/s
};

extern const StageConstraints S1;
extern const StageConstraints S2;

// "Hardware / integration" hooks (currently stubs in logic.cpp; intended to be wired to real sensors/pyros).
Sample readSensors();
bool sepConfirm();
bool interlocksOK();
void pyroFire(int ch, uint32_t ms);
void setEnableS2(bool en);

// Stage-aware logic gates
bool liftoff_ok(const Sample& s, int stage);
bool burnout_ok(const Sample& s, int stage);
bool sep_time_ok(uint32_t now_ms, uint32_t t_burnout);
bool sep_alt_ok(const Sample& s, int stage);
bool sep_confirm_ok(bool sepC);
bool apogee_ok(const float* v_hist, int n, int stage);
bool main_ok(const Sample& s, int stage);

