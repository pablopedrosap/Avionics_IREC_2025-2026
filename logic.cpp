#pragma once
#include <Arduino.h>

struct Sample { uint32_t t_ms; float alt, v_alt, a_mag; };
enum State { SAFE, ARMED, ASCENT, COAST, S2_ARMED, APOGEE, DROGUE, MAIN, RECOVERY };

Sample readSensors() { return {millis(), 0, 0, 0}; }
bool sepConfirm() { return false; }
bool interlocksOK() { return true; }
void pyroFire(int ch, uint32_t ms) {}
void setEnableS2(bool en) {}
inline bool vote2of3(bool a, bool b, bool c) { return (a + b + c) >= 2; }

// Advika writes these:

####
bool liftoff_ok(const Sample& s) { return false; }
bool burnout_ok(const Sample& s) { return false; }
bool sep_time_ok(uint32_t now_ms, uint32_t t_burnout) { return false; }
bool sep_alt_ok(const Sample& s) { return false; }
bool sep_confirm_ok(bool sepC) { return sepC; }
bool apogee_ok(const float* v_hist, int n) { return false; }
bool main_ok(const Sample& s) { return false; }
