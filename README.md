# Avionics_IREC_2025-2026
Avionics for Booster + Sustainer

Advika: write the physics in `logic.py` (functions return True/False from IMU+baro data).  
Pablo: wire ESP32 in `main.cpp`, call those functions, and hook pins.

Flow: SAFE → ARMED → ASCENT → COAST → S2_ARMED → APOGEE → DROGUE → MAIN → RECOVERY.

ARMED (optional): clean pre-flight gate (arm, zero sensors...). Skip only if we fold these checks into SAFE->ASCENT.
COAST: post-burnout window to run S1/S2 timing, avoids drogue at burnout.

Next:
1) Advika fills `logic.py` (thresholds and math).
2) Pablo keeps `main.cpp` compiling, feeds real samples to those functions, maps outputs to pins.
3) Chan reviews pin map and staging signals.
