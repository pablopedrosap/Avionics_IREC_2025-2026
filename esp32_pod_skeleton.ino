// ---- HAL ----
struct Sample { float ax, ay, az, alt, vAlt; uint32_t tMs; };
bool readIMU(Sample& s);
bool readBaro(Sample& s);
void pyroFire(int ch, uint32_t ms);
bool pyroContOK(int ch);
bool sepConfirm(); 
void armLED(bool on);

// ---- State machine ----
enum State { SAFE, ARMED, ASCENT, COAST, SEP_DONE, S2_ARMED, APOGEE, DROGUE_DEP, MAIN_DEP, RECOVERY };
State st = SAFE;

// ---- Detectors ----
bool burnout(const Sample& s);
bool apogeeReached(const Sample& s);
bool inTimeWindow(uint32_t now, uint32_t t0, uint32_t open, uint32_t close);
bool aboveAlt(const Sample& s, float h);

// ---- Voter (sustainer) ----
inline bool vote2of3(bool a, bool b, bool c){ return (a+b+c) >= 2; }

void loopTick(){
  Sample s; readIMU(s); readBaro(s);
  switch(st){
    case SAFE:   /* if armed + checks */ st = ARMED; break;
    case ARMED:  if(/*liftoff*/ ) st = ASCENT; break;
    case ASCENT: if(burnout(s)) st = COAST; break;
    case COAST:
      // Booster channel logic uses its own 2-of-3 cues + interlocks to decide S1_SEP
      // Sustainer uses vote2of3(cueTime, cueAlt/vel, sepConfirm()) to arm S2_IGNEN
      break;
    case APOGEE: /* fire DROGUE */ break;
    ...
    default: break;
  }
}
