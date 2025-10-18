#include <Arduino.h>

struct Sample { uint32_t t_ms; float alt, v_alt, a_mag; };
enum State { SAFE, ARMED, ASCENT, COAST, S2_ARMED, APOGEE, DROGUE, MAIN, RECOVERY };

bool liftoff_ok(const Sample& s);
bool burnout_ok(const Sample& s);
bool sep_time_ok(uint32_t now_ms, uint32_t t_burnout);
bool sep_alt_ok(const Sample& s);
bool sep_confirm_ok(bool sepC);
bool apogee_ok(const float* v_hist, int n);
bool main_ok(const Sample& s);
bool vote2of3(bool a,bool b,bool c);

Sample readSensors(){ return {millis(),0,0,0}; }
bool sepConfirm(){ return false; }
bool interlocksOK(){ return true; }
void pyroFire(int ch, uint32_t ms){}
void setEnableS2(bool en){}

State ST = SAFE;
uint32_t t_burnout = 0;
float v_hist[256]; int vh_n=0;

void setup(){ Serial.begin(115200); }

void loop(){
  static uint32_t last=0; if (millis()-last<20) return; last=millis();
  Sample s = readSensors();
  bool sepC = sepConfirm();
  bool armed = interlocksOK();
  if (vh_n<256) v_hist[vh_n++]=s.v_alt; else { memmove(v_hist, v_hist+1, sizeof(v_hist)-sizeof(float)); v_hist[255]=s.v_alt; }

  switch(ST){
    case SAFE:   if (armed) ST=ARMED; break;
    case ARMED:  if (liftoff_ok(s)) ST=ASCENT; break;
    case ASCENT: if (burnout_ok(s)) { t_burnout=s.t_ms; ST=COAST; } break;
    case COAST: {
      bool timeOK = sep_time_ok(s.t_ms, t_burnout);
      bool altOK  = sep_alt_ok(s);
      bool confOK = sep_confirm_ok(sepC);
      if (vote2of3(timeOK, altOK, confOK)) { setEnableS2(true); ST=S2_ARMED; }
      if (apogee_ok(v_hist, vh_n)) ST=APOGEE;
    } break;
    case APOGEE:   pyroFire(1, 60); ST=DROGUE; break;
    case DROGUE:   if (main_ok(s)) { pyroFire(2, 80); ST=MAIN; } break;
    case MAIN:     if (s.alt < 50) ST=RECOVERY; break;
    default: break;
  }
}
