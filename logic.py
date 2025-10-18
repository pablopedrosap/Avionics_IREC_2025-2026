# decisions.py
from dataclasses import dataclass

@dataclass
class Sample:
    t_ms:int; alt:float; v_alt:float; ax:float

def vote2of3(a,b,c): return (a+b+c) >= 2
def in_time_window(now, t0, open_ms, close_ms):
    return (now - t0) >= open_ms and (now - t0) <= close_ms

def burnout(win_ax):  # win_ax = media móvil de |ax|
    TH = 0.5  # g (ajustable)
    return win_ax < TH

def apogee(v_alt_hist):
    # True si v_alt cruza + a − y |v| < eps durante Y ms
    EPS, Y = 1.0, 150
    return (v_alt_hist.crosses_zero_pos_to_neg()
            and v_alt_hist.stays_within(EPS, Y))


# repo logic_py/ con state_machine.py, decisions.py, tests/ (pytest verde) y
#  un replay.py que lee un CSV t,alt,acc y muestra en qué instantes se disparan eventos.
