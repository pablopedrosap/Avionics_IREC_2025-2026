from dataclasses import dataclass

SAFE, ARMED, ASCENT, COAST, S2_ARMED, APOGEE, DROGUE, MAIN, RECOVERY = range(9)

@dataclass
class Sample:
    t_ms: int
    alt: float
    v_alt: float
    a_mag: float

def liftoff_ok(s: Sample) -> bool: raise NotImplementedError
def burnout_ok(s: Sample) -> bool: raise NotImplementedError
def sep_time_ok(now_ms: int, t_burnout: int) -> bool: raise NotImplementedError
def sep_alt_ok(s: Sample) -> bool: raise NotImplementedError
def sep_confirm_ok(sep_confirm_signal: bool) -> bool: raise NotImplementedError
def apogee_ok(v_hist: list[float]) -> bool: raise NotImplementedError
def main_ok(s: Sample) -> bool: raise NotImplementedError
def vote2of3(a: bool, b: bool, c: bool) -> bool: raise NotImplementedError
