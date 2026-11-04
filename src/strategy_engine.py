"""
Energy strategy for the endurance race: turns the latest telemetry into a recommended
battery power for the pilot.

There is no MQTT or InfluxDB in this file, so the same code runs live
(strategy_service.py) and on a recorded bridge log (strategy_service.py --replay).

Controllers, tried in order; the first one that gives a value is used:
  SimpleController  the energy-budget strategy used in the 2026 endurance race
                    (LV Final Report, section 3.4.2)
  (MPC)             not here yet: it goes in front of SimpleController in
                    StrategyEngine.controllers, with the same step() call, and returns
                    None whenever it can't give a trustworthy answer.

Units: power in W, energy in Wh, time in s, SOC as a fraction 0-1 (the batteries send %).
"""

import json
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone


# ==========================
# Race settings
# ==========================
@dataclass
class RaceConfig:
    start: float              # race start, epoch seconds
    duration_s: float
    battery_wh: float         # energy of all packs together at 100 % SOC
    soc_reserve: float        # SOC to keep at the finish (fraction)
    p_max_w: float
    ramp_w_per_s: float = 0.0  # 0 = no ramp limit
    band_w: float = 500.0     # half-width of the range shown to the pilot
    stale_s: float = 10.0     # an input older than this is not used
    packs: tuple = (1, 2)     # battery/<n> groups that make up the main battery


# ==========================
# Latest telemetry
# ==========================
class TelemetryState:
    """Latest value of every key of every group, with the time it arrived."""

    def __init__(self):
        self.values = {}  # (group, key) -> (value, t)

    def update(self, group: str, data: dict, t: float):
        for key, value in data.items():
            if key != "ts":
                self.values[(group, key)] = (value, t)

    def get(self, group: str, key: str, now: float, max_age: float):
        """The value as a float, or None if it is missing, too old or not a number."""
        item = self.values.get((group, key))
        if item is None:
            return None
        value, t = item
        if now - t > max_age:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None


# ==========================
# Controllers
# ==========================
class SimpleController:
    """Spread the energy left above the reserve evenly over the time left:

        E_usable = max(0, E - E_battery * SOC_reserve)
        P_des    = min(3600 * E_usable / (T_race - t), P_max)
        P_k      = P_k-1 + clip(P_des - P_k-1, -ramp * dt, +ramp * dt)
    """

    name = "simple"

    def __init__(self, cfg: RaceConfig):
        self.cfg = cfg
        self.p_prev = None

    def desired(self, soc: float, t_elapsed: float) -> float:
        cfg = self.cfg
        e_usable = max(0.0, (soc - cfg.soc_reserve) * cfg.battery_wh)
        t_left = cfg.duration_s - t_elapsed
        if t_left <= 0:
            return 0.0
        return min(3600.0 * e_usable / t_left, cfg.p_max_w)

    def step(self, inputs: dict, t_elapsed: float, dt: float):
        soc = inputs.get("soc")
        if soc is None:
            return None

        p_des = self.desired(soc, t_elapsed)

        if self.p_prev is None:
            # First step, or first after a restart mid-race: ramp from what the boat
            # is drawing now, not from 0.
            p_batt = inputs.get("p_batt")
            self.p_prev = p_des if p_batt is None else min(max(p_batt, 0.0), self.cfg.p_max_w)

        if self.cfg.ramp_w_per_s > 0:
            limit = self.cfg.ramp_w_per_s * dt
            p_cmd = self.p_prev + min(max(p_des - self.p_prev, -limit), limit)
        else:
            p_cmd = p_des

        self.p_prev = p_cmd
        return p_cmd


# ==========================
# Engine
# ==========================
@dataclass
class Recommendation:
    t: float
    controller: str           # name of the controller used, or "none"
    p_w: float = None         # recommended battery power
    p_min_w: float = None
    p_max_w: float = None
    reason: str = ""          # why there is no recommendation
    inputs: dict = field(default_factory=dict)
    t_left_s: float = None

    @property
    def ok(self) -> bool:
        return self.p_w is not None

    def payload(self) -> dict:
        """The JSON the downlink already uses on boat/message/power (kW)."""
        return {
            "powerMin": round(self.p_min_w / 1000, 2),
            "powerMax": round(self.p_max_w / 1000, 2),
        }


class StrategyEngine:
    def __init__(self, cfg: RaceConfig, state: TelemetryState):
        self.cfg = cfg
        self.state = state
        self.controllers = [SimpleController(cfg)]
        self.t_last = None

    def inputs(self, now: float) -> dict:
        """Fresh inputs only: a value missing or older than stale_s is left out."""
        cfg, s = self.cfg, self.state
        out = {}

        socs = [s.get(f"battery/{n}", "soc", now, cfg.stale_s) for n in cfg.packs]
        if all(v is not None and 0 <= v <= 100 for v in socs):
            # equal packs assumed, so the energy-weighted SOC is the mean
            out["soc"] = sum(socs) / len(socs) / 100.0

        powers = [s.get(f"battery/{n}", "power", now, cfg.stale_s) for n in cfg.packs]
        if all(v is not None for v in powers):
            out["p_batt"] = sum(powers)

        for name, group, key in (("speed", "gps", "speed"), ("p_motor", "motor", "power")):
            value = s.get(group, key, now, cfg.stale_s)
            if value is not None:
                out[name] = value

        return out

    def step(self, now: float) -> Recommendation:
        dt = 0.0 if self.t_last is None else now - self.t_last
        self.t_last = now

        t_elapsed = now - self.cfg.start
        inputs = self.inputs(now)
        rec = Recommendation(t=now, controller="none", inputs=inputs,
                             t_left_s=self.cfg.duration_s - t_elapsed)

        if t_elapsed < 0:
            rec.reason = "race not started"
            return rec
        if t_elapsed >= self.cfg.duration_s:
            rec.reason = "race finished"
            return rec
        if "soc" not in inputs:
            rec.reason = "SOC missing or stale"
            return rec

        for controller in self.controllers:
            p = controller.step(inputs, t_elapsed, dt)
            if p is not None:
                p = min(max(p, 0.0), self.cfg.p_max_w)
                rec.controller = controller.name
                rec.p_w = p
                rec.p_min_w = max(0.0, p - self.cfg.band_w)
                rec.p_max_w = min(self.cfg.p_max_w, p + self.cfg.band_w)
                return rec

        rec.reason = "no controller gave a value"
        return rec


# ==========================
# Bridge log replay
# ==========================
_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_KV = re.compile(r"(\w+)=(\S+)")


def parse_log_line(line: str, day: str = "2000-01-01"):
    """One line of mqtt_subscriber_real_test.py output -> (t, group, data), or None.

    Two formats exist:
      JSON groups (timestamps-code):  15:20:49  battery/1   voltage=49.5 ... soc=19.6 ts=2026-07-10T13:20:48.765Z
      One topic per field (main):     15:39:36.024  battery/1/soc   83.6
    The first uses the boat's ts; the second only has the bridge's local time of day,
    placed on <day>.
    """
    parts = _ANSI.sub("", line).split()
    if len(parts) < 3 or not re.fullmatch(r"\d{2}:\d{2}:\d{2}(\.\d+)?", parts[0]):
        return None

    if "=" in line:
        data = dict(_KV.findall(_ANSI.sub("", line)))
        ts = data.get("ts")
        if not ts:
            return None
        try:
            t = datetime.fromisoformat(ts.replace("Z", "+00:00")).timestamp()
        except ValueError:
            return None
        return t, parts[1], data

    if len(parts) != 3 or "/" not in parts[1]:
        return None
    group, key = parts[1].rsplit("/", 1)
    t = datetime.fromisoformat(f"{day}T{parts[0]}").replace(tzinfo=timezone.utc).timestamp()
    return t, group, {key: parts[2]}


def load_log(path: str, day: str = "2000-01-01"):
    """All telemetry records of a bridge log, sorted by time."""
    records = []
    with open(path, errors="replace") as f:
        for line in f:
            rec = parse_log_line(line, day)
            if rec:
                records.append(rec)
    records.sort(key=lambda r: r[0])
    return records


def payload_text(rec: Recommendation) -> str:
    return json.dumps(rec.payload())
