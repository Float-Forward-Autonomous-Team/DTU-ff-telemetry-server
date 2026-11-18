"""
Energy strategy for the endurance race: turns the latest telemetry into a recommended
battery power for the pilot.

The strategy itself is the strategy team's code in the Strategy_EnduranceRace repo
(packages `strategy` and `boat`). This file only feeds it the boat's telemetry, the
same way its backend/live_controller.py does, but from HiveMQ messages instead of
InfluxDB queries. Nothing from that repo is copied here, so its updates are used as
soon as they are pulled.

Where the repo is: $STRATEGY_PATH, else ../../Strategy_EnduranceRace next to ff_server.

Controllers (RaceConfig.mode):
  instant  strategy.instant_values.recommend: the energy left above the reserve spread
           over the time left, plus the speed that matches it and an
           ABOVE / ON TARGET / BELOW status for speed and power
  mpc      Seg-MPC: one calibration lap at the average power, then per lap it learns
           the disturbance force on each track segment (ThrustKalman + LapAlphaHistory)
           and re-plans the power per segment with endurance_mpc_solve. The instant
           values are still computed for the speed target and the status.

There is no MQTT or InfluxDB in this file, so the same code runs live
(strategy_service.py) and on a recorded bridge log (strategy_service.py --replay).

Units: power in W, energy in Wh, time in s, speed in m/s, SOC as a fraction 0-1
(the batteries send %).
"""

import json
import math
import os
import re
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_STRATEGY_PATH = BASE_DIR.parent.parent / "Strategy_EnduranceRace"

# Monaco lap split into 4 segments, as in Strategy_EnduranceRace/backend/main.py
MONACO_SEG_LENGTHS = (500.0, 111.0, 500.0, 111.32)


DEFAULT_BOAT_CONFIG = BASE_DIR / "config" / "boat_2026.json"


def load_boat_config(path=DEFAULT_BOAT_CONFIG) -> dict:
    """Boat data (config/boat_2026.json); {} if path is empty."""
    if not path:
        return {}
    with open(path) as f:
        return {k: v for k, v in json.load(f).items() if not k.startswith("_")}


def apply_boat_model(boat: dict):
    """Put mass_kg and eta_grid from the boat config into the strategy repo's physical model,
    so resistance(), efficiency() and the Seg-MPC use them whatever that repo's defaults are."""
    if "mass_kg" not in boat and "eta_grid" not in boat and "surge" not in boat:
        return
    import numpy as np
    import boat.physical_model as pm

    if "eta_grid" in boat:
        eta = np.array(boat["eta_grid"], dtype=float)
        if eta.shape != pm.V_grid.shape:
            raise SystemExit(f"eta_grid needs {len(pm.V_grid)} values (one per V_grid speed)")
        pm.eta_grid = eta
    if "mass_kg" in boat:
        pm.M_BOAT = float(boat["mass_kg"])

    def rebuild():
        (pm.R_grid, pm.P_useful_grid, pm.P_elec_grid,
         pm._resist_interp, pm._power_from_v, pm._v_from_power, pm._eta_interp,
         pm._resist_slope_end) = pm._build_at_mass(pm.M_BOAT)
        pm.P_ELEC_MAX = float(pm.P_elec_grid[-1])

    rebuild()
    if "surge" in boat:
        pm.eta_grid = surge_eta_grid(boat["surge"], pm.V_grid, pm.R_grid, pm.eta_grid)
        rebuild()


def surge_eta_grid(surge: dict, v_grid, r_grid, eta_grid):
    """eta_grid such that the strategy's P_elec(v) = R(v) * v / eta(v) equals the fitted
    P(v) = a * v^b (config/surge_model_2026.json, src/surge_model.py) on the straight. Only at the
    V_grid speeds inside the speeds it was fitted on; the others keep eta_grid. The turn term
    c_turn * r^2 * u is not passed on: the Seg-MPC learns the turns per segment as a disturbance."""
    import numpy as np
    a, b = float(surge["a"]), float(surge["b"])
    lo, hi = surge.get("valid_u_range_mps", (0.0, float("inf")))
    v, eta = np.asarray(v_grid, float), np.array(eta_grid, float)
    inside = (v >= lo - 0.5) & (v <= hi + 0.5)
    eta[inside] = np.clip(np.asarray(r_grid, float)[inside] * v[inside] / (a * v[inside] ** b), 0.05, 0.95)
    return eta


def import_team_strategy(path=None, boat: dict = None):
    """Import the strategy team's `strategy` and `boat` packages; returns them as a dict.
    `boat` (from load_boat_config) overrides their physical model's mass and efficiency."""
    path = Path(path or os.getenv("STRATEGY_PATH") or DEFAULT_STRATEGY_PATH).expanduser()
    if not (path / "strategy" / "seg_mpc.py").exists():
        raise SystemExit(
            f"Strategy_EnduranceRace not found at {path}. "
            "Set STRATEGY_PATH (or --strategy-path) to the folder of that repo.")
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

    apply_boat_model(boat or {})
    from boat.physical_model import efficiency, resistance
    from strategy.instant_values import PRERACE, recommend
    from strategy.kalman import ThrustKalman
    from strategy.seg_mpc import V_CALIB_MIN, LapAlphaHistory, endurance_mpc_solve, ramp

    return dict(efficiency=efficiency, resistance=resistance, PRERACE=PRERACE,
                recommend=recommend, ThrustKalman=ThrustKalman, V_CALIB_MIN=V_CALIB_MIN,
                LapAlphaHistory=LapAlphaHistory, endurance_mpc_solve=endurance_mpc_solve,
                ramp=ramp, path=path)


# ==========================
# Race settings
# ==========================
@dataclass
class RaceConfig:
    start: float              # race start, epoch seconds
    duration_s: float
    battery_wh: float         # energy of all packs together at 100 % SOC
                              # (the strategy uses the lowest pack's SOC)
    soc_reserve: float        # SOC to keep at the finish (fraction)
    p_max_w: float
    mode: str = "instant"     # "instant" | "mpc"
    ramp_w_per_s: float = 800.0   # power slew limit (dp_max_per_s in the strategy repo)
    band_w: float = 500.0     # half-width of the range shown to the pilot
    stale_s: float = 10.0     # an input older than this is not used
    packs: tuple = (1, 2)     # battery/<n> groups that make up the main battery
    # boat model, passed to the strategy code
    eta: float = 0.35         # electrical -> useful power (pre-race planner eta_system)
    mass_kg: float = 315.0
    v_opt_mps: float = None   # pre-race plan; None = PRERACE in instant_values.py
    p_opt_w: float = None
    # Seg-MPC track
    lap_len_m: float = sum(MONACO_SEG_LENGTHS)
    seg_lengths_m: tuple = MONACO_SEG_LENGTHS
    # telemetry
    speed_to_mps: float = 1.0     # gps speed unit -> m/s (1/3.6 if the boat sends km/h)
    use_loadcell: bool = False    # feed thrust/loadcell_n to the thrust Kalman filter
    # "speed" (dv/dt) | "imu" (mean imu ax of the last batch). IMU ax is m/s^2, x forward,
    # gravity included (DBC GnssImu + July race data). Underway the boat sits ~1.6 deg nose
    # up, so ax reads ~0.3 m/s^2 high (~94 N in D_hat at 315 kg), and gps pitch can't
    # correct it (0 +/- 2.7 deg underway, junk at rest). So dv/dt is the default.
    accel_source: str = "speed"
    strategy_path: str = None
    boat: dict = field(default_factory=dict)  # config/boat_2026.json (mass_kg, eta_grid, ...)


# ==========================
# Latest telemetry
# ==========================
class TelemetryState:
    """Latest value of every key of every group, with the time it arrived."""

    def __init__(self):
        self.values = {}  # (group, key) -> (value, t)

    def update(self, group: str, data: dict, t: float):
        if group == "imu/batch":
            # keep only the mean longitudinal acceleration of the batch
            ax = [s.get("ax") for s in data.get("samples") or [] if isinstance(s, dict)]
            ax = [float(a) for a in ax if isinstance(a, (int, float))]
            if ax:
                self.values[("imu", "ax_mean")] = (sum(ax) / len(ax), t)
            return
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
            out = float(value)
        except (TypeError, ValueError):
            return None
        return out if math.isfinite(out) else None


# ==========================
# Controllers
# ==========================
def _dt_obs(dt: float) -> float:
    """Time step as live_controller.py uses it: the real gap if 0-15 s, else 1 s."""
    return dt if 0.0 < dt <= 15.0 else 1.0


class InstantController:
    """strategy.instant_values.recommend on a 30-sample rolling mean of speed and power,
    then the ramp limit (live_controller.py, mode "instant")."""

    name = "instant"
    WINDOW = 30

    def __init__(self, cfg: RaceConfig, team: dict):
        self.cfg, self.team = cfg, team
        self.prerace = dict(team["PRERACE"])
        if cfg.v_opt_mps is not None:
            self.prerace["v_opt_mps"] = cfg.v_opt_mps
        if cfg.p_opt_w is not None:
            self.prerace["p_elec_opt_w"] = cfg.p_opt_w
        self.speeds, self.powers = [], []
        self.p_prev = None

    def step(self, inputs: dict, t_elapsed: float, dt: float):
        cfg = self.cfg
        self.speeds.append(inputs.get("speed", 0.0))
        self.powers.append(inputs.get("p_meas", 0.0))
        del self.speeds[:-self.WINDOW], self.powers[:-self.WINDOW]

        rec = self.team["recommend"](
            v_actual_mps=sum(self.speeds) / len(self.speeds),
            p_actual_elec_w=sum(self.powers) / len(self.powers),
            energy_wh=cfg.battery_wh * inputs["soc"],
            battery_wh=cfg.battery_wh,
            target_soc=cfg.soc_reserve,
            t_remaining_s=max(cfg.duration_s - t_elapsed, 1.0),
            eta=cfg.eta, mass_kg=cfg.mass_kg, p_max_w=cfg.p_max_w,
            prerace=self.prerace,
        )

        if self.p_prev is None:
            # First step, or first after a restart mid-race: ramp from what the boat is
            # drawing now, not from 0.
            self.p_prev = min(max(inputs.get("p_meas", rec["p_elec_rec_w"]), 0.0), cfg.p_max_w)
        if cfg.ramp_w_per_s > 0:
            # dt = 0 on the first step: hold the measured power
            step_dt = _dt_obs(dt) if dt else 0.0
            p_cmd = max(0.0, self.team["ramp"](self.p_prev, rec["p_elec_rec_w"],
                                               cfg.ramp_w_per_s, step_dt))
        else:
            p_cmd = rec["p_elec_rec_w"]
        self.p_prev = p_cmd
        return p_cmd, rec


class SegMpcController:
    """Seg-MPC as in live_controller.py, mode "mpc"."""

    name = "mpc"

    def __init__(self, cfg: RaceConfig, team: dict):
        self.cfg, self.team = cfg, team
        n = len(cfg.seg_lengths_m)
        self.alpha_hist = team["LapAlphaHistory"](n_segs=n, seg_lengths_m=list(cfg.seg_lengths_m))
        self.kalman = team["ThrustKalman"]()
        self.n_segs = n
        self.is_calib = True
        self.laps = 0
        self.lap_targets = None
        self.v_obs_ema = 0.0
        self.x_m = 0.0
        self.p_prev = None
        self.v_prev = None
        self.D_hat = 0.0

    def step(self, inputs: dict, t_elapsed: float, dt: float):
        cfg, team = self.cfg, self.team
        dt = _dt_obs(dt) if self.v_prev is not None else 0.0
        v = inputs.get("speed", 0.0)
        p_meas = inputs.get("p_meas", 0.0)
        energy_wh = cfg.battery_wh * inputs["soc"]
        t_remaining = max(cfg.duration_s - t_elapsed, 1.0)

        if self.lap_targets is None:
            # Calibration lap at the average power that empties the battery to the
            # reserve at the finish (P_ref in live_controller.py, from the energy now).
            e_usable = max(0.0, energy_wh - cfg.battery_wh * cfg.soc_reserve)
            p_ref = min(max(e_usable * 3600.0 / t_remaining, 0.0), cfg.p_max_w)
            self.p_ref = p_ref
            self.lap_targets = [p_ref] * self.n_segs
        if self.p_prev is None:
            self.p_prev = min(max(p_meas, 0.0), cfg.p_max_w)

        self.x_m += v * dt
        seg = self.alpha_hist.seg_id(self.x_m, cfg.lap_len_m)
        target = self.p_ref if self.is_calib else self.lap_targets[seg]
        p_cmd = team["ramp"](self.p_prev, min(target, cfg.p_max_w), cfg.ramp_w_per_s, dt) \
            if cfg.ramp_w_per_s > 0 else min(target, cfg.p_max_w)

        # Disturbance force D = F_T - R(v) - m*a
        if "accel" in inputs:
            a_long = inputs["accel"] - 9.81 * math.sin(math.radians(inputs.get("pitch", 0.0)))
        elif self.v_prev is not None and dt > 0:
            a_long = (v - self.v_prev) / dt
        else:
            a_long = 0.0
        v_pos = max(v, 0.0)
        f_t = self.kalman.update(v, p_meas, float(team["efficiency"](v_pos)),
                                 inputs.get("thrust", 0.0))
        self.D_hat = f_t - float(team["resistance"](v_pos)) - cfg.mass_kg * a_long

        if v >= team["V_CALIB_MIN"]:
            self.alpha_hist.observe(seg, v, p_meas, dt, D_hat=self.D_hat)
        if v >= 0.5:
            self.v_obs_ema = 0.995 * self.v_obs_ema + 0.005 * v if self.v_obs_ema > 0 else v

        new_laps = int(self.x_m / max(cfg.lap_len_m, 1.0))
        if new_laps > self.laps:
            self.alpha_hist.end_lap()
            self.laps = new_laps
            targets, _, _ = team["endurance_mpc_solve"](
                E_usable=max(0.0, energy_wh - cfg.battery_wh * cfg.soc_reserve),
                T_remaining=t_remaining, P_max=cfg.p_max_w,
                d_segs=self.alpha_hist.current_D_estimate(),
                n_segs=self.n_segs, lap_length_m=cfg.lap_len_m,
                v_obs=self.v_obs_ema, seg_lengths_m=list(cfg.seg_lengths_m),
            )
            self.lap_targets = list(targets)
            self.is_calib = False

        self.p_prev = p_cmd
        self.v_prev = v
        return p_cmd, {
            "seg": seg, "laps": self.laps, "is_calib": int(self.is_calib),
            "D_hat_n": round(self.D_hat, 2), "x_m": round(self.x_m, 1),
            **{f"lap_target_{i}_w": round(p, 1) for i, p in enumerate(self.lap_targets)},
        }


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
    info: dict = field(default_factory=dict)  # speed target, status, Seg-MPC state
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
    def __init__(self, cfg: RaceConfig, state: TelemetryState, team: dict = None):
        if cfg.mode not in ("instant", "mpc"):
            raise ValueError(f"unknown mode {cfg.mode!r}")
        self.cfg = cfg
        self.state = state
        self.team = team or import_team_strategy(cfg.strategy_path, cfg.boat)
        self.instant = InstantController(cfg, self.team)
        self.mpc = SegMpcController(cfg, self.team) if cfg.mode == "mpc" else None
        self.t_last = None

    def inputs(self, now: float) -> dict:
        """Fresh inputs only: a value missing or older than stale_s is left out."""
        cfg, s = self.cfg, self.state
        get = lambda g, k: s.get(g, k, now, cfg.stale_s)
        out = {}

        socs = [get(f"battery/{n}", "soc") for n in cfg.packs]
        if all(v is not None and 0 <= v <= 100 for v in socs):
            # The motor needs every pack, so the race ends when the lowest one is empty
            # (2026 endurance: back 6 min early at 1.8 % and 11 %). Plan on the lowest pack.
            out["soc"] = min(socs) / 100.0
            out["soc_mean"] = sum(socs) / len(socs) / 100.0

        powers = [get(f"battery/{n}", "power") for n in cfg.packs]
        if all(v is not None for v in powers):
            out["p_batt"] = sum(powers)

        speed = get("gps", "speed")
        if speed is not None:
            out["speed"] = max(0.0, speed * cfg.speed_to_mps)

        # measured power for the strategy: motor power, as live_controller.py uses
        p_motor = get("motor", "power")
        if p_motor is not None:
            out["p_motor"] = p_motor
        p_meas = p_motor if p_motor is not None else out.get("p_batt")
        if p_meas is not None:
            out["p_meas"] = max(0.0, p_meas)

        pitch = get("gps", "pitch")
        if pitch is not None:
            out["pitch"] = pitch
        if cfg.accel_source == "imu":
            ax = get("imu", "ax_mean")
            if ax is not None:
                out["accel"] = ax
        if cfg.use_loadcell:
            thrust = get("thrust", "loadcell_n")
            if thrust is not None:
                out["thrust"] = max(0.0, thrust)

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

        p, iv = self.instant.step(inputs, t_elapsed, dt)
        rec.controller = self.instant.name
        rec.info = {
            "v_rec_mps": iv["v_rec_mps"], "p_instant_w": iv["p_elec_rec_w"],
            "p_needed_w": iv["p_needed_elec_w"], "e_usable_wh": iv["e_usable_wh"],
            "speed_status": iv["speed_status"], "power_status": iv["power_status"],
        }
        if self.mpc is not None:
            p, mpc_info = self.mpc.step(inputs, t_elapsed, dt)
            rec.controller = self.mpc.name
            rec.info.update(mpc_info)

        p = min(max(p, 0.0), self.cfg.p_max_w)
        rec.p_w = p
        rec.p_min_w = max(0.0, p - self.cfg.band_w)
        rec.p_max_w = min(self.cfg.p_max_w, p + self.cfg.band_w)
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
