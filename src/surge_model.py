"""Data-driven power model for the boat: P = k * u^3 + c_turn * r^2 * u.

After "Data-driven parametrization of an energy-propulsive model for an electric racing vessel"
(unpublished draft, Oct 2026), fitted on our own race telemetry (Database2026/race_data CSVs).

  P       motor electrical power [W] (motor_power, what the strategy service controls)
  u       GPS speed [m/s]
  r       yaw rate [rad/s] (from gps_heading, or from the GPS track when there is no heading)
  k       how hard the boat is to push in a straight line: hull drag and drivetrain efficiency
          together, k = rho*Cd*A / (2*eta) [W s^3 / m^3]
  c_turn  extra power in turns: the propeller also pushes sideways, F = m*u*r [kg m]

Steps (as in the paper):
  1. Clean (physical limits + Hampel filter) and resample every signal onto one 1 s grid,
     leaving gaps longer than 3 samples empty.
  2. Keep "propulsion" rows (P > 200 W and u > 1 m/s) and cut them into power plateaus:
     >= 30 s with the power inside a narrow band. Speed is the response, so we cut on power.
  3. Fit k and c_turn by least squares on the plateau means (no intercept).
  4. Battery, per pack: V = OCV(SOC) - R0 * I, OCV a cubic in SOC; Q_eff by coulomb counting.
  5. Blind check: fit on one day, predict another day's power from its speed and yaw rate,
     and its speed from its power (forward replay of m_eff du/dt = eta (P - P_model) / u).
     The same is done for the model the strategy uses now (ShipFlow resistance / eta_grid) and
     for the P = 88 v^2.29 curve, so the numbers can be compared.

  python src/surge_model.py                     # fit on the endurance race, test on the laps test
  python src/surge_model.py --fit endurance --test lapstest --out config/surge_model_2026.json

Needs numpy (and the Strategy_EnduranceRace repo for the comparison with the current model).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

BASE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_DATA_DIR = BASE_DIR.parent / "Database2026" / "race_data"
DEFAULT_STRATEGY_PATH = BASE_DIR.parent.parent / "Strategy_EnduranceRace"

DAYS = {
    "lapstest": "telemetry_08_07_2026_lapstest.csv",
    "speedrecord": "telemetry_09_07_2026_speedrecord.csv",
    "endurance": "telemetry_10_07_2026_endurance_race.csv",
    "slalom": "telemetry_11_07_2026_slalom_race.csv",
}

DT = 1.0                  # resampling grid [s]; the motor reports every ~1 s, GPS every 1-2 s
MAX_GAP_S = 12.0          # interpolate over gaps up to this (paper: 3 samples of 3.44 s); the
                          # 08/07 export loses ~10 s every 3-4 samples
P_MIN_W, U_MIN = 200.0, 1.0   # "propulsion": both, or a moored power spike counts
PLATEAU_MIN_S = 30.0
PLATEAU_TOL_W = 300.0     # power band of a plateau (max - min); the paper's 50 W is for ~3.4 s data
PLATEAU_TOL_REL = 0.10    # ... or 10 % of the power, whichever is larger
PLATEAU_SMOOTH_S = 5      # the band is checked on a 5 s moving average of the power
EXPONENTS = np.round(np.arange(1.0, 3.51, 0.01), 2)  # searched for the free-exponent law
STRAIGHT_R = 0.03         # |r| below this [rad/s] counts as straight (paper)
RHO = 1025.0
G = 9.81

# physical limits, before the Hampel filter
LIMITS = {
    "motor_power": (0.0, 30000.0),
    "gps_speed": (0.0, 15.0),
    "battery_1_voltage": (30.0, 70.0), "battery_2_voltage": (30.0, 70.0),
    "battery_1_current": (-400.0, 400.0), "battery_2_current": (-400.0, 400.0),
    "battery_1_soc": (0.0, 100.0), "battery_2_soc": (0.0, 100.0),
    "rotary_angle_deg": (-60.0, 60.0),
}
SIGNALS = ["motor_power", "gps_speed", "gps_heading", "gps_latitude", "gps_longitude",
           "rotary_angle_deg",
           "battery_1_voltage", "battery_1_current", "battery_1_soc",
           "battery_2_voltage", "battery_2_current", "battery_2_soc"]


# ============================================================ loading and cleaning

def _parse_time(s: str) -> float:
    return datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp()


def load_csv(path: Path) -> dict:
    """{signal: (t[s], values)} for every SIGNALS column present, only rows that have it."""
    raw = {s: ([], []) for s in SIGNALS}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t = None
            for s in SIGNALS:
                v = row.get(s)
                if v in (None, ""):
                    continue
                try:
                    x = float(v)
                except ValueError:
                    continue
                if t is None:
                    t = _parse_time(row["timestamp"])
                raw[s][0].append(t)
                raw[s][1].append(x)
    return {s: (np.array(t), np.array(v)) for s, (t, v) in raw.items() if len(t) > 1}


def hampel(x: np.ndarray, half: int = 3, n_sigma: float = 3.0) -> np.ndarray:
    """Replace points further than n_sigma robust sigmas from the moving median by NaN."""
    y = x.astype(float).copy()
    n = len(x)
    for i in range(n):
        w = x[max(0, i - half): i + half + 1]
        w = w[np.isfinite(w)]
        if len(w) < 3:
            continue
        med = np.median(w)
        mad = 1.4826 * np.median(np.abs(w - med))
        if mad > 0 and abs(x[i] - med) > n_sigma * mad:
            y[i] = np.nan
    return y


def _resample(t, v, grid, max_gap):
    """Linear interpolation onto grid; NaN where the nearest samples are more than max_gap apart."""
    ok = np.isfinite(v)
    t, v = t[ok], v[ok]
    if len(t) < 2:
        return np.full(len(grid), np.nan)
    order = np.argsort(t)
    t, v = t[order], v[order]
    out = np.interp(grid, t, v, left=np.nan, right=np.nan)
    idx = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
    out[(t[idx] - t[idx - 1]) > max_gap] = np.nan
    return out


def _course_from_track(t, lat, lon):
    """Course over ground [rad] at the midpoints of consecutive GPS fixes."""
    y = np.diff(lat) * math.pi / 180 * 6371000.0
    x = np.diff(lon) * math.pi / 180 * 6371000.0 * np.cos(np.radians(lat[:-1]))
    moved = np.hypot(x, y) > 0.5  # standing still: no course
    return (t[:-1] + t[1:])[moved] / 2, np.arctan2(x, y)[moved]


def _yaw_rate(t_course, course, grid, max_gap):
    """Yaw rate [rad/s] on the grid from a heading series [rad]: unwrap, then derivative."""
    if len(t_course) < 3:
        return np.full(len(grid), np.nan)
    order = np.argsort(t_course)
    tc, c = t_course[order], np.unwrap(course[order])
    psi = _resample(tc, c, grid, max_gap)
    r = np.gradient(psi, DT)
    # 3-point moving average: the GPS course is noisy at 1-2 s
    k = np.ones(3) / 3
    return np.convolve(np.nan_to_num(r, nan=0.0), k, mode="same") * np.where(np.isfinite(r), 1, np.nan)


def prepare(path: Path) -> dict:
    """Load, clean and resample one day onto a DT grid. Returns arrays keyed by name."""
    raw = load_csv(path)
    if "motor_power" not in raw or "gps_speed" not in raw:
        raise SystemExit(f"{path.name}: needs motor_power and gps_speed")
    t0 = min(v[0].min() for v in raw.values())
    t1 = max(v[0].max() for v in raw.values())
    grid = np.arange(t0, t1, DT)
    day = {"t": grid, "name": path.stem}

    for s, (t, v) in raw.items():
        if s in ("gps_heading", "gps_latitude", "gps_longitude"):
            continue
        lo, hi = LIMITS.get(s, (-np.inf, np.inf))
        v = np.where((v >= lo) & (v <= hi), v, np.nan)
        if s in ("motor_power", "gps_speed"):
            v = hampel(v)
        native = float(np.median(np.diff(t)))
        day[s] = _resample(t, v, grid, max_gap=max(3 * native, MAX_GAP_S))

    # yaw rate: from the heading if logged, else from the track (the endurance day has no heading)
    if "gps_heading" in raw:
        t, h = raw["gps_heading"]
        tc, course = t, np.radians(h)
        day["yaw_source"] = "gps_heading"
    else:
        t, lat = raw["gps_latitude"]
        _, lon = raw["gps_longitude"]
        tc, course = _course_from_track(t, lat, lon)
        day["yaw_source"] = "gps track"
    native = float(np.median(np.diff(np.sort(tc))))
    day["r"] = _yaw_rate(tc, course, grid, max_gap=max(3 * native, MAX_GAP_S))
    return day


# ============================================================ segmentation

def propulsion(day) -> np.ndarray:
    P, u, r = day["motor_power"], day["gps_speed"], day["r"]
    return np.isfinite(P) & np.isfinite(u) & np.isfinite(r) & (P > P_MIN_W) & (u > U_MIN)


def _moving_average(x, n):
    y = np.convolve(np.nan_to_num(x), np.ones(n) / n, mode="same")
    return np.where(np.isfinite(x), y, np.nan)


def plateaus(day, min_s=PLATEAU_MIN_S) -> list:
    """Maximal runs of propulsion rows whose power stays in a band of
    max(PLATEAU_TOL_W, PLATEAU_TOL_REL * P) for at least min_s. List of (i0, i1) (i1 exclusive).
    The band is checked on a PLATEAU_SMOOTH_S moving average: our motor power comes in ~110 W
    steps and wobbles +-200 W at a steady throttle (the paper's 3.44 s samples average that out)."""
    P, ok = _moving_average(day["motor_power"], PLATEAU_SMOOTH_S), propulsion(day)
    n, out, i = len(P), [], 0
    min_n = int(round(min_s / DT))
    while i < n:
        if not ok[i]:
            i += 1
            continue
        lo = hi = P[i]
        j = i + 1
        while j < n and ok[j]:
            lo2, hi2 = min(lo, P[j]), max(hi, P[j])
            if hi2 - lo2 > max(PLATEAU_TOL_W, PLATEAU_TOL_REL * hi2):
                break
            lo, hi, j = lo2, hi2, j + 1
        if j - i >= min_n:
            out.append((i, j))
            i = j
        else:
            i += 1
    return out


def plateau_table(day, segs) -> dict:
    """Per plateau: mean P, mean u, straight fraction, duration, and its rows (u, r) so the
    regressors mean(u^b) and mean(r^2 u) can be built for any exponent b."""
    P, u, r = day["motor_power"], day["gps_speed"], day["r"]
    tab = {"P": [], "u": [], "straight": [], "dur": [], "rows": []}
    for i0, i1 in segs:
        tab["P"].append(np.mean(P[i0:i1]))
        tab["u"].append(np.mean(u[i0:i1]))
        tab["straight"].append(np.mean(np.abs(r[i0:i1]) < STRAIGHT_R))
        tab["dur"].append((i1 - i0) * DT)
        tab["rows"].append((u[i0:i1], r[i0:i1]))
    rows = tab.pop("rows")
    out = {k: np.array(v) for k, v in tab.items()}
    out["rows"] = rows
    return out


def merge_tables(tabs) -> dict:
    out = {k: np.concatenate([t[k] for t in tabs]) for k in ("P", "u", "straight", "dur")}
    out["rows"] = [row for t in tabs for row in t["rows"]]
    return out


# ============================================================ fits

def _fit_fixed_b(tab, b, w):
    """P = a u^b + c_turn r^2 u on plateau means, weighted least squares through the origin."""
    X = np.array([[np.mean(u ** b), np.mean(r ** 2 * u)] for u, r in tab["rows"]])
    sw = np.sqrt(w)
    (a, c), *_ = np.linalg.lstsq(X * sw[:, None], tab["P"] * sw, rcond=None)
    res = tab["P"] - X @ np.array([a, c])
    ss_res = float(np.sum(w * res ** 2))
    dof = max(len(w) - 2, 1)
    cov = np.linalg.inv((X * w[:, None]).T @ X) * ss_res / dof
    return float(a), float(c), ss_res, np.sqrt(np.maximum(np.diag(cov), 0))


def fit_power_law(tab, weights=None) -> dict:
    """Two laws on the plateau means, weighted by plateau duration:
      cubic (the paper): P = k u^3 + c_turn r^2 u
      free exponent:     P = a u^b + c_turn r^2 u, b searched over EXPONENTS
    Plus k on straight plateaus only (no turn term), as a check on c_turn."""
    w = tab["dur"] if weights is None else weights
    ss_tot = float(np.sum(w * (tab["P"] - np.average(tab["P"], weights=w)) ** 2))
    out = {"n_plateaus": int(len(w)), "plateau_s": float(np.sum(tab["dur"])),
           "u_range_mps": [float(tab["u"].min()), float(tab["u"].max())]}

    k, c, ss, se = _fit_fixed_b(tab, 3.0, w)
    out["cubic"] = {"k": k, "c_turn": c, "k_se": float(se[0]), "c_turn_se": float(se[1]),
                    "r2": 1 - ss / ss_tot}

    best = min((_fit_fixed_b(tab, b, w) + (b,) for b in EXPONENTS), key=lambda f: f[2])
    a, c, ss, se, b = best
    out["free"] = {"a": a, "b": float(b), "c_turn": c, "a_se": float(se[0]), "c_turn_se": float(se[1]),
                   "r2": 1 - ss / ss_tot}

    st = tab["straight"] >= 0.8
    if st.sum() >= 3:
        u3 = np.array([np.mean(u ** 3) for u, _ in tab["rows"]])[st]
        out["cubic"]["k_straight_only"] = float(np.sum(w[st] * tab["P"][st] * u3) / np.sum(w[st] * u3 ** 2))
        out["n_straight_plateaus"] = int(st.sum())
    return out


def fit_battery(days) -> dict:
    """Per pack: V = OCV(SOC) - R0 * I_discharge, OCV = cubic in SOC (0..1), pooled over days;
    Q_eff = Ah taken out / SOC drop, on the longest day. The BMS reports discharge as negative."""
    out = {}
    for pack in (1, 2):
        V, I, S = [], [], []
        for d in days:
            v, i, s = (d.get(f"battery_{pack}_{x}") for x in ("voltage", "current", "soc"))
            if v is None or i is None or s is None:
                continue
            ok = np.isfinite(v) & np.isfinite(i) & np.isfinite(s)
            V.append(v[ok]); I.append(-i[ok]); S.append(s[ok] / 100)
        if not V:
            continue
        V, I, S = np.concatenate(V), np.concatenate(I), np.concatenate(S)
        X = np.column_stack([np.ones_like(S), S, S ** 2, S ** 3, -I])
        coef, *_ = np.linalg.lstsq(X, V, rcond=None)
        rmse = float(np.sqrt(np.mean((X @ coef - V) ** 2)))
        res = {"R0_ohm": float(coef[4]), "ocv_poly": [float(x) for x in coef[:4]],
               "ocv_poly_note": "OCV = c0 + c1*SOC + c2*SOC^2 + c3*SOC^3, SOC 0..1",
               "rmse_v": rmse, "n": int(len(V)), "max_current_a": float(I.max())}
        # coulomb counting on the day with the biggest SOC drop
        best = None
        for d in days:
            i, s = d.get(f"battery_{pack}_current"), d.get(f"battery_{pack}_soc")
            if i is None or s is None:
                continue
            ok = np.isfinite(i) & np.isfinite(s)
            if ok.sum() < 10:
                continue
            drop = (np.nanmax(s[ok][:60]) - np.nanmin(s[ok][-60:])) / 100
            ah = np.sum(-i[ok]) * DT / 3600
            if drop > 0.2 and (best is None or drop > best[0]):
                best = (drop, ah, d["name"])
        if best:
            res["Q_eff_ah"] = float(best[1] / best[0])
            res["Q_eff_from"] = f"{best[2]}: {best[1]:.1f} Ah for a {best[0] * 100:.1f} % SOC drop"
        out[f"pack_{pack}"] = res
    return out


# ============================================================ models and validation

def p_surge(u, r, a, b, c_turn):
    """P = a u^b + c_turn r^2 u (the paper's law is b = 3, a = k)."""
    return a * u ** b + c_turn * r ** 2 * u


def law_params(law: dict):
    """(a, b, c_turn) from a fit_power_law() entry: {"k", "c_turn"} or {"a", "b", "c_turn"}."""
    return (law["k"], 3.0, law["c_turn"]) if "k" in law else (law["a"], law["b"], law["c_turn"])


def current_model(strategy_path=None, boat_config=None):
    """The model the strategy uses now: P_elec(v) = R_ShipFlow(v, mass) * v / eta_grid(v),
    from Strategy_EnduranceRace with config/boat_2026.json applied. None if the repo is missing."""
    path = Path(strategy_path or DEFAULT_STRATEGY_PATH)
    if not (path / "boat" / "physical_model.py").exists():
        return None
    sys.path.insert(0, str(BASE_DIR / "src"))
    from strategy_engine import import_team_strategy, load_boat_config
    team = import_team_strategy(path, load_boat_config(boat_config or BASE_DIR / "config" / "boat_2026.json"))
    eff, res = team["efficiency"], team["resistance"]
    return lambda u, r: np.array([float(res(x)) * x / float(eff(x)) for x in np.atleast_1d(u)])


def power_error(day, model) -> dict:
    """Predict P from the measured u and r on the propulsion rows; RMSE, bias, energy, range error."""
    ok = propulsion(day)
    P, u, r = day["motor_power"][ok], day["gps_speed"][ok], day["r"][ok]
    pred = model(u, r)
    e_meas, e_pred = np.sum(P) * DT, np.sum(pred) * DT
    return {"rmse_w": float(np.sqrt(np.mean((pred - P) ** 2))), "bias_w": float(np.mean(pred - P)),
            "energy_meas_mj": e_meas / 1e6, "energy_pred_mj": e_pred / 1e6,
            # range the model would promise on the measured energy, vs the distance actually sailed
            "range_error_pct": float((e_meas / e_pred - 1) * 100), "n": int(ok.sum())}


def forward_replay(day, model, mass_kg, eta, m11=0.08, substeps=8) -> dict:
    """Speed from the measured power: m_eff du/dt = eta (P - P_model(u, r)) / u, integrated over
    every contiguous propulsion stretch, starting from the measured speed. RMSE, bias, distance."""
    ok = propulsion(day)
    P, um, r = day["motor_power"], day["gps_speed"], day["r"]
    m_eff = mass_kg * (1 + m11)
    pred = np.full(len(P), np.nan)
    i, n = 0, len(P)
    while i < n:
        if not ok[i]:
            i += 1
            continue
        j = i
        while j < n and ok[j]:
            j += 1
        u = um[i]
        for t in range(i, j):
            pred[t] = u
            h = DT / substeps
            for _ in range(substeps):
                u = max(u + h * eta * (P[t] - float(model(np.array([u]), np.array([r[t]]))[0])) / (m_eff * u), 0.3)
        i = j
    sel = ok & np.isfinite(pred)
    d_meas, d_pred = np.sum(um[sel]) * DT, np.sum(pred[sel]) * DT
    return {"rmse_mps": float(np.sqrt(np.mean((pred[sel] - um[sel]) ** 2))),
            "bias_mps": float(np.mean(pred[sel] - um[sel])),
            "distance_meas_km": d_meas / 1000, "distance_error_pct": float((d_pred / d_meas - 1) * 100)}


def k_drift(day, b, c_turn, parts=4) -> list:
    """The coefficient a (k for b = 3) refitted on each quarter of the day with b and c_turn held:
    the paper finds k rising ~10 % as the battery empties."""
    ok = propulsion(day)
    idx = np.flatnonzero(ok)
    out = []
    for chunk in np.array_split(idx, parts):
        P, u, r = (day[x][chunk] for x in ("motor_power", "gps_speed", "r"))
        y = P - c_turn * r ** 2 * u
        kk = float(np.sum(y * u ** b) / np.sum(u ** (2 * b)))
        soc = np.nanmean(np.fmin(day.get("battery_1_soc", np.full(len(ok), np.nan))[chunk],
                                 day.get("battery_2_soc", np.full(len(ok), np.nan))[chunk]))
        out.append({"k": kk, "mean_soc_pct": None if np.isnan(soc) else float(soc)})
    return out


# ============================================================ CLI

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    ap.add_argument("--fit", nargs="+", default=["endurance"], choices=DAYS, help="days to fit on")
    ap.add_argument("--test", nargs="+", default=["lapstest"], choices=DAYS, help="blind days")
    ap.add_argument("--battery-days", nargs="+", default=["endurance", "lapstest", "speedrecord"],
                    choices=DAYS, help="days for the battery fit (speedrecord has the high currents)")
    ap.add_argument("--boat-config", type=Path, default=BASE_DIR / "config" / "boat_2026.json")
    ap.add_argument("--strategy-path", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=BASE_DIR / "config" / "surge_model_2026.json")
    a = ap.parse_args(argv)

    boat = json.loads(a.boat_config.read_text())
    mass, eta = float(boat["mass_kg"]), float(boat["eta"])
    cache = {}

    def day(name):
        if name not in cache:
            print(f"loading {DAYS[name]} ...", flush=True)
            cache[name] = prepare(a.data_dir / DAYS[name])
        return cache[name]

    # ---- fit
    tabs = []
    for name in a.fit:
        d = day(name)
        segs = plateaus(d)
        tabs.append(plateau_table(d, segs))
        print(f"  {name}: {propulsion(d).sum()} s of propulsion, {len(segs)} plateaus "
              f"({sum(j - i for i, j in segs)} s), yaw rate from {d['yaw_source']}")
    fit = fit_power_law(merge_tables(tabs))
    cu, fr = fit["cubic"], fit["free"]
    print(f"\nfitted on {fit['n_plateaus']} plateaus, u {fit['u_range_mps'][0]:.1f}-{fit['u_range_mps'][1]:.1f} m/s")
    print(f"  cubic (paper): P = {cu['k']:.2f} u^3 + {cu['c_turn']:.0f} r^2 u   R2 = {cu['r2']:.3f}"
          f"   (k on straight plateaus only: {cu.get('k_straight_only', float('nan')):.2f})")
    print(f"  free exponent: P = {fr['a']:.1f} u^{fr['b']:.2f} + {fr['c_turn']:.0f} r^2 u   R2 = {fr['r2']:.3f}"
          f"   (the strategy notes have 88 u^2.29)")
    for lname, law in (("cubic", cu), ("free", fr)):
        A, B, C = law_params(law)
        print(f"  {lname}: a 20 m radius turn at 4.5 m/s costs {C * (4.5 / 20) ** 2 * 4.5:.0f} W "
              f"on top of {A * 4.5 ** B:.0f} W straight")

    battery = fit_battery([day(n) for n in a.battery_days])
    for p, bat in battery.items():
        print(f"  battery {p}: R0 = {bat['R0_ohm'] * 1000:.1f} mOhm, OCV fit RMSE {bat['rmse_v']:.3f} V, "
              f"Q_eff = {bat.get('Q_eff_ah', float('nan')):.1f} Ah ({bat.get('Q_eff_from', '-')})")

    # ---- blind validation
    cur = current_model(a.strategy_path, a.boat_config)
    models = {}
    for lname, law in (("cubic", cu), ("free", fr)):
        A, B, C = law_params(law)
        models[f"new: {lname}"] = (lambda A, B, C: lambda u, r: p_surge(u, r, A, B, C))(A, B, C)
        models[f"new: {lname}, no turn term"] = (lambda A, B: lambda u, r: A * u ** B)(A, B)
    models["88 u^2.29 (strategy notes)"] = lambda u, r: 88.0 * u ** 2.29
    if cur is not None:
        models["now: ShipFlow / eta_grid"] = cur
    else:
        print("  (Strategy_EnduranceRace not found: no comparison with the current model)")
    validation = {}
    for name in a.fit + a.test:
        d = day(name)
        tag = "fit" if name in a.fit else "blind"
        validation[name] = {"role": tag}
        print(f"\n{name} ({tag}, {propulsion(d).sum()} s):")
        print(f"  {'model':30s} {'P RMSE':>7s} {'P bias':>7s} {'range err':>9s} {'u RMSE':>7s} {'u bias':>7s} {'dist err':>8s}")
        for mname, m in models.items():
            pe, fw = power_error(d, m), forward_replay(d, m, mass, eta)
            validation[name][mname] = {"power": pe, "speed": fw}
            print(f"  {mname:30s} {pe['rmse_w']:6.0f}W {pe['bias_w']:+6.0f}W {pe['range_error_pct']:+8.1f}% "
                  f"{fw['rmse_mps']:6.2f}  {fw['bias_mps']:+6.2f}  {fw['distance_error_pct']:+7.1f}%")

    # the law to use: the lower power RMSE on the blind days (on the fit days if there are none)
    judge = [n for n in a.test if n not in a.fit] or a.fit
    score = {lname: np.mean([validation[n][f"new: {lname}"]["power"]["rmse_w"] for n in judge])
             for lname in ("cubic", "free")}
    selected = min(score, key=score.get)
    A, B, C = law_params(fit[selected])
    drift = {n: k_drift(day(n), B, C) for n in a.fit}
    print(f"\nselected: {selected} (blind power RMSE {score[selected]:.0f} W vs {max(score.values()):.0f} W)")
    print(f"  P = {A:.2f} u^{B:.2f} + {C:.0f} r^2 u")
    for n, parts in drift.items():
        print(f"  a per quarter of {n}: " + ", ".join(
            f"{p['k']:.1f}" + (f" (SOC {p['mean_soc_pct']:.0f} %)" if p["mean_soc_pct"] is not None else "")
            for p in parts))

    out = {
        "_source": "src/surge_model.py, after 'Data-driven parametrization of an energy-propulsive model "
                   "for an electric racing vessel' (Oct 2026 draft)",
        "_model": "P_motor [W] = a * u^b + c_turn * r^2 * u, u GPS speed [m/s], r yaw rate [rad/s]",
        "law": selected, "a": A, "b": B, "c_turn": C,
        "valid_u_range_mps": fit["u_range_mps"],
        "fit": fit, "fit_days": a.fit, "test_days": a.test,
        "plateau_rule": {"min_s": PLATEAU_MIN_S, "tol_w": PLATEAU_TOL_W, "tol_rel": PLATEAU_TOL_REL,
                         "smooth_s": PLATEAU_SMOOTH_S, "p_min_w": P_MIN_W, "u_min_mps": U_MIN,
                         "dt_s": DT, "max_gap_s": MAX_GAP_S},
        "battery": battery, "a_drift": drift, "validation": validation,
    }
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, indent=2))
    print(f"\nwrote {a.out}")


if __name__ == "__main__":
    main()
