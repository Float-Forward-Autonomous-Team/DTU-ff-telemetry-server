"""Run from the repo root: python -m unittest discover -s tests

Needs numpy and scipy, and the Strategy_EnduranceRace repo ($STRATEGY_PATH or
../../Strategy_EnduranceRace).
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from strategy_engine import (DEFAULT_BOAT_CONFIG, RaceConfig, StrategyEngine, TelemetryState,
                             import_team_strategy, load_boat_config, parse_log_line)

START = 1_000_000.0
BOAT = load_boat_config(DEFAULT_BOAT_CONFIG)
TEAM = import_team_strategy(boat=BOAT)


def config(**kw):
    base = dict(start=START, duration_s=3600, battery_wh=5000, soc_reserve=0.05,
                p_max_w=10_000, band_w=500, stale_s=10, ramp_w_per_s=0)
    base.update(kw)
    return RaceConfig(**base)


def feed(state, t, soc=50, speed=None, motor_power=None, **groups):
    for n in (1, 2):
        state.update(f"battery/{n}", {"soc": soc}, t)
    if speed is not None:
        state.update("gps", {"speed": speed}, t)
    if motor_power is not None:
        state.update("motor", {"power": motor_power}, t)
    for group, data in groups.items():
        state.update(group.replace("__", "/"), data, t)


def engine(cfg=None, **feed_kw):
    state = TelemetryState()
    feed(state, START, **feed_kw)
    return StrategyEngine(cfg or config(), state, TEAM), state


class InstantTest(unittest.TestCase):
    def test_spreads_usable_energy_over_time_left(self):
        # 50 % of 5000 Wh, 5 % reserve -> 2250 Wh over 1 h -> 2250 W
        rec = engine()[0].step(START)
        self.assertEqual(rec.controller, "instant")
        self.assertAlmostEqual(rec.p_w, 2250, places=0)
        self.assertEqual(rec.payload(), {"powerMin": 1.75, "powerMax": 2.75})

    def test_matches_team_recommend(self):
        eng, _ = engine(config(eta=0.4, mass_kg=300), speed=4.0, motor_power=1500)
        rec = eng.step(START)
        ref = TEAM["recommend"](4.0, 1500, 2500, 5000, 0.05, 3600, 0.4, 300, 10_000)
        self.assertAlmostEqual(rec.p_w, ref["p_elec_rec_w"], places=0)
        self.assertEqual(rec.info["v_rec_mps"], ref["v_rec_mps"])
        self.assertEqual(rec.info["speed_status"], ref["speed_status"])

    def test_capped_at_p_max(self):
        rec = engine(config(p_max_w=1000), soc=100)[0].step(START)
        self.assertEqual(rec.p_w, 1000)
        self.assertEqual(rec.p_max_w, 1000)

    def test_ramp_starts_from_measured_power(self):
        eng, state = engine(config(ramp_w_per_s=800), motor_power=1000)
        self.assertAlmostEqual(eng.step(START).p_w, 1000)
        feed(state, START + 1, motor_power=1000)
        self.assertAlmostEqual(eng.step(START + 1).p_w, 1800)
        feed(state, START + 2, motor_power=1000)
        self.assertAlmostEqual(eng.step(START + 2).p_w, 2250, delta=3)

    def test_speed_in_kmh(self):
        eng, _ = engine(config(speed_to_mps=1 / 3.6), speed=36.0)
        self.assertAlmostEqual(eng.step(START).inputs["speed"], 10.0)


class SegMpcTest(unittest.TestCase):
    def run_laps(self, cfg, laps, speed=4.0, power=2000.0):
        eng, state = engine(cfg, speed=speed, motor_power=power)
        recs, t = [], START
        seconds = int(laps * cfg.lap_len_m / speed) + 2
        for k in range(seconds):
            t = START + k
            feed(state, t, soc=50 - 0.001 * k, speed=speed, motor_power=power)
            recs.append(eng.step(t))
        return recs

    def test_calibration_lap_uses_average_power(self):
        cfg = config(mode="mpc")
        rec = self.run_laps(cfg, 0.5)[-1]
        self.assertEqual(rec.controller, "mpc")
        self.assertEqual(rec.info["is_calib"], 1)
        self.assertAlmostEqual(rec.p_w, 2250, delta=5)

    def test_replans_after_a_lap(self):
        cfg = config(mode="mpc")
        recs = self.run_laps(cfg, 1.2)
        last = recs[-1]
        self.assertEqual(last.info["laps"], 1)
        self.assertEqual(last.info["is_calib"], 0)
        targets = [last.info[f"lap_target_{i}_w"] for i in range(4)]
        self.assertTrue(all(0 <= p <= cfg.p_max_w for p in targets))

    def test_loadcell_ignored_unless_enabled(self):
        eng, _ = engine(config(mode="mpc"), thrust={"loadcell_n": 9440})
        self.assertNotIn("thrust", eng.step(START).inputs)
        eng, _ = engine(config(mode="mpc", use_loadcell=True), thrust={"loadcell_n": 9440})
        self.assertEqual(eng.step(START).inputs["thrust"], 9440)

    def test_imu_accel_from_batch(self):
        batch = {"samples": [{"ax": 0.1}, {"ax": 0.3}]}
        eng, _ = engine(config(mode="mpc", accel_source="imu"), imu__batch=batch)
        self.assertAlmostEqual(eng.step(START).inputs["accel"], 0.2)


class EngineTest(unittest.TestCase):
    def test_uses_lowest_pack(self):
        state = TelemetryState()
        state.update("battery/1", {"soc": 30}, START)
        state.update("battery/2", {"soc": 50}, START)
        rec = StrategyEngine(config(), state, TEAM).step(START)
        self.assertAlmostEqual(rec.inputs["soc"], 0.30)
        self.assertAlmostEqual(rec.inputs["soc_mean"], 0.40)
        # 30 % of 5000 Wh - 5 % reserve = 1250 Wh over 1 h
        self.assertAlmostEqual(rec.p_w, 1250, places=0)

    def test_stale_soc_gives_no_recommendation(self):
        rec = engine()[0].step(START + 11)
        self.assertFalse(rec.ok)
        self.assertIn("stale", rec.reason)

    def test_missing_pack_gives_no_recommendation(self):
        state = TelemetryState()
        state.update("battery/1", {"soc": 50}, START)
        self.assertFalse(StrategyEngine(config(), state, TEAM).step(START).ok)

    def test_outside_race(self):
        self.assertEqual(engine()[0].step(START - 5).reason, "race not started")
        self.assertEqual(engine()[0].step(START + 3600).reason, "race finished")


class BoatConfigTest(unittest.TestCase):
    def test_boat_config_applied_to_physical_model(self):
        import boat.physical_model as pm
        self.assertEqual(pm.M_BOAT, BOAT["mass_kg"])
        for v, eta in zip(pm.V_grid, BOAT["eta_grid"]):
            self.assertAlmostEqual(pm.efficiency(float(v)), eta, places=6)
        # power <-> speed still invertible with the measured curve
        self.assertAlmostEqual(pm.power_to_speed(pm.speed_to_power(5.0)), 5.0, delta=0.02)


class LogParseTest(unittest.TestCase):
    def test_json_group_line(self):
        line = ("\x1b[2m15:20:49\x1b[0m  \x1b[33mbattery/1                          \x1b[0m "
                "voltage=49.5 current=0.0 power=0.0 soc=19.6 status=stationary ts=2026-07-10T13:20:48.765Z")
        t, group, data = parse_log_line(line)
        self.assertEqual(group, "battery/1")
        self.assertEqual(data["soc"], "19.6")
        self.assertAlmostEqual(t % 60, 48.765, places=3)

    def test_per_field_line(self):
        t, group, data = parse_log_line("15:39:36.024  battery/1/soc                       83.6")
        self.assertEqual((group, data), ("battery/1", {"soc": "83.6"}))

    def test_other_lines_ignored(self):
        self.assertIsNone(parse_log_line("15:20:21  Ignored unknown topic: boat/telemetry/x"))
        self.assertIsNone(parse_log_line("Connected: Success"))


if __name__ == "__main__":
    unittest.main()
