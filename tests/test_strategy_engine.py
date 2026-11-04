"""Run from the repo root: python -m unittest discover -s tests"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from strategy_engine import RaceConfig, StrategyEngine, TelemetryState, parse_log_line

START = 1_000_000.0


def config(**kw):
    base = dict(start=START, duration_s=3600, battery_wh=5000, soc_reserve=0.05,
                p_max_w=10_000, band_w=500, stale_s=10)
    base.update(kw)
    return RaceConfig(**base)


def engine_with(soc_pct, t, cfg=None, power=None):
    state = TelemetryState()
    for n in (1, 2):
        data = {"soc": soc_pct}
        if power is not None:
            data["power"] = power
        state.update(f"battery/{n}", data, t)
    return StrategyEngine(cfg or config(), state)


class SimpleControllerTest(unittest.TestCase):
    def test_spreads_usable_energy_over_time_left(self):
        # 50 % of 5000 Wh, 5 % reserve -> 2250 Wh over 1 h -> 2250 W
        rec = engine_with(50, START).step(START)
        self.assertEqual(rec.controller, "simple")
        self.assertAlmostEqual(rec.p_w, 2250)
        self.assertEqual(rec.payload(), {"powerMin": 1.75, "powerMax": 2.75})

    def test_half_time_left_doubles_power(self):
        t = START + 1800
        rec = engine_with(50, t).step(t)
        self.assertAlmostEqual(rec.p_w, 4500)

    def test_capped_at_p_max(self):
        rec = engine_with(100, START, config(p_max_w=3000)).step(START)
        self.assertEqual(rec.p_w, 3000)
        self.assertEqual(rec.p_max_w, 3000)

    def test_below_reserve_gives_zero(self):
        rec = engine_with(4, START).step(START)
        self.assertEqual(rec.p_w, 0)
        self.assertEqual(rec.p_min_w, 0)

    def test_ramp_starts_from_measured_power(self):
        eng = engine_with(50, START, config(ramp_w_per_s=100), power=500)  # 2 packs -> 1000 W
        self.assertAlmostEqual(eng.step(START).p_w, 1000)
        self.assertAlmostEqual(eng.step(START + 1).p_w, 1100)
        self.assertAlmostEqual(eng.step(START + 2).p_w, 1200)


class EngineTest(unittest.TestCase):
    def test_stale_soc_gives_no_recommendation(self):
        rec = engine_with(50, START).step(START + 11)
        self.assertFalse(rec.ok)
        self.assertIn("stale", rec.reason)

    def test_missing_pack_gives_no_recommendation(self):
        state = TelemetryState()
        state.update("battery/1", {"soc": 50}, START)
        self.assertFalse(StrategyEngine(config(), state).step(START).ok)

    def test_outside_race(self):
        self.assertEqual(engine_with(50, START - 5).step(START - 5).reason, "race not started")
        t = START + 3600
        self.assertEqual(engine_with(50, t).step(t).reason, "race finished")


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
